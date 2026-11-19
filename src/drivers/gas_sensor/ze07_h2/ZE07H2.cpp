// Copyright (c) 2026 PX4 Development Team. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include "Protocol.hpp"

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/perf/perf_counter.h>
#include <parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_gas_concentration.h>

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace time_literals;

class ZE07H2 : public ModuleBase<ZE07H2>, public px4::ScheduledWorkItem
{
public:
	ZE07H2(const char *port, int32_t warmup_s) :
		ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
		_warmup_us(uint64_t(warmup_s) * 1000000)
	{
		strncpy(_port, port, sizeof(_port) - 1);
		device::Device::DeviceId id{};
		id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
		id.devid_s.devtype = DRV_GAS_DEVTYPE_ZE07_H2;

		// Use the complete numeric suffix (not only the last digit).
		const char *suffix = _port + strlen(_port);

		while (suffix > _port && suffix[-1] >= '0' && suffix[-1] <= '9') { --suffix; }

		id.devid_s.bus = strtoul(suffix, nullptr, 10) & 0x1f;
		_device_id = id.devid;
	}

	~ZE07H2() override
	{
		ScheduleClear();
		close_uart();
		perf_free(_bad_packets);
		perf_free(_io_errors);
		perf_free(_measurements);
	}

	static int task_spawn(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int custom_command(int, char *[]) { return print_usage("unknown command"); }
	int print_status() override;

private:
	void Run() override;
	bool open_uart();
	void close_uart();
	void request_upload();

	char _port[32] {};
	uint32_t _device_id{0};
	const uint64_t _warmup_us;
	int _fd{-1};
	uint64_t _retry_at{0};
	uint64_t _ready_at{0};
	uint64_t _last_byte{0};
	uint64_t _last_valid{0};
	uint64_t _last_mode_request{0};
	size_t _tx_offset{0};
	bool _tx_pending{false};
	ze07_h2::Parser _parser;
	uORB::PublicationMulti<sensor_gas_concentration_s> _pub{ORB_ID(sensor_gas_concentration)};
	perf_counter_t _bad_packets{perf_alloc(PC_COUNT, MODULE_NAME ": invalid packets")};
	perf_counter_t _io_errors{perf_alloc(PC_COUNT, MODULE_NAME ": UART errors")};
	perf_counter_t _measurements{perf_alloc(PC_COUNT, MODULE_NAME ": valid packets")};
};

void ZE07H2::close_uart()
{
	if (_fd >= 0) { close(_fd); _fd = -1; }

	_parser.reset();
	_tx_pending = false;
}

bool ZE07H2::open_uart()
{
	_fd = open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) { return false; }

	termios tty{};

	if (tcgetattr(_fd, &tty) != 0) { close_uart(); return false; }

	cfmakeraw(&tty);
	tty.c_cflag = (tty.c_cflag & ~(CSIZE | PARENB | CSTOPB | CRTSCTS)) | CS8 | CLOCAL | CREAD;
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 0;

	if (cfsetispeed(&tty, B9600) != 0 || cfsetospeed(&tty, B9600) != 0
	    || tcsetattr(_fd, TCSANOW, &tty) != 0 || tcflush(_fd, TCIFLUSH) != 0) {
		close_uart();
		return false;
	}

	const uint64_t now = hrt_absolute_time();
	_ready_at = now + _warmup_us;
	_last_valid = now;
	_last_byte = 0;
	_last_mode_request = now;
	_parser.reset();
	_tx_offset = 0;
	_tx_pending = true;
	return true;
}

void ZE07H2::request_upload()
{
	if (!_tx_pending) { return; }

	const ssize_t n = write(_fd, ze07_h2::START_UPLOAD + _tx_offset, sizeof(ze07_h2::START_UPLOAD) - _tx_offset);

	if (n > 0) {
		_tx_offset += n;
		_tx_pending = _tx_offset < sizeof(ze07_h2::START_UPLOAD);

	} else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
		perf_count(_io_errors);
		close_uart();
		_retry_at = hrt_absolute_time() + 2_s;
	}
}

void ZE07H2::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	uint64_t now = hrt_absolute_time();

	if (_fd < 0) {
		if (now < _retry_at) { ScheduleDelayed(100_ms); return; }

		if (!open_uart()) {
			perf_count(_io_errors);
			_retry_at = now + 2_s;
			ScheduleDelayed(100_ms);
			return;
		}
	}

	now = hrt_absolute_time();
	request_upload();

	if (_fd < 0) { ScheduleDelayed(100_ms); return; }

	// At 9600 baud a frame takes about 10 ms. Discard partial frames after
	// a long gap so bytes from different sensor cycles cannot be combined.
	if (_last_byte && now - _last_byte > 200_ms) { _parser.reset(); }

	uint8_t bytes[64];

	// Bound work even if a different device floods this port.
	for (unsigned batch = 0; batch < 4; ++batch) {
		const ssize_t n = read(_fd, bytes, sizeof(bytes));

		if (n <= 0) {
			if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
				perf_count(_io_errors);
				close_uart();
				_retry_at = now + 2_s;
			}

			break;
		}

		_last_byte = now;

		for (ssize_t i = 0; i < n; ++i) {
			uint16_t ppm = 0;
			const auto result = _parser.feed(bytes[i], ppm);

			if (result == ze07_h2::Parser::Result::Invalid) {
				perf_count(_bad_packets);

			} else if (result == ze07_h2::Parser::Result::Measurement) {
				// A long outage may mean the sensor power-cycled. Warm up again.
				if (now - _last_valid > 5_s) { _ready_at = now + _warmup_us; }

				_last_valid = now;
				perf_count(_measurements);

				if (now >= _ready_at) {
					sensor_gas_concentration_s sample{};
					sample.timestamp_sample = now; // Receipt time; sensor has no clock.
					sample.device_id = _device_id;
					sample.concentration_ppm = ppm;
					sample.temperature = NAN; // Not exposed by the ZE07-H2 protocol.
					sample.gas_type = sensor_gas_concentration_s::GAS_TYPE_H2;
					sample.timestamp = hrt_absolute_time();
					_pub.publish(sample);
				}
			}
		}
	}

	// Recover a sensor left in Q&A mode or one that missed the first command.
	// Default-active sensors also work with only TX -> autopilot RX connected.
	if (_fd >= 0 && !_tx_pending && now - _last_valid > 5_s && now - _last_mode_request > 5_s) {
		_tx_offset = 0;
		_tx_pending = true;
		_last_mode_request = now;
	}

	// Do not republish old data or turn a communication failure into zero ppm.
	ScheduleDelayed(20_ms);
}

int ZE07H2::task_spawn(int argc, char *argv[])
{
	const char *port = nullptr;
	const char *arg = nullptr;
	int index = 1;
	int ch;

	while ((ch = px4_getopt(argc, argv, "d:", &index, &arg)) != EOF) {
		if (ch == 'd') { port = arg; } else { return print_usage("invalid option"); }
	}

	if (!port || !port[0] || strlen(port) >= 32) { return print_usage("requires -d serial device"); }

	int32_t warmup_s = 300;
	param_get(param_find("SENS_ZE07_WARM"), &warmup_s);

	if (warmup_s < 0 || warmup_s > 600) { return print_usage("invalid SENS_ZE07_WARM"); }

	auto *instance = new ZE07H2(port, warmup_s);

	if (!instance) { return PX4_ERROR; }

	if (!instance->open_uart()) {
		PX4_ERR("cannot configure %s", port);
		delete instance;
		return PX4_ERROR;
	}

	// Reserve a stable instance before starting the work queue.
	if (!instance->_pub.advertise()) { delete instance; return PX4_ERROR; }
	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleNow();
	return PX4_OK;
}

int ZE07H2::print_status()
{
	PX4_INFO("%s, 9600 baud, 8N1, device ID %lu", _port, (unsigned long)_device_id);
	// Copy the publication instead of reading mutable work-queue state.
	uORB::Subscription sub{ORB_ID(sensor_gas_concentration), uint8_t(_pub.get_instance())};
	sensor_gas_concentration_s sample{};

	if (sub.copy(&sample) && sample.timestamp != 0) {
		PX4_INFO("%.0f ppm H2, age %.1f s (%s)", (double)sample.concentration_ppm,
			 double(hrt_elapsed_time(&sample.timestamp)) / 1e6,
			 hrt_elapsed_time(&sample.timestamp) <= 5_s ? "fresh" : "stale/warming up");

	} else {
		PX4_INFO("waiting for valid data / warming up (%lu s configured)", (unsigned long)(_warmup_us / 1000000));
	}

	perf_print_counter(_measurements);
	perf_print_counter(_bad_packets);
	perf_print_counter(_io_errors);
	return PX4_OK;
}

int ZE07H2::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s", reason); }

	PRINT_MODULE_DESCRIPTION("UART driver for Winsen ZE07-H2. Publishes sensor_gas_concentration at approximately 1 Hz.");
	PRINT_MODULE_USAGE_NAME("ze07_h2", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "device", "Serial device (SENS_ZE07_CFG at boot)", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return reason ? PX4_ERROR : PX4_OK;
}

extern "C" __EXPORT int ze07_h2_main(int argc, char *argv[])
{
	return ZE07H2::main(argc, argv);
}
