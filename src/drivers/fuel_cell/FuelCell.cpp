// Copyright (c) 2026 PX4 Development Team. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
#include "Telemetry.hpp"
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/getopt.h>
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdio>

class FuelCell : public ModuleBase<FuelCell>, public px4::ScheduledWorkItem
{
public:
	FuelCell(int32_t source, int32_t timeout, int32_t baud, const char *device) :
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
		_source(source), _timeout_us(uint64_t(timeout) * 1000), _baud(baud)
	{
		if (device) { strncpy(_device, device, sizeof(_device) - 1); }

		_status = fuelcell::unavailable(_source, hrt_absolute_time());
	}

	~FuelCell() override { if (_fd >= 0) { close(_fd); } }
	static int task_spawn(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int custom_command(int, char *[]) { return print_usage("unknown command"); }
	int print_status() override;

private:
	void Run() override;
	bool open_uart();
	void publish(const fuel_cell_s &sample) { _status = sample; _pub.publish(_status); }
	const uint8_t _source;
	const uint64_t _timeout_us;
	const int32_t _baud;
	char _device[32]{};
	int _fd{-1};
	uint64_t _retry_at{0};
	bool _started{false};
	fuelcell::SerialParser _parser;
	fuel_cell_s _status{};
	fuel_cell_can_s _can{};
	uORB::Publication<fuel_cell_s> _pub{ORB_ID(fuel_cell)};
	uORB::Subscription _can_sub{ORB_ID(fuel_cell_can)};
};

bool FuelCell::open_uart()
{
	speed_t speed = B9600;

	switch (_baud) {
	case 9600: speed = B9600; break;
	case 19200: speed = B19200; break;
	case 38400: speed = B38400; break;
	case 57600: speed = B57600; break;
	case 115200: speed = B115200; break;
	default: return false;
	}

	_fd = open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) { return false; }

	termios tty{};

	if (tcgetattr(_fd, &tty) != 0) { close(_fd); _fd = -1; return false; }

	cfmakeraw(&tty);
	tty.c_cflag = (tty.c_cflag & ~(CSIZE | PARENB | CSTOPB | CRTSCTS)) | CS8 | CLOCAL | CREAD;
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 0;
	cfsetispeed(&tty, speed);
	cfsetospeed(&tty, speed);

	if (tcsetattr(_fd, TCSANOW, &tty) != 0) { close(_fd); _fd = -1; return false; }

	_parser = fuelcell::SerialParser{};
	return true;
}

void FuelCell::Run()
{
	const uint64_t now = hrt_absolute_time();

	if (should_exit()) {
		publish(fuelcell::unavailable(_source, now, _status.timestamp_sample));
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (!_started) { publish(_status); _started = true; }

	if (_source == 1) {
		if (_fd < 0 && now >= _retry_at && !open_uart()) {
			PX4_WARN("Cannot open %s; retrying", _device);
			_retry_at = now + 2000000;
		}

		if (_fd >= 0) {
			char bytes[256];
			// Bounded, nonblocking reads: never stall a PX4 work queue.
			for (unsigned batch = 0; batch < 4; ++batch) {
				const ssize_t n = read(_fd, bytes, sizeof(bytes));

				if (n <= 0) {
					if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
						close(_fd); _fd = -1; _retry_at = now + 2000000;
					}

					break;
				}

				for (ssize_t i = 0; i < n; ++i) {
					fuel_cell_s sample{};

					if (_parser.feed(bytes[i], sample, now)) { publish(sample); }
				}
			}
		}

	} else if (_can_sub.update(&_can)) {
		// Check the selected interface even if a previous backend published.
		if (_can.interface == _source - 1) {
			publish(fuelcell::from_can(_can, _source, now, _timeout_us));
		}
	}

	// Also detects loss of the whole UAVCAN module, not just CAN RX silence.
	if (_status.connected && !fuelcell::fresh(_status.timestamp_sample, now, _timeout_us)) {
		publish(fuelcell::unavailable(_source, now, _status.timestamp_sample));
	}

	ScheduleDelayed(20000);
}

int FuelCell::task_spawn(int argc, char *argv[])
{
	int32_t source = 0, timeout = 2000, baud = 9600;
	param_get(param_find("FC_INTERFACE"), &source);
	param_get(param_find("FC_TIMEOUT"), &timeout);
	param_get(param_find("FC_BAUD"), &baud);
	const char *device = nullptr;
	int ch, index = 1;
	const char *arg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &index, &arg)) != EOF) {
		if (ch == 'd') { device = arg; } else { return print_usage("invalid option"); }
	}

	if (source < 1 || source > 3) { return print_usage("Set FC_INTERFACE (1 UART, 2 CAN1, 3 CAN2)"); }

	if (timeout < 200 || timeout > 5000) { return print_usage("Invalid FC_TIMEOUT"); }

	if (source == 1 && (!device || strlen(device) >= 32)) { return print_usage("UART requires -d device or FC_SER_CFG"); }

	if (source == 1 && baud != 9600 && baud != 19200 && baud != 38400 && baud != 57600 && baud != 115200) {
		return print_usage("Invalid FC_BAUD");
	}

#if !defined(CONFIG_UAVCAN_FUEL_CELL)
	if (source > 1) { return print_usage("CAN backend not included in this build"); }
#endif
	FuelCell *instance = new FuelCell(source, timeout, baud, device);

	if (!instance) { return PX4_ERROR; }

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleNow();
	return PX4_OK;
}

int FuelCell::print_status()
{
	// Read uORB snapshots rather than racing the work-queue state.
	auto status = fuelcell::unavailable(_source, hrt_absolute_time());
	fuel_cell_can_s can{};
	uORB::Subscription status_sub{ORB_ID(fuel_cell)};
	uORB::Subscription can_sub{ORB_ID(fuel_cell_can)};
	status_sub.copy(&status);
	can_sub.copy(&can);
	PX4_INFO("%s: %s", _source == 1 ? "UART" : (_source == 2 ? "CAN1" : "CAN2"),
		 status.connected ? "connected" : "waiting/stale");

	if (_source == 1) { PX4_INFO("%s, %ld baud", _device, (long)_baud); }
	else {
		PX4_INFO("RX %lu, repeated counter %lu, counter %u, raw error %u",
			 (unsigned long)can.received_frames, (unsigned long)can.duplicate_frames,
			 unsigned(can.counter), unsigned(can.error_code));
		printf("  CAN payload:");
		for (uint8_t b : can.raw_data) { printf(" %02x", unsigned(b)); }
		printf("\n");
		PX4_INFO("Unverified CAN power W: output %.1f, stack %.1f, battery %.1f",
			 (double)can.output_power_w, (double)can.stack_power_w, (double)can.battery_power_w);
	}

	return PX4_OK;
}

int FuelCell::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s", reason); }

	PRINT_MODULE_USAGE_NAME("fuel_cell", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "device", "UART device (selected by FC_SER_CFG at boot)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return PX4_OK;
}

extern "C" __EXPORT int fuel_cell_main(int argc, char *argv[])
{
	return FuelCell::main(argc, argv);
}
