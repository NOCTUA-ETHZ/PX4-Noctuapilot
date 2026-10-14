// Copyright (c) 2026 Noctua. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include <atomic>
#include <cmath>
#include <cstring>

#include <drivers/drv_sensor.h>
#include <drivers/fuel_cell/Telemetry.hpp>
#include <drivers/uavcan/fuel_cell/Protocol.hpp>
#include <lib/drivers/device/Device.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_gas_concentration.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class NoctuaTelemetrySim : public ModuleBase<NoctuaTelemetrySim>, public px4::ScheduledWorkItem
{
public:
	NoctuaTelemetrySim() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

private:
	void Run() override;

	uORB::Publication<fuel_cell_s> _fuel_cell_pub{ORB_ID(fuel_cell)};
	uORB::Publication<fuel_cell_can_s> _can_pub{ORB_ID(fuel_cell_can)};
	uORB::PublicationMulti<sensor_gas_concentration_s> _gas_pub{ORB_ID(sensor_gas_concentration)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	std::atomic<uint8_t> _source{1};
	std::atomic<bool> _paused{false};
	uint64_t _last_update{0};
	float _elapsed{0.f};
	float _remaining{82.f};
	uint32_t _frames{0};
	bool _armed{false};
};

void NoctuaTelemetrySim::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	const uint64_t now = hrt_absolute_time();
	const float dt = _last_update ? static_cast<float>(now - _last_update) * 1e-6f : 0.f;
	_last_update = now;

	if (_paused.load()) {
		return; // No publications: QGC must expire all three streams naturally.
	}

	vehicle_status_s vehicle_status{};

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	}

	_elapsed += dt;
	_remaining = fmaxf(5.f, _remaining - dt * (_armed ? 0.02f : 0.005f));
	const float wave = sinf(_elapsed * 0.2f);
	const float voltage = 48.f + 0.2f * wave;
	const float power = (_armed ? 650.f : 200.f) + 20.f * wave;
	const float stack_power = power + 15.f;
	const float battery_power = -20.f + 10.f * wave;

	// Synthetic CAN payload follows the existing decoder's layout and quantization.
	const uint16_t pressure_raw = static_cast<uint16_t>(_remaining * 6.f);
	const uint16_t voltage_raw = static_cast<uint16_t>(voltage * 10.f);
	const uint16_t power_raw = static_cast<uint16_t>(power / 10.f);
	const uint16_t stack_raw = static_cast<uint16_t>(stack_power / 5.f);
	const uint16_t battery_raw = static_cast<uint16_t>((battery_power + 5000.f) / 10.f);
	fuel_cell_can_s can{};
	can.timestamp = can.timestamp_sample = now;
	can.interface = 1;
	can.connected = true;
	can.received_frames = ++_frames;
	can.raw_data[0] = static_cast<uint8_t>(((_frames & 0x0fu) << 4) | 2u);
	can.raw_data[1] = static_cast<uint8_t>(pressure_raw >> 2);
	can.raw_data[2] = static_cast<uint8_t>((pressure_raw << 6) | (voltage_raw >> 4));
	can.raw_data[3] = static_cast<uint8_t>((voltage_raw << 4) | (power_raw >> 6));
	can.raw_data[4] = static_cast<uint8_t>((power_raw << 2) | (stack_raw >> 8));
	can.raw_data[5] = static_cast<uint8_t>(stack_raw);
	can.raw_data[6] = static_cast<uint8_t>(battery_raw >> 3);
	can.raw_data[7] = static_cast<uint8_t>(battery_raw << 5);
	fuelcell_can::Status decoded{};

	if (!fuelcell_can::decode(fuelcell_can::StatusId, true, false, false, can.raw_data, sizeof(can.raw_data), decoded)) {
		return;
	}

	can.tank_pressure_bar = decoded.tank_pressure_bar;
	can.battery_voltage_v = decoded.battery_voltage_v;
	can.output_power_w = decoded.output_power_w;
	can.stack_power_w = decoded.stack_power_w;
	can.battery_power_w = decoded.battery_power_w;
	can.counter = decoded.counter;
	can.state = decoded.state;
	can.error_code = decoded.error;
	_can_pub.publish(can);

	const uint8_t source = _source.load();
	auto fuel_cell = fuelcell::unavailable(source, now, now);

	if (source == 2) {
		// Reuse the driver's validated MCU v3.63 CAN power mapping.
		fuel_cell = fuelcell::from_can(can, source, now, 2_s);

	} else {
		fuel_cell.connected = true;
		fuel_cell.tankpressure = _remaining;
		fuel_cell.regpressure = 0.8f + 0.01f * wave;
		fuel_cell.voltage = voltage;
		fuel_cell.outputpower = power;
		fuel_cell.spmpower = stack_power;
		fuel_cell.battpower = battery_power;
		fuel_cell.psustate = 2;
		fuel_cell.mainerror = fuel_cell.suberror = 0;
	}

	_fuel_cell_pub.publish(fuel_cell);

	device::Device::DeviceId device_id{};
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SIMULATION;
	device_id.devid_s.address = 1;
	device_id.devid_s.devtype = DRV_GAS_DEVTYPE_DFROBOT_GAS;
	sensor_gas_concentration_s gas{};
	gas.timestamp = gas.timestamp_sample = now;
	gas.device_id = device_id.devid;
	gas.gas_type = sensor_gas_concentration_s::GAS_TYPE_H2;
	gas.concentration_ppm = 1.2f + 0.4f * sinf(_elapsed * 0.3f);
	gas.temperature = 24.f + 1.5f * sinf(_elapsed * 0.05f);
	_gas_pub.publish(gas);
}

int NoctuaTelemetrySim::task_spawn(int argc, char *argv[])
{
	auto *instance = new NoctuaTelemetrySim();

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleOnInterval(500_ms);
	PX4_WARN("SIMULATED fuel-cell and H2-probe data enabled (UART mode)");
	return PX4_OK;
}

int NoctuaTelemetrySim::custom_command(int argc, char *argv[])
{
	auto *instance = get_instance();

	if (!instance) {
		PX4_ERR("start the simulator first");
		return PX4_ERROR;
	}

	if (argc == 2 && strcmp(argv[0], "source") == 0) {
		if (strcmp(argv[1], "uart") == 0 || strcmp(argv[1], "can") == 0) {
			instance->_source.store(strcmp(argv[1], "can") == 0 ? 2 : 1);
			return PX4_OK;
		}

	} else if (argc == 1 && (strcmp(argv[0], "pause") == 0 || strcmp(argv[0], "resume") == 0)) {
		instance->_paused.store(strcmp(argv[0], "pause") == 0);
		return PX4_OK;
	}

	return print_usage("unknown command");
}

int NoctuaTelemetrySim::print_status()
{
	PX4_INFO("SIMULATED telemetry: %s, %s", _source.load() == 2 ? "CAN1" : "UART",
		 _paused.load() ? "paused" : "publishing at 2 Hz");
	return PX4_OK;
}

int NoctuaTelemetrySim::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("SITL-only demo data; not a physical fuel-cell or leak model.\n"
				 "source uart|can selects real transport semantics; pause/resume tests telemetry loss.");
	PRINT_MODULE_USAGE_NAME("noctua_telemetry_sim", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("source", "Select uart or can");
	PRINT_MODULE_USAGE_ARG("uart|can", "Fuel-cell transport to simulate", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("pause", "Stop publishing simulated samples");
	PRINT_MODULE_USAGE_COMMAND_DESCR("resume", "Resume publishing simulated samples");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return PX4_OK;
}

extern "C" __EXPORT int noctua_telemetry_sim_main(int argc, char *argv[])
{
	return NoctuaTelemetrySim::main(argc, argv);
}
