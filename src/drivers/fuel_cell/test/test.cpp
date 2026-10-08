#include "Telemetry.hpp"
#include <cassert>
#include <string>
#include <cstdio>

static std::string frame(const std::string &fields)
{
	std::string text = "<" + fields + ",";
	unsigned sum = 0;
	for (unsigned char c : text) { sum += c; }
	return text + std::to_string(255 - (sum & 255)) + ">\r\n";
}

int main()
{
	fuelcell::SerialParser parser;
	fuel_cell_s sample{};
	const std::string valid = frame("50,0.91,46.2,15,3,,-15,0,36,5,");
	unsigned count = 0;
	for (char c : valid) { count += parser.feed(c, sample, 1000000); }
	assert(count == 1 && sample.connected && sample.source == 1);
	assert(std::fabs(sample.voltage - 46.2f) < 0.001f);
	assert(sample.battpower == -15 && sample.mainerror == 36 && sample.suberror == 5);
	assert(sample.tankpressure == 50 && std::isnan(sample.tank_pressure_bar));
	assert(sample.raw_can_error == -1);

	// Checksum corruption, empty numeric fields and malformed numbers must
	// never become a plausible zero or move following fields into place.
	std::string corrupt = valid; corrupt[1] = '6';
	for (const auto &bad : {corrupt, frame("50,,46.2,15,3,,-15,0,36,5,"),
		frame("50,0.91,nan,15,3,,-15,0,36,5,"), frame("50,0.91,46.2x,15,3,,-15,0,36,5,"),
		std::string("[metadata]\r\n"), std::string(300, 'x') + "\r\n"}) {
		for (char c : bad) { assert(!parser.feed(c, sample, 2000000)); }
	}
	assert(sample.timestamp_sample == 1000000);
	for (char c : frame("50,0.91,46.2,15,3,,-15,0,36,5,info,20,30,40,50,0")) {
		count += parser.feed(c, sample, 2100000);
	}
	assert(count == 2); // extended formats share the original fields
	for (char c : std::string("<truncated") + valid) { count += parser.feed(c, sample, 2200000); }
	assert(count == 3);

	fuel_cell_can_s can{};
	can.connected = true; can.interface = 1; can.timestamp_sample = 3000000;
	can.battery_voltage_v = 46.2f; can.tank_pressure_bar = 100;
	can.state = 0; can.error_code = 32; can.battery_power_w = -2510;
	auto mapped = fuelcell::from_can(can, 2, 3100000, 2000000);
	assert(mapped.connected && mapped.source == 2 && mapped.timestamp_sample == 3000000);
	assert(mapped.tank_pressure_bar == 100 && std::isnan(mapped.tankpressure));
	assert(std::isnan(mapped.regpressure) && std::isnan(mapped.battpower));
	assert(std::isnan(mapped.outputpower) && std::isnan(mapped.spmpower));
	assert(mapped.raw_can_error == 32 && mapped.mainerror == -1 && mapped.suberror == -1);
	assert(!fuelcell::from_can(can, 3, 3100000, 2000000).connected); // wrong port
	assert(!fuelcell::from_can(can, 1, 3100000, 2000000).connected); // UART selected
	assert(!fuelcell::from_can(can, 2, 5000000, 2000000).connected); // exact timeout
	assert(!fuelcell::from_can(can, 2, 2900000, 2000000).connected); // future stamp
	can.connected = false;
	assert(!fuelcell::from_can(can, 2, 3100000, 2000000).connected);
	auto stale = fuelcell::unavailable(1, 5000000, sample.timestamp_sample);
	assert(!stale.connected && std::isnan(stale.voltage) && stale.psustate == -1);
	assert(stale.timestamp_sample == sample.timestamp_sample);
	puts("PASS: UART framing/checksum, empty fields, recovery, CAN mapping, unavailable values and freshness");
}
