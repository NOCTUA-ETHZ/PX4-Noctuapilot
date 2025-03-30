#include "IE_Fuelcell.hpp"
#include <termios.h>
#include <cstring>    // for memset, memcpy, etc.
#include <cstdio>     // for printf/fprintf
#include <cstdlib>    // for atof, atoi
#include <sys/select.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>

// ... other includes as needed ...

IE_Fuelcell::IE_Fuelcell(const char *device_name, const char *baud_rate_parameter) :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{
	strncpy(_stored_device_name, device_name, sizeof(_stored_device_name) - 1);
	_stored_device_name[sizeof(_stored_device_name) - 1] = '\0';
	PX4_INFO("IE_Fuelcell created");

	strncpy(_stored_baud_rate_parameter, baud_rate_parameter, sizeof(_stored_baud_rate_parameter) - 1);
	_stored_baud_rate_parameter[sizeof(_stored_baud_rate_parameter) - 1] = '\0';
}

IE_Fuelcell::~IE_Fuelcell()
{
	PX4_WARN("Shutting down driver");
	close(_uart_fd);
}

int IE_Fuelcell::initializeUART()
{
	// Optional: remove these prints for silence
	//PX4_INFO("Initializing UART");

	// Hard-coded for now
	const char *UART_DEVICE = "/dev/ttyS1";
	int32_t BAUDRATE = B9600;

	int fd = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);

	if (fd < 0) {
		PX4_ERR("Error opening %s: %s", UART_DEVICE, strerror(errno));
		return PX4_ERROR;
	}

	// Clear O_NDELAY => blocking reads (though we still use select)
	fcntl(fd, F_SETFL, 0);

	struct termios tty;
	memset(&tty, 0, sizeof(tty));

	if (tcgetattr(fd, &tty) != 0) {
		PX4_ERR("tcgetattr error: %s", strerror(errno));
		close(fd);
		return PX4_ERROR;
	}

	cfsetispeed(&tty, BAUDRATE);
	cfsetospeed(&tty, BAUDRATE);

	// 8N1, no flow control, local, enable receiver
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	// Raw input
	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	// No software flow control
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_iflag &= ~(ICRNL | BRKINT | INLCR | IGNCR | INPCK | ISTRIP);

	// Raw output
	tty.c_oflag &= ~OPOST;

	// VMIN=0 => return as soon as data is available
	// VTIME=15 => up to 1.5 second read() block
	tty.c_cc[VMIN]  = 0;
	tty.c_cc[VTIME] = 15;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		PX4_ERR("tcsetattr error: %s", strerror(errno));
		close(fd);
		return PX4_ERROR;
	}

	tcflush(fd, TCIOFLUSH);

	_uart_fd = fd;
	//PX4_INFO("UART successfully initialized on %s", UART_DEVICE);
	return PX4_OK;
}

void IE_Fuelcell::Run()
{
	int iteration = 0;
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (!_uart_initialized) {
		if (initializeUART() == PX4_OK) {
			_uart_initialized = true;
		} else {
			PX4_WARN("UART initialization failed, will retry...");
			ScheduleDelayed(200_ms);
			return;
		}
	}

	if (_uart_initialized) {
		fuel_cell_s data;
		int ret = readData(data);

		if (ret < 0 && iteration > 20) {
			iteration = 0;
			PX4_WARN("Read error => re-init port");
			_uart_initialized = false;
			if (_uart_fd >= 0) {
				close(_uart_fd);
				_uart_fd = -1;
			}
		}
		// If ret == 0 => either no data or successfully parsed => do nothing special
	}

	// Run at 5 Hz
	ScheduleDelayed(200_ms);
	iteration++;
}

int IE_Fuelcell::readData(fuel_cell_s &data)
{
	// Single pass of select => up to 100ms block
	PX4_WARN("Reading data...");
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(_uart_fd, &readfds);

	struct timeval tv;
	tv.tv_sec  = 0;
	tv.tv_usec = 100000; // 100ms

	int ret_select = select(_uart_fd + 1, &readfds, nullptr, nullptr, &tv);
	if (ret_select < 0) {
		PX4_ERR("select() error: %s", strerror(errno));
		return -1; // real error
	}

	if (ret_select == 0) {
		// No data in 100ms => not an error
		return 0;
	}

	// If here, data is available
	char tmp_buf[64];
	ssize_t n = read(_uart_fd, tmp_buf, sizeof(tmp_buf));

	if (n < 0) {
		PX4_ERR("read() error: %s", strerror(errno));
		return -1; // real error
	}

	if (n == 0) {
		// No data read => not an error
		return 0;
	}

	// Accumulate into _line_accum
	for (ssize_t i = 0; i < n; i++) {
		char c = tmp_buf[i];

		if (c == '\n' || c == '\r') {
			// We have a full line
			_line_accum[_line_pos] = '\0';
			PX4_INFO("Received line: %s", _line_accum);

			// parseLine returns PX4_OK or PX4_ERROR
			int parse_status = parseLine(_line_accum, data);
			// Reset for next line
			_line_pos = 0;

			if (parse_status == PX4_ERROR) {
				// Return negative => triggers re-init in this example
				return -1;
			}

			// Successfully parsed or ignored `[ ... ]` => do not re-init
		} else {
			// Accumulate if there's space
			if (_line_pos < (int)sizeof(_line_accum) - 1) {
				_line_accum[_line_pos++] = c;
			} else {
				PX4_ERR("Line too long; discarding");
				_line_pos = 0;
				return -1;
			}
		}
	}

	// If we get here, we processed everything we read. It's possible we didn't see a newline,
	// in which case we just keep the partial line for next pass.
	return 0;
}

int IE_Fuelcell::parseLine(const char *line_buf, fuel_cell_s &data)
{

	// 1) If we see [ ... ], discard without error
	{
		const char *startSq = strchr(line_buf, '[');
		const char *endSq   = (startSq ? strchr(startSq, ']') : nullptr);

		if (startSq && endSq && (endSq > startSq)) {
			PX4_INFO("Discarding line in square brackets: '%s'", line_buf);
			return PX4_OK; // Not an error
		}
	}

	// 2) If we see <...>, parse it
	{
		const char *startAng = strchr(line_buf, '<');
		const char *endAng   = (startAng ? strchr(startAng, '>') : nullptr);

		if (startAng && endAng && (endAng > startAng)) {
			size_t data_len = endAng - (startAng + 1);

			if (data_len >= 256) {
				PX4_ERR("Data string too long");
				return PX4_ERROR;
			}

			char dataStr[256];
			memcpy(dataStr, startAng + 1, data_len);
			dataStr[data_len] = '\0';

			char *tokens[20];
			memset(tokens, 0, sizeof(tokens));
			int token_count = 0;

			char *saveptr = nullptr;
			char *token = strtok_r(dataStr, ",", &saveptr);

			while (token && token_count < 20) {
				tokens[token_count++] = token;
				token = strtok_r(NULL, ",", &saveptr);
			}

			if (token_count < 12) {
				PX4_ERR("Incomplete data received");
				return PX4_ERROR;
			}

			data.tankpressure = atof(tokens[0]);
			data.regpressure  = atof(tokens[1]);
			data.voltage      = atof(tokens[2]);
			data.outputpower  = atof(tokens[3]);
			data.spmpower     = atof(tokens[4]);
			// tokens[5] is not used
			data.battpower    = atof(tokens[6]);
			data.psustate     = atoi(tokens[7]);
			data.mainerror    = atoi(tokens[8]);
			data.suberror     = atoi(tokens[9]);

			PX4_INFO("Tank=%.2f, Reg=%.2f, V=%.2f, OutPow=%.2f, SPM=%.2f, Batt=%.2f, PSU=%ld, MainErr=%ld, SubErr=%ld",
			         (double)data.tankpressure,
			         (double)data.regpressure,
			         (double)data.voltage,
			         (double)data.outputpower,
			         (double)data.spmpower,
			         (double)data.battpower,
			         (long)data.psustate,
			         (long)data.mainerror,
			         (long)data.suberror);
			PX4_INFO("Parsed: '%s'", line_buf);
			publishData(data);
			PX4_INFO("Data published");
			return PX4_OK;
		}
	}

	// 3) If neither <...> nor [ ... ], treat as error
	PX4_ERR("No recognized data format in: '%s'", line_buf);
	return PX4_ERROR;
}

int IE_Fuelcell::publishData(const fuel_cell_s &data)
{
	fuel_cell_s fuel_cell_msg{};
	fuel_cell_msg.timestamp = hrt_absolute_time();
	fuel_cell_msg.tankpressure = data.tankpressure;
	fuel_cell_msg.regpressure  = data.regpressure;
	fuel_cell_msg.voltage      = data.voltage;
	fuel_cell_msg.outputpower  = data.outputpower;
	fuel_cell_msg.spmpower     = data.spmpower;
	fuel_cell_msg.battpower    = data.battpower;
	fuel_cell_msg.psustate     = data.psustate;
	fuel_cell_msg.mainerror    = data.mainerror;
	fuel_cell_msg.suberror     = data.suberror;

	_fuel_cell_pub.publish(fuel_cell_msg);
	PX4_WARN("Published fuel cell data: %f", static_cast<double>(fuel_cell_msg.tankpressure));
	return PX4_OK;
}

//------------------------------------------
// Boilerplate for PX4 module
//------------------------------------------

int IE_Fuelcell::task_spawn(int argc, char *argv[])
{
	if (argc < 3) {
		return print_usage("missing arguments");
	}

	if (argc > 3) {
		return print_usage("too many arguments");
	}

	PX4_INFO("Starting task_spawn");
	const char *device_name = argv[1];
	const char *baud_rate_parameter_value = argv[2];

	IE_Fuelcell *instance = new IE_Fuelcell(device_name, baud_rate_parameter_value);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		instance->ScheduleNow();
		PX4_INFO("Task spawned");
		return OK;
	}

	PX4_ERR("alloc failed");
	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return ERROR;
}

int IE_Fuelcell::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int IE_Fuelcell::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description

This driver communicates over UART with the Intelligent Energy FuelCell.

The driver needs to be enabled using param IEFC_SER_CFG and the baud rate set with IEFC_BAUD.

To start:
  $ ie_fuelcell start <UART device> <baud rate>
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ie_fuelcell", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

int IE_Fuelcell::print_status()
{
	if (_uart_initialized) {
		PX4_INFO("Driver is running");
		return PX4_OK;
	}
	PX4_INFO("Driver is not running");
	return PX4_ERROR;
}

bool IE_Fuelcell::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
                                unsigned num_outputs, unsigned num_control_groups_updated)
{
	return false;
}

void IE_Fuelcell::updateParams()
{
	PX4_INFO("Fuel cell parameters updated");
}

extern "C" __EXPORT int ie_fuelcell_main(int argc, char *argv[])
{
	return IE_Fuelcell::main(argc, argv);
}
