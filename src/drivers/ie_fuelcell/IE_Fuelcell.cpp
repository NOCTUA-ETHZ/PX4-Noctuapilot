#include "IE_Fuelcell.hpp"
#include <termios.h>


IE_Fuelcell::IE_Fuelcell(const char *device_name, const char *baud_rate_parameter) :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{
	// Store device name
	if (device_name) {
		strncpy(_stored_device_name, device_name, sizeof(_stored_device_name) - 1);
		_stored_device_name[sizeof(_stored_device_name) - 1] = '\0';
	} else {
		// Default if none provided
		strncpy(_stored_device_name, "/dev/ttyS1", sizeof(_stored_device_name) - 1);
	}

	// Store baud rate string
	if (bad_rate_parameter) {
		strncpy(_stored_baud_rate_parameter, bad_rate_parameter, sizeof(_stored_baud_rate_parameter) - 1);
		_stored_baud_rate_parameter[sizeof(_stored_baud_rate_parameter) - 1] = '\0';
	} else {
		// Default if none provided
		strcpy(_stored_baud_rate_parameter, "9600");
	}
}

IE_Fuelcell::~IE_Fuelcell()
{
	PX4_WARN("Shutting down driver");
	close(_uart_fd);
}

int IE_Fuelcell::initializeUART()
{
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

	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_iflag &= ~(ICRNL | BRKINT | INLCR | IGNCR | INPCK | ISTRIP);

	tty.c_oflag &= ~OPOST;

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
	return PX4_OK;
}

void IE_Fuelcell::Run()
{
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
		if (readData(data) < 0) {
			PX4_WARN("Read error => re-init port");
			_uart_initialized = false;
			if (_uart_fd >= 0) {
				close(_uart_fd);
				_uart_fd = -1;
			}
		}
	}

	// Run at 5 Hz
	ScheduleDelayed(200_ms);
}

int IE_Fuelcell::readData(fuel_cell_s &data)
{
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(_uart_fd, &readfds);

	struct timeval tv;
	tv.tv_sec  = 0;
	tv.tv_usec = 200000; // 100ms

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
			_line_accum[_line_pos] = '\0';
			int parse_status = parseLine(_line_accum, data);
			_line_pos = 0;

			if (parse_status == PX4_ERROR) {
				return -1;
			}

		} else {
			if (_line_pos < (int)sizeof(_line_accum) - 1) {
				_line_accum[_line_pos++] = c;
			} else {
				PX4_ERR("Line too long; discarding");
				_line_pos = 0;
				return -1;
			}
		}
	}

	return 0;
}

int IE_Fuelcell::parseLine(const char *line_buf, fuel_cell_s &data)
{

	// 1) If we see [ ... ], discard without error
	{
		const char *startSq = strchr(line_buf, '[');
		const char *endSq   = (startSq ? strchr(startSq, ']') : nullptr);

		if (startSq || endSq) {
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
				PX4_ERR("Incomplete data received: %d tokens", token_count);
				return PX4_ERROR;
			}

			data.tankpressure = atof(tokens[0]);
			data.regpressure  = atof(tokens[1]);
			data.voltage      = atof(tokens[2]);
			data.outputpower  = atof(tokens[3]);
			data.spmpower     = atof(tokens[4]);
			data.battpower    = atof(tokens[6]);
			data.psustate     = atoi(tokens[7]);
			data.mainerror    = atoi(tokens[8]);
			data.suberror     = atoi(tokens[9]);
			publishData(data);
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
	return PX4_OK;
}


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
		Driver communicating over UART with the Intelligent Energy FuelCell.
		Configure which device to use and baud rate as follows:

		  $ ie_fuelcell start <device_path> <baud_string>
		  E.g.: ie_fuelcell start /dev/ttyS1 9600
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
