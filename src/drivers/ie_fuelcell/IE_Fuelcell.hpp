/**
 * @file IE_Fuelcell.hpp
 * @brief Driver for Intelligent Energy FuelCell over UART
 */

 #pragma once

 #include <px4_platform_common/module.h>
 #include <px4_platform_common/module_params.h>
 #include <lib/mixer_module/mixer_module.hpp>
 #include <sys/select.h>

 #include <uORB/Subscription.hpp>
 #include <uORB/topics/parameter_update.h>
 #include <uORB/Publication.hpp>
 #include <uORB/topics/fuel_cell.h>

 #include <cstring>
#include <cstdio>
#include <cstdlib>
#include <sys/select.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

 /**
  * @class IE_Fuelcell
  *
  * This class implements communication with the Intelligent Energy FuelCell
  * via a UART interface.
  */
 class IE_Fuelcell
	 : public ModuleBase<IE_Fuelcell>
	 , public OutputModuleInterface
 {
 public:
	 /**
	  * @param device_name        Name of the serial port, e.g. "/dev/ttyS3"
	  * @param bad_rate_parameter String representing the desired baud rate, e.g. "9600"
	  */
	 IE_Fuelcell();

	 virtual ~IE_Fuelcell();

	 static int task_spawn(int argc, char *argv[]);
	 static int custom_command(int argc, char *argv[]);
	 static int print_usage(const char *reason = nullptr);

	 int print_status() override;


	 bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			    unsigned num_outputs, unsigned num_control_groups_updated) override;
	 void updateParams() override;

	 void Run() override;
	 int  readData(fuel_cell_s &data);
	 int  publishData(const fuel_cell_s &data);

 private:


	 int  initializeUART();
	 int  parseLine(const char *line_buf, fuel_cell_s &data);


	 uORB::Subscription         _parameter_update_sub{ORB_ID(parameter_update)};
	 uORB::Publication<fuel_cell_s> _fuel_cell_pub{ORB_ID(fuel_cell)};


	 bool _uart_initialized{false};
	 int  _uart_fd{-1};

	 char _line_accum[256]{};
	 int  _line_pos{0};

	 fd_set _uart_fd_set;

 };
