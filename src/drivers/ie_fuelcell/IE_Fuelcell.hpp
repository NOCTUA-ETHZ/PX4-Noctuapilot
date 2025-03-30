
/**
 * @file IE_Fuelcell.hpp
 */

 #pragma once

 #include <lib/mixer_module/mixer_module.hpp>
 #include <px4_platform_common/module.h>
 #include <px4_platform_common/module_params.h>
 #include <sys/select.h>

 #include <uORB/Subscription.hpp>
 #include <uORB/topics/parameter_update.h>

 #include <uORB/Publication.hpp>
 #include <uORB/topics/fuel_cell.h>


 #include <px4_platform_common/log.h>
 #include <uORB/topics/vehicle_command.h>
 #include <systemlib/mavlink_log.h>
 #include <uORB/topics/mavlink_log.h>



 class IE_Fuelcell : public ModuleBase<IE_Fuelcell>, public OutputModuleInterface //public px4::ScheduledWorkItem, public ModuleParams
 {
 public:
	 /**
	  * @param device_name Name of the serial port e.g. "/dev/ttyS3"
	  * @param bad_rate_parameter Name of the parameter that holds the baud rate of this serial port
	  */
	 IE_Fuelcell(const char *device_name, const char *bad_rate_parameter);
	 virtual ~IE_Fuelcell();


	 static int task_spawn(int argc, char *argv[]); ///< @see ModuleBase
	 static int custom_command(int argc, char *argv[]); ///< @see ModuleBase
	 static int print_usage(const char *reason = nullptr); ///< @see ModuleBase
	 int print_status() override; ///< @see ModuleBase

	 void Run() override;
	 int readData(fuel_cell_s &data);
	 int publishData(const fuel_cell_s &data);


	// Implement pure virtual methods from OutputModuleInterface
	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
		unsigned num_outputs, unsigned num_control_groups_updated) override;
	void updateParams() override;

 private:

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Publication<fuel_cell_s> _fuel_cell_pub{ORB_ID(fuel_cell)};
	uORB::Publication<mavlink_log_s> _mavlink_log_pub{ORB_ID(mavlink_log)};

	char _stored_device_name[256]; // Adjust size as necessary
	char _stored_baud_rate_parameter[256]; // Adjust size as necessary


	// UART handling
	int initializeUART();
	bool _uart_initialized{false};
	int _uart_fd{0};
	fd_set _uart_fd_set;

	// Partial line accumulation:
	char _line_accum[256] {};
	int  _line_pos {0};

	int parseLine(const char *line_buf, fuel_cell_s &data);



 };
