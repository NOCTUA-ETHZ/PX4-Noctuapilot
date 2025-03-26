****************************************************************************/

/**
 * @file IE_Fuelcell.hpp
 *
 * Intelligent Energy Fuel Cell driver
 *
 * Product page: https://www.intelligent-energy.com/
 * Manual: https://www.intelligent-energy.com/product-support/ie-soar-support/
 */

#pragma once

#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <sys/select.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/wheel_encoders.h>

class IE_Fuelcell : public ModuleBase<IE_Fuelcell>, public OutputModuleInterface
{
public:
	/**
	 * @param device_name Name of the serial port e.g. "/dev/ttyS2"
	 * @param bad_rate_parameter Name of the parameter that holds the baud rate of this serial port
	 */
	IE_Fuelcell(const char *device_name, const char *bad_rate_parameter);
	virtual ~IE_Fuelcell();


	static int task_spawn(int argc, char *argv[]); ///< @see ModuleBase
	static int custom_command(int argc, char *argv[]); ///< @see ModuleBase
	static int print_usage(const char *reason = nullptr); ///< @see ModuleBase
	int print_status() override; ///< @see ModuleBase

	void Run() override;

	/** @see OutputModuleInterface */
	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	int readEncoder();
	void resetEncoders();

private:
	enum class Command : uint8_t {
		ReadStatus = 90,

		DriveForwardMotor1 = 0,
		DriveBackwardsMotor1 = 1,
		DriveForwardMotor2 = 4,
		DriveBackwardsMotor2 = 5,
		DutyCycleMotor1 = 32,
		DutyCycleMotor2 = 33,

		ReadSpeedMotor1 = 18,
		ReadSpeedMotor2 = 19,
		ResetEncoders = 20,
		ReadEncoderCounters = 78,
	};

	static constexpr int MAX_ACTUATORS = 2;

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Publication<wheel_encoders_s> _wheel_encoders_pub{ORB_ID(wheel_encoders)};
	uORB::Publication<fuel_cell_status_s> _fuel_cell_status_pub{ORB_ID(fuel_cell_status)};

	char _stored_device_name[256]; // Adjust size as necessary
	char _stored_baud_rate_parameter[256]; // Adjust size as necessary

	void sendUnsigned7Bit(Command command, float data);
	void sendSigned16Bit(Command command, float data);


	int receiveTransaction(Command cmd, uint8_t *read_buffer, size_t bytes_to_read);
	int writeCommand(Command cmd);
	int readResponse(Command command, uint8_t *read_buffer, size_t bytes_to_read);

	static uint16_t _calcCRC(const uint8_t *buf, size_t n, uint16_t init = 0);

	// UART handling
	int initializeUART();
	bool _uart_initialized{false};
	int _uart_fd{0};
	fd_set _uart_fd_set;
	struct timeval _uart_fd_timeout;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::IEFC_ADDRESS>) _param_iefc_address,
	)
};
