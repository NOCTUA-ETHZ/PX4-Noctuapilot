#ifndef NOCTUA_SENSORS_HPP
#define NOCTUA_SENSORS_HPP

#include <uORB/topics/noctua_sensors.h>

class MavlinkStreamNoctuaSensors : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamNoctuaSensors(mavlink);
    }
    const char *get_name() const
    {
        return MavlinkStreamNoctuaSensors::get_name_static();
    }
    static const char *get_name_static()
    {
        return "NOCTUA_SENSORS";
    }
    static uint16_t get_id_static()
    {
        return MAVLINK_MSG_ID_NOCTUA_SENSORS;
    }
    uint16_t get_id()
    {
        return get_id_static();
    }
    unsigned get_size()
    {
        return MAVLINK_MSG_ID_NOCTUA_SENSORS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    // uORB::Subscription is used to subscribe to a single-instance topic
    uORB::Subscription _noctua_sensors_sub{ORB_ID(noctua_sensors)};

    /* do not allow top copying this class */
    MavlinkStreamNoctuaSensors(MavlinkStreamNoctuaSensors &);
    MavlinkStreamNoctuaSensors& operator = (const MavlinkStreamNoctuaSensors &);

protected:
    explicit MavlinkStreamNoctuaSensors(Mavlink *mavlink) : MavlinkStream(mavlink)
    {}
	bool send() override
	{
		noctua_sensors_s data;

		if (_noctua_sensors_sub.update(&data))
		{
			mavlink_noctua_sensors_t msg{};
			msg.timestamp = data.timestamp;
			msg.temperature = data.temperature;
			msg.humidity = data.humidity;
			msg.hydrogen = data.hydrogen;

			mavlink_msg_noctua_sensors_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // NOCTUA_SENSORS_HPP
