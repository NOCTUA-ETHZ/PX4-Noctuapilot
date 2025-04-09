#ifndef FUEL_CELL_HPP
#define FUEL_CELL_HPP

#include <uORB/topics/fuel_cell.h>

class MavlinkStreamFuelCell: public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamFuelCell(mavlink);
    }
    const char *get_name() const
    {
        return MavlinkStreamFuelCell::get_name_static();
    }
    static const char *get_name_static()
    {
        return "FUEL_CELL";
    }
    static uint16_t get_id_static()
    {
        return MAVLINK_MSG_ID_FUEL_CELL;
    }
    uint16_t get_id()
    {
        return get_id_static();
    }
    unsigned get_size()
    {
        return MAVLINK_MSG_ID_FUEL_CELL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    // uORB::Subscription is used to subscribe to a single-instance topic
    uORB::Subscription _fuel_cell_sub{ORB_ID(fuel_cell)};

    /* do not allow top copying this class */
    MavlinkStreamFuelCell(MavlinkStreamFuelCell &);
    MavlinkStreamFuelCell& operator = (const MavlinkStreamFuelCell &);

protected:
    explicit MavlinkStreamFuelCell(Mavlink *mavlink) : MavlinkStream(mavlink)
    {}

	bool send() override
	{
		struct fuel_cell_s data;

		if (_fuel_cell_sub.update(&data))
		{
		    mavlink_fuel_cell_t msg;

		    msg.timestamp    = data.timestamp;
		    msg.tankpressure = data.tankpressure;
		    msg.regpressure  = data.regpressure;
		    msg.voltage      = data.voltage;
		    msg.outputpower  = data.outputpower;
		    msg.spmpower     = data.spmpower;
		    msg.battpower    = data.battpower;
		    msg.psustate     = data.psustate;
		    memcpy(msg.info, data.info, sizeof(msg.info)); // handle uint8[2]

		    mavlink_msg_fuel_cell_send_struct(_mavlink->get_channel(), &msg);
		    return true;
		}

		return false;
	}

};

#endif // FUEL_CELL_HPP
