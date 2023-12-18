#ifndef ROS2_CONTROL_DEMO_EXAMPLE_3__REACH_COMMS_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_3__REACH_COMMS_HPP_

#include <iostream>
#include <boost/asio.hpp>

class ReachComms
{

public:

    ReachComms ();

    void connect (const std::string& serial_device, const int& baud_rate);
    void readEncoderValues (const std::uint8_t& device_id, const std::uint8_t& packet_id, double& val);
    void sendMsg (const std::uint8_t& device_id, const std::uint8_t& packet_id, const double& floatdata);
    void setMode (const std::uint8_t& mode_id);
    bool connected () const { return serial_conn_.is_open(); }
    void disconnect () { return serial_conn_.close(); }

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial_conn_; ///< Underlying serial connection
};

#endif // ROS2_CONTROL_DEMO_EXAMPLE_3__REACH_COMMS_HPP_