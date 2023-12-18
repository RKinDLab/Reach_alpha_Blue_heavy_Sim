#include "ros2_control_demo_example_3/bplprotocol.h"
#include "ros2_control_demo_example_3/reach_comms.hpp"
#include <sstream>
#include <cstdlib>

ReachComms::ReachComms() : serial_conn_(io) {}

void ReachComms::connect(const std::string &serial_device, const int &baud_rate)
{
    try
    {
        serial_conn_.open(serial_device);
        serial_conn_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        serial_conn_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_conn_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_conn_.set_option(boost::asio::serial_port_base::character_size(8));

        printf("serial connection setup successfully\n");
    }
    catch (const boost::system::system_error &e)
    {
        std::cerr << "Error connecting to serial device: " << e.what() << '\n';
    }
}

void ReachComms::setMode(const std::uint8_t &mode_id)
{
    try
    {
        /* **************************** encodePacket() Example *************************************** */
        // create a buffer of bytes for your packet to be filled in with.
        uint8_t encodedPacket[MAX_PACKET_LENGTH];
        // Set data to zeros
        memset(encodedPacket, 0, MAX_PACKET_LENGTH);

        // Encoding data with the following information
        uint8_t deviceID = 0xFF;
        uint8_t packetID = PacketID_MODE;

        uint8_t data[1] = {mode_id}; // Int packet

        struct Packet packet;

        packet.deviceID = deviceID;
        packet.packetID = packetID;
        memcpy(packet.data, data, 1);
        packet.dataLength = 1;

        // encode the packet.
        encodePacket(encodedPacket, &packet);

        // Define a char type array
        unsigned char serialOutBuffer[MAX_PACKET_LENGTH];

        // Copy data from encodedPacket to serialOutBuffer
        std::copy(encodedPacket, encodedPacket + MAX_PACKET_LENGTH, serialOutBuffer);

        // Send data
        boost::asio::write(serial_conn_, boost::asio::buffer(serialOutBuffer, sizeof(serialOutBuffer)));
    }
    catch (const boost::system::system_error &e)
    {
        std::cerr << "Error in setMode: " << e.what() << '\n';
    }
}

void ReachComms::sendMsg(const std::uint8_t &device_id, const std::uint8_t &packet_id, const double &floatdata)
{
    try
    {
        // create a buffer of bytes for your packet to be filled in with.
        uint8_t encodedPacket[MAX_PACKET_LENGTH];
        // Set data to zeros
        memset(encodedPacket, 0, MAX_PACKET_LENGTH);

        // Encoding data with the following information
        uint8_t deviceID = device_id;
        uint8_t packetID = packet_id;
        float f = floatdata;

        uint8_t encodedFloatData[4];
        size_t dataLength = encodeFloat(encodedFloatData, f);

        struct Packet packet;

        packet.deviceID = deviceID;
        packet.packetID = packetID;
        memcpy(packet.data, encodedFloatData, dataLength);
        packet.dataLength = dataLength;

        // encode the packet.
        encodePacket(encodedPacket, &packet);

        // Define a char type array
        unsigned char serialOutBuffer[MAX_PACKET_LENGTH];

        // Copy data from encodedPacket to serialOutBuffer
        std::copy(encodedPacket, encodedPacket + MAX_PACKET_LENGTH, serialOutBuffer);

        // Send data
        boost::asio::write(serial_conn_, boost::asio::buffer(serialOutBuffer, sizeof(serialOutBuffer)));
    }
    catch (const boost::system::system_error &e)
    {
        std::cerr << "Error in sendMsg: " << e.what() << '\n';
    }
}


// add timeout -- priority
void ReachComms::readEncoderValues(const std::uint8_t &device_id, const std::uint8_t &packet_id, double &val)
{
    try
    {
        // create a buffer of bytes for your packet to be filled in with.
        uint8_t encodedPacket[MAX_PACKET_LENGTH];
        // Set data to zeros
        memset(encodedPacket, 0, MAX_PACKET_LENGTH);

        // Encoding data with the following information
        uint8_t deviceID = device_id;
        uint8_t packetID = PacketID_REQUEST_PACKET;

        uint8_t data[1] = {packet_id}; // Int packet

        struct Packet packet;

        packet.deviceID = deviceID;
        packet.packetID = packetID;
        memcpy(packet.data, data, 1);
        packet.dataLength = 1;

        // encode the packet.
        encodePacket(encodedPacket, &packet);

        // Define a char type array
        unsigned char serialInBuffer[MAX_PACKET_LENGTH];

        // Copy data from encodedPacket to serialOutBuffer
        std::copy(encodedPacket, encodedPacket + MAX_PACKET_LENGTH, serialInBuffer);

        // Send data
        boost::asio::write(serial_conn_, boost::asio::buffer(serialInBuffer, sizeof(serialInBuffer)));

        // Buffer for reading data
        char serialOutBuffer[MAX_PACKET_LENGTH];

        // Read data from the serial port
        size_t bytesRead = serial_conn_.read_some(boost::asio::buffer(serialOutBuffer, MAX_PACKET_LENGTH));

        // Encoding data with the following information
        struct Packet read_packet;

        // object to hold buffer to uint8_t
        uint8_t serialOutBufferInt8[MAX_PACKET_LENGTH];

        // Copy data from encodedPacket to serialOutBuffer
        std::copy(serialOutBuffer, serialOutBuffer + MAX_PACKET_LENGTH, serialOutBufferInt8);

        // decode the packet.
        int result = decodePacket(&read_packet, serialOutBufferInt8, static_cast<int>(bytesRead));
        if (result == 1)
        {
            // printf("Decoded packet: \n");
            // printf("Device ID: %d \n", read_packet.deviceID);
            // printf("Packet ID: %d \n", read_packet.packetID);

            float floatList[MAX_PACKET_LENGTH / 4];

            decodeFloats(floatList, read_packet.data, read_packet.dataLength);

            // printf("Decoded Floats: ");
            // printf(" %.9f", floatList[0]);
            val = floatList[0];
            // printf("\n");
        }
        else
        {
            printf("decodePackets ERROR: An error occurred during decoding.\n");
        }
    }
    catch (const boost::system::system_error &e)
    {
        std::cerr << "Error in readEncoderValues: " << e.what() << '\n';
    }
}