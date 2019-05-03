#ifndef HW_INTERFACE_PLUGIN_SMART_BOARD_HPP__
#define HW_INTERFACE_PLUGIN_SMART_BOARD_HPP__

//always inlclude these
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <hw_interface/base_interface.hpp>

//include the header of the base type you want, Serial or UDP
#include <hw_interface/base_serial_interface.hpp>

//include ros message types
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h> 
#include <sensor_msgs/Temperature.h> 

#define CRC32_POLYNOMIAL 0xEDB88320L
#define PI 3.14159265358979
#define RAD2DEG 180.0/PI
#define DEG2RAD PI/180.0
#define GRAVITY 9.81 

namespace hw_interface_plugin_SMART_Board {

    class SMART_Board_serial : public base_classes::base_serial_interface
    {
    public:
        SMART_Board_serial();
        ~SMART_Board_serial() {}

    protected:

        //these methods are abstract as defined by the base_serial_interface
            //they must be defined
        bool subPluginInit(ros::NodeHandlePtr nhPtr);
        void setInterfaceOptions();
        bool interfaceReadHandler(const size_t &length, int arrayStartPos, const boost::system::error_code &ec);
        bool verifyChecksum();

        std::size_t SMART_BoardStreamMatcher(const boost::system::error_code &error, long totalBytesInBuffer);

        long headerLen;
        long fullPacketLen;
        sensor_msgs::Imu imuMessage;
        sensor_msgs::MagneticField magMessage; 
        sensor_msgs::Temperature tempMessage; 
        ros::Publisher imuPublisher;
        ros::Publisher magPublisher; 
        ros::Publisher tempPublisher; 
        double gyroScaleFactor;
        double accelScaleFactor;
        double magScaleFactor;
        double tempScaleFactor;
        
    private:
        unsigned long CRC32Value_(int i);
        unsigned long CalculateBlockCRC32_(unsigned long ulCount, unsigned char *ucBuffer ); // Number of bytes in the data block, Data block
    };

}

PLUGINLIB_EXPORT_CLASS(hw_interface_plugin_SMART_Board::SMART_Board_serial, base_classes::base_interface)



#endif //HW_INTERFACE_PLUGIN_SMART_BOARD_HPP__
