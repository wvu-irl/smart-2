#include <hw_interface_plugin_SMART_Board/hw_interface_plugin_SMART_Board_serial.hpp>

//class constructor, do required instatiation only here
    //do not setup other things as they my not have been setup by the parent classes yet
hw_interface_plugin_SMART_Board::SMART_Board_serial::SMART_Board_serial()
{
    //A debug message
    ROS_INFO("A Wild SMART Board Plugin Appeared!");

    //force the ROS Console level to Debug Level
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
           ros::console::notifyLoggerLevelsChanged();
        }

    //enable automatic class metric collection.
    enableMetrics();

    // Compute IMU scale factors once for use later
    gyroScaleFactor = DEG2RAD*0.05; // rad/s/LSB
    accelScaleFactor = GRAVITY*.0033; // m/s^2/LSB
    magScaleFactor = .0005*pow(10,-4);
    tempScaleFactor = .14; 
    // Set first term in orientation covariance in imu message to -1 to signify no orientation data
    imuMessage.orientation_covariance[0] = -1;
}

//this is called each time the plugin is enabled, before anything else of the plugin is called
bool hw_interface_plugin_SMART_Board::SMART_Board_serial::subPluginInit(ros::NodeHandlePtr nhPtr)
{
    ROS_DEBUG_EXTRA("%s Plugin Init", pluginName.c_str());

    /*for Serial interfaces, 'deviceName' is an inherited member and must be defined.
        failing to define this variable will disable the plugin.
        Opening of the device port is handled automatically

        deviceName is the name and path of the port to be opened
            example: "/dev/ttyS0" */
    deviceName = "";
    ros::param::get(pluginName+"/deviceName", deviceName);

    // bind a functor to the streamCompletionChecker inherited member
    streamCompletionChecker = boost::bind(&hw_interface_plugin_SMART_Board::SMART_Board_serial::SMART_BoardStreamMatcher, this, _1, _2);

    // enable the completion functor if the bind succeeded
    enableCompletionFunctor = !streamCompletionChecker.empty();

    //retrieve string from the ROS Parameter Server
        //of the format '<plugin_name>/<parameter>
    std::string tempString = "";

    //place that wants to write data to the device
    if(ros::param::get(pluginName+"/subscribeToTopic", tempString))
    {
	//This will create a ros subscriber to the topic from the ROS parameter server.
	    //The rosMsgCallback method will be called whenever there is a message pending.
        //rosDataSub = nh->subscribe(tempString, 1, &roboteq_drive::rosMsgCallback, this);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic subscription name", pluginName.c_str());
    }

    //place to publish the data after reading from device
    if(ros::param::get(pluginName+"/imuTopic", tempString))
    {
        imuPublisher = nhPtr->advertise<sensor_msgs::Imu>(tempString, 1, false);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic advertisment name", pluginName.c_str());
    }
    if(ros::param::get(pluginName+"/magTopic", tempString))
    {
        magPublisher = nhPtr->advertise<sensor_msgs::MagneticField>(tempString, 1, false);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic advertisment name", pluginName.c_str());
    }
    if(ros::param::get(pluginName+"/tempTopic", tempString))
    {
        tempPublisher = nhPtr->advertise<sensor_msgs::Temperature>(tempString, 1, false);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic advertisment name", pluginName.c_str());
    }

    return true;
}

/*this function is called to setup the port
    typical serial port setup uses 115200 baud rate, 8 bit character size, no flow control,
      no parity, 1 stop bit. This is typical, but no all encompassing. Change this if
      the port requires different. */
void hw_interface_plugin_SMART_Board::SMART_Board_serial::setInterfaceOptions()
{
	int tempBaudRate = 0;
    ros::param::get(pluginName+"/baudrate", tempBaudRate);
    setOption<boost::asio::serial_port_base::baud_rate>(
                new boost::asio::serial_port_base::baud_rate(tempBaudRate));
    //8 bits per character
    setOption<boost::asio::serial_port_base::character_size>(
    			new boost::asio::serial_port_base::character_size( 8 ));

    //flow control
    setOption<boost::asio::serial_port_base::flow_control>(
    			new boost::asio::serial_port_base::flow_control(
    										boost::asio::serial_port_base::flow_control::type::none));

    //parity
    setOption<boost::asio::serial_port_base::parity>(
    			new boost::asio::serial_port_base::parity(
    										boost::asio::serial_port_base::parity::type::none));

    //stop-bits
    setOption<boost::asio::serial_port_base::stop_bits>(
    			new boost::asio::serial_port_base::stop_bits(
    										boost::asio::serial_port_base::stop_bits::type::one));

    ROS_INFO("%s :: Device: %s :: Baudrate %d", pluginName.c_str(), deviceName.c_str(), tempBaudRate);
}

//this is called automatically when data that passes the streamMatcher is okay
    //this function is called with a data length and a position in an inherited array member
        //named 'receivedData'
//data should be published to topics from here
bool hw_interface_plugin_SMART_Board::SMART_Board_serial::interfaceReadHandler(const size_t &length,
                                                                            int arrayStartPos, const boost::system::error_code &ec)
{
    //ROS_INFO_EXTRA_SINGLE("SMART Board Plugin Data Handler");

    long packetChecksum = ((long)receivedData[arrayStartPos+49]&0xFF);
    long computedChecksum = 0; 
    for(int i = 0; i < 49 ; i++)
    {
        computedChecksum += receivedData[arrayStartPos + i]&0xFF; 
    }
	computedChecksum = computedChecksum%256; 
    if(packetChecksum != computedChecksum)
    {      
        ROS_WARN("Checksum failed!... Bitch");
    }
    else
    {
        ROS_INFO("Array Start Pos: %i", arrayStartPos); 
        ROS_INFO("Accel x: %x", ((receivedData[arrayStartPos+4]&0xFF) +((receivedData[arrayStartPos+3]&0xFF)<<8))); 
        ROS_INFO("Accel x: %i", (int16_t)((receivedData[arrayStartPos+4]&0xFF) +((receivedData[arrayStartPos+3]&0xFF)<<8))); 
        ROS_INFO("Accel x: %f", (double)((int16_t)((receivedData[arrayStartPos+4]&0xFF) +((receivedData[arrayStartPos+3]&0xFF)<<8)))); 
        ROS_INFO("Accel x: %f", accelScaleFactor*(double)((int16_t)((receivedData[arrayStartPos+4]&0xFF) +((receivedData[arrayStartPos+3]&0xFF)<<8)))); 
	    // populate IMU message accelerations and angular velocities
	    imuMessage.linear_acceleration.x = accelScaleFactor*(double)((int16_t)((receivedData[arrayStartPos+4]&0xFF) +
			                            ((receivedData[arrayStartPos+3]&0xFF)<<8))); // m/s^2

	    imuMessage.linear_acceleration.y = accelScaleFactor*(double)((int16_t)((receivedData[arrayStartPos+6]&0xFF) +
			                            ((receivedData[arrayStartPos+5]&0xFF)<<8))); // m/s^2

	    imuMessage.linear_acceleration.z = accelScaleFactor*(double)((int16_t)((receivedData[arrayStartPos+8]&0xFF) +
			                            ((receivedData[arrayStartPos+7]&0xFF)<<8))); // m/s^2

	    imuMessage.angular_velocity.x = gyroScaleFactor*(double)((int16_t)((receivedData[arrayStartPos+10]&0xFF) +
			                            ((receivedData[arrayStartPos+9]&0xFF)<<8))); // rad/s

	    imuMessage.angular_velocity.y = gyroScaleFactor*(double)((int16_t)((receivedData[arrayStartPos+12]&0xFF) +
			                            ((receivedData[arrayStartPos+11]&0xFF)<<8))); // rad/s

	    imuMessage.angular_velocity.z = gyroScaleFactor*(double)((int16_t)((receivedData[arrayStartPos+14]&0xFF) +
			                            ((receivedData[arrayStartPos+13]&0xFF)<<8))); // rad/s
			             

	    // populate IMU message header
	    imuMessage.header.stamp = ros::Time::now();

	    // publish message
	    imuPublisher.publish(imuMessage);

	    // increment sequence number for next cycle
	    imuMessage.header.seq++;
	    
	    magMessage.magnetic_field.x = magScaleFactor*(double)((int16_t)((receivedData[arrayStartPos+16]&0xFF) +
			                            ((receivedData[arrayStartPos+15]&0xFF)<<8))); // Tesla
	    magMessage.magnetic_field.y = magScaleFactor*(double)((int16_t)((receivedData[arrayStartPos+18]&0xFF) +
			                            ((receivedData[arrayStartPos+17]&0xFF)<<8))); // Tesla 
	    magMessage.magnetic_field.z = magScaleFactor*(double)((int16_t)((receivedData[arrayStartPos+20]&0xFF) +
			                            ((receivedData[arrayStartPos+19]&0xFF)<<8))); // Tesla 			                            	    
        // populate Mag message header 
        magMessage.header.stamp = ros::Time::now();

	    // publish message
	    magPublisher.publish(magMessage);

	    // increment sequence number for next cycle
	    magMessage.header.seq++;    
	    
	    tempMessage.temperature = tempScaleFactor*(double)((int16_t)((receivedData[arrayStartPos+22]&0xFF) +
			                            ((receivedData[arrayStartPos+21]&0xFF)<<8))) + 25; // degrees Fahrenheit 
        // populate Temp message header 
        tempMessage.header.stamp = ros::Time::now();

	    // publish message
	    tempPublisher.publish(tempMessage);

	    // increment sequence number for next cycle
	    tempMessage.header.seq++; 
    }

    ROS_INFO("Buf Pointer: 0x%p\r\n", &receivedData[arrayStartPos]);
    ROS_INFO("length = %u, arrayStartPos = %i",length, arrayStartPos);
    std::printf("Contents: ");
    for(int i = 0; i < length; i++)
    {
        std::printf("%x | ", receivedData[arrayStartPos + i]);
    }
    std::printf("\r\n");

    return true;
}

//automatically called to check the checksum of the packet.
    //If un-wanted/un-needed, return true.
bool hw_interface_plugin_SMART_Board::SMART_Board_serial::verifyChecksum()
{
    return true;
}

std::size_t hw_interface_plugin_SMART_Board::SMART_Board_serial::SMART_BoardStreamMatcher(const boost::system::error_code &error, long totalBytesInBuffer)
{
    const long syncSeqLen = 2;
    headerLen = 0;
    fullPacketLen = 0;
    dataArrayStart = 0;
    ROS_INFO("Total Bytes in Buffer: %i", totalBytesInBuffer); 
    if(error != boost::asio::error::operation_aborted)
    {
        if(totalBytesInBuffer < syncSeqLen)
        {
            ROS_DEBUG_EXTRA_NAME("Returning Early, %ld", syncSeqLen-totalBytesInBuffer);
            return syncSeqLen-totalBytesInBuffer;
        }
        for(long i=0; i<totalBytesInBuffer; i++)
        {
            if((receivedData[i]&0xFF) == 0x41) // Find first sync char
            {
                if((totalBytesInBuffer-(i+1)) < (syncSeqLen-1)) // Not enough to find rest of rest of sync sequence, read remaining
                {
                    return syncSeqLen-1;
                }
                else
                {
                    if((receivedData[i+1]&0xFF) == 0x5A) // Find second sync char
                    {
			dataArrayStart = i; 
                        if((totalBytesInBuffer-dataArrayStart) < 50) // Not enough to find rest of rest of sync sequence, read remaining
                        {
                           return 50 - (totalBytesInBuffer-dataArrayStart);
                        }
                        else
                        {
                            return 0; 
                        }
                    }
                }
            }
        }
    }
}

