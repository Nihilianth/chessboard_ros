#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <iostream>


/*
params:
topic_in
topic_out
ser_port
ser_baud
*/

serial::Serial ser_port;
uint32_t line_num = 0;

void write_cb(const std_msgs::String::ConstPtr &message)
{
    ROS_INFO("writing to marlin");

    std::stringstream ss;
    ss << "N" << ++line_num << " " << message->data.c_str();
    //ss.seekg(0, ios::end);
    // add checksum
    // http://reprap.org/wiki/G-code#Special_fields
    int cs = 0;
    //for (int i = 0; ss.str().c_str)
    for (char i : ss.str())
        cs = cs^i;

    cs &= 0xFF;

    ss << "*" << cs << "\n";
    
    //size_t bytes_write = ser_port.write((message->data+"\n\r").c_str());
    size_t bytes_write = ser_port.write(ss.str().c_str());
    ROS_INFO("wrote %zu bytes to marlin %s ", bytes_write, ss.str().c_str());
    
}
int main (int argc, char * argv[])
{
    ros::init(argc, argv, "marlin_serial_node");
    ros::NodeHandle nh;

    ros::Subscriber ser_sub = nh.subscribe("serial_write", 1000, write_cb);
    ros::Publisher ser_pub = nh.advertise<std_msgs::String>("serial_read", 1000);

    ROS_INFO("Node started");

    try
    {
       serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
       ser_port.setBaudrate(115200);
       ser_port.setPort("/dev/ttyUSB0");
       ser_port.setTimeout(timeout); // 1s
       ser_port.open();

    }
    catch(serial::IOException &e)    {
        std::cerr << e.what() << '\n';
    }

    if (ser_port.isOpen())
        ROS_INFO("Serial port open.");
    else
    {
        ROS_ERROR("Could not open serial port.");
        return -1;
    }
    
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        if (ser_port.available())
        {
            //ROS_INFO("Reading Data from serial");
            std_msgs::String recv;
            recv.data = ser_port.read(ser_port.available());
            ROS_INFO_STREAM(recv.data);
            ser_pub.publish(recv);
        }
        loop_rate.sleep();
    }

    ros::spin();
}