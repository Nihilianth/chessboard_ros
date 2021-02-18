#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include "marlin_serial/GcodeAction.h"



/*
params:
topic_in
topic_out
ser_port
ser_baud
*/

serial::Serial ser_port;
uint32_t line_num = 0;


void write_gcode(const std::string msg)
{
    ROS_INFO("writing to marlin");

    std::stringstream ss;
    ss << "N" << ++line_num << " " << msg.c_str();
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

class GcodeAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<marlin_serial::GcodeAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  marlin_serial::GcodeFeedback feedback_;
  marlin_serial::GcodeResult result_;

public:

  GcodeAction(std::string name) :
    as_(nh_, name, boost::bind(&GcodeAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~GcodeAction(void)
  {
  }

  void executeCB(const marlin_serial::GcodeGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // TODO: read 'ok' messages
    for (auto itr : goal->commands)
        write_gcode(itr);
    
    if(success)
    {
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }

};

void write_cb(const std_msgs::String::ConstPtr &message)
{
    write_gcode(message->data);
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
       //ser_port.setPort("/dev/ttyACM0");
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
    
    GcodeAction gcode("gcode");
    
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        if (ser_port.available())
        {
            ROS_INFO("Reading Data from serial");
            std_msgs::String recv;
            recv.data = ser_port.read(ser_port.available());
            ROS_INFO_STREAM(recv.data);
            ser_pub.publish(recv);
        }
        loop_rate.sleep();
    }


    ros::spin();
}