#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <iostream>
#include <queue>
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

std::string gcode_decorate(const std::string gcode_raw)
{
    std::stringstream ss;
    ss << "N" << ++line_num << " " << gcode_raw.c_str();
    //ss.seekg(0, ios::end);
    // add checksum
    // http://reprap.org/wiki/G-code#Special_fields
    int cs = 0;
    //for (int i = 0; ss.str().c_str)
    for (char i : ss.str())
        cs = cs^i;

    cs &= 0xFF;

    ss << "*" << cs << "\n";

    return ss.str();
}

void write_gcode(const std::string gcode)
{
    //ROS_INFO("writing to marlin");
    
    //size_t bytes_write = ser_port.write((message->data+"\n\r").c_str());
    size_t bytes_write = ser_port.write(gcode.c_str());
    ROS_INFO("wrote %zu bytes to marlin %s ", bytes_write, gcode.c_str());
}


class GcodeAction
{
protected:

  bool success_;
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<marlin_serial::GcodeAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  marlin_serial::GcodeFeedback feedback_;
  marlin_serial::GcodeResult result_;

  std::queue<std::string> command_queue_;

public:

  GcodeAction(std::string name) :
    as_(nh_, name, boost::bind(&GcodeAction::executeCB, this, _1), false),
    action_name_(name), success_(false)
  {
    as_.start();
  }

  ~GcodeAction(void)
  {
  }

  void ParseSerial(const std::string line)
  {
    /*
    *   N<int>  Line number of the command, if any
    *   P<int>  Planner space remaining
    *   B<int>  Block queue space remaining
  */
    // "ok N1 P15 B3\n"

    if (line.length() <= 3)
      return;

    //ROS_ERROR(line.substr(0,2).c_str());
    if (!line.substr(0,2).compare("ok"))
    {
      ROS_ERROR("got OK");

      if (command_queue_.empty())
      {
        success_ = true;
        //ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        //as_.setSucceeded(result_);
      }
      else
      {
        write_gcode(command_queue_.front());
        command_queue_.pop();
      }
    }

    
  }

  void executeCB(const marlin_serial::GcodeGoalConstPtr &goal)
  {
    success_ = false;
    // helper variables
    ros::Rate r(1);
    bool success = true;

    assert(command_queue_.empty());

    // TODO: read 'ok' messages
    for (auto itr : goal->commands)
      command_queue_.push(gcode_decorate(itr));
    
    const int BUFSIZE = 8;

    int sent = 0;
    for (;sent < BUFSIZE && !command_queue_.empty(); ++sent)
    {
      write_gcode(command_queue_.front());
      command_queue_.pop();
    }

    ROS_WARN_STREAM("GcodeAction::executeCB: sent " << sent << " with " << command_queue_.size() << " cmd remaining in queue");


    ros::Rate loop_rate(10);
    while (ros::ok())
    {
      ros::spinOnce();
      if (success_)
      {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
        return;
      }

      loop_rate.sleep();
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

    //ros::Subscriber ser_sub = nh.subscribe("serial_write", 1000, write_cb);
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
    write_gcode(gcode_decorate("M999"));
    
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        if (ser_port.available())
        {
            ROS_INFO("Reading Data from serial");
            std_msgs::String recv;
            std::vector<std::string> lines = ser_port.readlines(ser_port.available());

            for (auto itr : lines)
            {
              ROS_INFO(itr.substr(0, itr.length() - 2).c_str());
              gcode.ParseSerial(itr);
              recv.data = itr.c_str();
              ser_pub.publish(recv);
            }
            //recv.data = ser_port.read(ser_port.available());
        }
        loop_rate.sleep();
    }


    ros::spin();
}