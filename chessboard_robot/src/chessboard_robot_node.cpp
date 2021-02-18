#include <string>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "chessboard_robot_node");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(60.0);

    const double degree = M_PI/180;

    // robot state
    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

    // message declarations
    //geometry_msgs::TransformStamped odom_trans;
    geometry_msgs::TransformStamped x_trans;
    geometry_msgs::TransformStamped y_trans;
    geometry_msgs::TransformStamped eef;
    //sensor_msgs::JointState joint_state;
    x_trans.header.frame_id = "home_link";
    x_trans.child_frame_id = "x_link_1";

    y_trans.header.frame_id = "home_link";
    y_trans.child_frame_id = "y_link_1";

    eef.header.frame_id = "home_link";
    eef.child_frame_id = "end_effector_1";

    while (ros::ok()) {
        //update joint_state
        /*
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(3);
        joint_state.position.resize(3);
        joint_state.name[0] ="swivel";
        joint_state.position[0] = swivel;
        joint_state.name[1] ="tilt";
        joint_state.position[1] = tilt;
        joint_state.name[2] ="periscope";
        joint_state.position[2] = height;
        */

    // todo: via tf
    // XZ plane
       // EEF position
        eef.header.stamp = ros::Time::now();
        eef.transform.translation.x = 0.0;//(0.12 + cos(angle)*0.09);
        eef.transform.translation.z = 0.0;//(0.12 + sin(angle)*0.09);
        eef.transform.translation.y = 0;
        eef.transform.rotation = tf::createQuaternionMsgFromYaw(0);
       // Calculate X
        x_trans.header.stamp = ros::Time::now();
        x_trans.transform.translation.x = eef.transform.translation.x;
        x_trans.transform.translation.z = eef.transform.translation.z;
        x_trans.transform.translation.y = 0;
        x_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
       // Calculate Y
        y_trans.header.stamp = ros::Time::now();
        y_trans.transform.translation.x = 0.301; // dont move in this direction
        y_trans.transform.translation.z = eef.transform.translation.z;
        y_trans.transform.translation.y = 0;
        y_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

        // update transform
        /*
        x_trans.header.stamp = ros::Time::now();
        x_trans.transform.translation.x = 0;
        x_trans.transform.translation.y = 0;
        x_trans.transform.translation.z = 0;
        x_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
        */
        // (moving in a circle with radius=2)
        /*
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = cos(angle)*2;
        odom_trans.transform.translation.y = sin(angle)*2;
        odom_trans.transform.translation.z = .7;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

        */
        //send the joint state and transform
        //joint_pub.publish(joint_state);
        broadcaster.sendTransform(x_trans);
        broadcaster.sendTransform(y_trans);
        //broadcaster.sendTransform(eef);

        // Create new robot state
        tilt += tinc;
        if (tilt<-.5 || tilt>0) tinc *= -1;
        height += hinc;
        if (height>.2 || height<0) hinc *= -1;
        swivel += degree;
        angle += degree/4;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}