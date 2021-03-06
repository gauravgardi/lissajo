#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>

mavros_msgs::State current_state;
sensor_msgs::Imu imuvar;
mavros_msgs::AttitudeTarget att_tgt;

float q[4], q_tgt[4];
float yaw_ang, yaw_tgt, pitch_ang, roll_ang;
float dt;
float rad = 3.0;            // radius of circle
float cent_x = 0.0;         // x coordinate of centre of circle
float cent_y = 0.0;         // y coordinate of entre of circle
float ang =0.0;             // angular position
float step = 0.01;          // angular rate
float x_set = 0.0;          // x coordinate set point
float y_set = 0.0;          // y coordinate set point
float z_set = 3.0;          // z coordinate set point
float eps = 0.2;            // allowed error in x and y coordinate
float eps_z = z_set*0.1;    //allowed error in z coordinate
float ErrorLin = 0.0;

nav_msgs::Odometry pos_feed;

/////////////////////////////////////////////////
//////////////////POSE SUBSCRIBER////////////////
/////////////////////////////////////////////////

void feedbackfn(const nav_msgs::Odometry::ConstPtr& odom_data)
{
    pos_feed= *odom_data;
    q[0] = pos_feed.pose.pose.orientation.w;
    q[1] = pos_feed.pose.pose.orientation.x;
    q[2] = pos_feed.pose.pose.orientation.y;
    q[3] = pos_feed.pose.pose.orientation.z;
    yaw_ang=atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
    pitch_ang=asin(2*(q[0]*q[2]-q[3]*q[1]));
    roll_ang=atan2(2*(q[0]*q[1]+q[3]*q[2]),1-2*(q[2]*q[2]+q[1]*q[1]) );
}

//////////////////////////////////
///// STATE SUBSCRIBER ////////////////
///////////////////////////////////

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
	float v_xi, v_yi, omg;

    //int rate = 20;

    ros::init(argc, argv, "testerror");
    ros::NodeHandle nh;

    //ros::Rate r(rate);

    //dt=1/rate;
    ros::Subscriber imu_yaw = nh.subscribe("mavros/local_position/odom", 10, feedbackfn);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/attitude", 10);

    geometry_msgs::PoseStamped pos_cmd;
    geometry_msgs::TwistStamped vel;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    
    pos_cmd.pose.position.x = 0;
    pos_cmd.pose.position.y = 0;
    pos_cmd.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pos_cmd);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
    yaw_tgt=yaw_ang;			}

////////////////////////////////////////////
    ///////////////////AUTO FLIGHT/////////////////////
    ////////////////////////////////////////////    
        ROS_INFO("auto mode");
        //x_set = cent_x + rad*cos(ang); // CIRCLE
        //y_set = cent_y + rad*sin(ang);
        x_set = 1.0;
        y_set = 2.0;
        ROS_INFO("Going towards %f %f %f", x_set, y_set, z_set);
	ErrorLin = sqrt((pos_feed.pose.pose.position.x - 1.0)*(pos_feed.pose.pose.position.x - 1.0) + (pos_feed.pose.pose.position.y - 2.0)*(pos_feed.pose.pose.position.y - 2.0));
        ROS_INFO("current position %f   %f  %f angle %f error %f",pos_feed.pose.pose.position.x, pos_feed.pose.pose.position.y, pos_feed.pose.pose.position.z, ang, ErrorLin);
        if(fabs(pos_feed.pose.pose.position.z - z_set)<=eps_z)
        {
            ROS_INFO("ALTD REACHED");
            //if(fabs(pos_feed.pose.pose.position.x - x_set)<=eps && fabs(pos_feed.pose.pose.position.y - y_set)<=eps)
            ang = ang + step;
            pos_cmd.pose.position.x = x_set;
            pos_cmd.pose.position.y = y_set;
            pos_cmd.pose.position.z = z_set;
            pos_cmd.pose.orientation.x = 0.0;
            pos_cmd.pose.orientation.y = 0.0;
            pos_cmd.pose.orientation.z = 0.0;
            pos_cmd.pose.orientation.w = 0.0;
            /*vel.twist.linear.x = 0.3;
            vel.twist.linear.y = 0.0;
            vel.twist.linear.z = 0.0;
            vel.twist.angular.z = 0.1;*/
        }
        else
        {
            ROS_INFO("REACHING ALTD");
            pos_cmd.pose.position.x = pos_feed.pose.pose.position.x;
            pos_cmd.pose.position.y = pos_feed.pose.pose.position.y;
            pos_cmd.pose.position.z = z_set;
            pos_cmd.pose.orientation.x = 0.0;
            pos_cmd.pose.orientation.y = 0.0;
            pos_cmd.pose.orientation.z = 0.0;
            pos_cmd.pose.orientation.w = 0.0;
        }

        local_pos_pub.publish(pos_cmd);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
