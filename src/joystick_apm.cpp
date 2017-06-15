#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/AttitudeTarget.h>
sensor_msgs::Joy joyvar;
mavros_msgs::State current_state;
sensor_msgs::Imu imuvar;
mavros_msgs::AttitudeTarget att_tgt;
bool fl_takeoff=false;
bool fl_manual=false ;
bool fl_auto=false ;
bool fl_no_sig=true ;
bool fl_land=false ;
float q[4], q_tgt[4];
float yaw_ang, yaw_tgt, pitch_ang, roll_ang;
float dt;


void yawfn(const sensor_msgs::Imu::ConstPtr& imu_data)
{imuvar= *imu_data;
q[0] = imuvar.orientation.w;
    q[1] = imuvar.orientation.x;
    q[2] = imuvar.orientation.y;
    q[3] = imuvar.orientation.z;
yaw_ang=atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
pitch_ang=asin(2*(q[0]*q[2]-q[3]*q[1]));
roll_ang=atan2(2*(q[0]*q[1]+q[3]*q[2]),1-2*(q[2]*q[2]+q[1]*q[1]) );
}

void joyfn(const sensor_msgs::Joy::ConstPtr& joy){
    //ROS_INFO("1");
    joyvar = *joy;

if (joyvar.buttons[0] == 1 && fl_takeoff==false && fl_land==false)
	{fl_manual=false;
	fl_takeoff=true;
ROS_INFO("TAKE OFF");}

if (joyvar.buttons[2] == 1 && fl_land==false && fl_takeoff==false)
	{fl_land=true;
    fl_manual=false;	
	ROS_INFO("LAND");}

 if (joyvar.buttons[1] == 1 && fl_land==false && fl_takeoff==false)
    {fl_manual=true;  
    ROS_INFO("manual");}   
}
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
     float v_xi, v_yi, omg;

    int rate = 10;

    ros::init(argc, argv, "joystick_apm");
    ros::NodeHandle n;

    ros::Rate r(rate);

    dt=1/rate;

    ros::Subscriber imu_yaw = n.subscribe("/mavros/imu/data", 10, yawfn);
    ros::Subscriber joy_ip = n.subscribe("joy", 10, joyfn);
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = n.advertise<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher att_pub = n.advertise<mavros_msgs::AttitudeTarget>
            ("/mavros/setpoint_raw/attitude", 10);

    mavros_msgs::SetMode srv_setMode;
    mavros_msgs::CommandBool srv;
    mavros_msgs::CommandTOL srv_takeoff;



    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped vel;
    ////////////////////////////////////////////
    /////////////////GUIDED/////////////////////
    ////////////////////////////////////////////
      while (ros::ok() )
    {







    if (fl_takeoff==true)
    {fl_manual=false;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "GUIDED";
    if(cl.call(srv_setMode)){
        ROS_INFO("setmode send ok %d value:", srv_setMode.response.success);
    }else{
        ROS_ERROR("Failed SetMode");
        return -1;
    }
sleep(2);
    ////////////////////////////////////////////
    ///////////////////ARM//////////////////////
    ////////////////////////////////////////////
  
    srv.request.value = true;
    if(arming_cl.call(srv)){
        ROS_INFO("ARM send ok %d", srv.response.success);
    }else{
        ROS_ERROR("Failed arming or disarming");
    }
sleep(2);
    ////////////////////////////////////////////
    /////////////////TAKEOFF////////////////////
    ////////////////////////////////////////////
    srv_takeoff.request.altitude =1;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    if(takeoff_cl.call(srv_takeoff)){
        ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);
    }else{
        ROS_ERROR("Failed Takeoff");
    }
    sleep(3);
	
    fl_takeoff=false;
    yaw_tgt=yaw_ang;			}
    ////////////////////////////////////////////
    /////////////////DO STUFF///////////////////
    ////////////////////////////////////////////
    //sleep(10);


if (fl_manual==true)
{
/*q[0] = imu.orientation.w;
    q[1] = imu.orientation.x;
    q[2] = imu.orientation.y;
    q[3] = imu.orientation.z;
    ang = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
 */   
    //cur_yaw.data = cur_yaw.data-90;
   //ROS_INFO("%lf,%lf,%lf,%f",joyvar.axes[3],joyvar.axes[2],joyvar.axes[1],ang*180/3.14);
   
    v_xi= joyvar.axes[2]*1;
    v_yi= joyvar.axes[3]*1;

    vel.twist.linear.y = v_yi;//*cos(-ang) - v_yi*sin(-ang);
    vel.twist.linear.x = v_xi;//*cos(-ang) + v_xi*sin(-ang);
    vel.twist.linear.z = joyvar.axes[1]*1;
   if(joyvar.buttons[6] != 0 ||  joyvar.buttons[7] != 0)
    {   omg = joyvar.buttons[6]*0.3;
    //else
       omg = -joyvar.buttons[7]*0.3;
    //if(joyvar.buttons[6] != 0)
     yaw_tgt+=omg*dt;  

    double t0 = cos(yaw_tgt * 0.5);
    double t1 = sin(yaw_tgt * 0.5);
    double t2 = cos(roll_ang* 0.5);
    double t3 = sin(roll_ang * 0.5);
    double t4 = cos(pitch_ang * 0.5);
    double t5 = sin(pitch_ang * 0.5);

    q_tgt[0] = t0 * t2 * t4 + t1 * t3 * t5;
    q_tgt[1] = t0 * t3 * t4 - t1 * t2 * t5;
    q_tgt[2] = t0 * t2 * t5 + t1 * t3 * t4;
    q_tgt[3] = t1 * t2 * t4 - t0 * t3 * t5;



    
    att_tgt.orientation.w=q_tgt[0];
    att_tgt.orientation.x=q_tgt[1];
    att_tgt.orientation.y=q_tgt[2];
    att_tgt.orientation.z=q_tgt[3];
    att_tgt.body_rate.z=omg;

    att_pub.publish(att_tgt);
    }
    else
    local_vel_pub.publish(vel);
}













    ////////////////////////////////////////////
    ///////////////////LAND/////////////////////
    ////////////////////////////////////////////
    
 	if (fl_land==true)
 	   {fl_manual=false;
    
   


    ros::ServiceClient land_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = 0;
   // srv_land.request.latitude = 0;
    //srv_land.request.longitude = 0;
    srv_land.request.min_pitch = 0;
    //srv_land.request.yaw = 0;
    if(land_cl.call(srv_land)){
        ROS_INFO("srv_land send ok %d", srv_land.response.success);
    }else{
        ROS_ERROR("Failed Land");
    }
    sleep(5);

  srv.request.value = false;
    if(arming_cl.call(srv)){
        ROS_INFO("ARM send ok %d", srv.response.success);
    }else{
        ROS_ERROR("Failed arming or disarming");
    }
     srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "STABILIZE";
    if(cl.call(srv_setMode)){
        ROS_INFO("setmode send ok %d value:", srv_setMode.response.success);
    }else{
        ROS_ERROR("Failed SetMode");
        return -1;
    }
fl_land=false;
 		 }











      ros::spinOnce();
      r.sleep();
    }

    return 0;
}
