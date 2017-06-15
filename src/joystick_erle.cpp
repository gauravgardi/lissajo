#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/Joy.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
//#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <nav_msgs/Odometry.h>


/* redgearjoystick button map 
0--A
1--B
2--X
3--Y
l1--4
r1--5
back--6
start--7
home--8
ljoy bbutton--9
rjoy button--10

axes0--yaw_n
axes1-- throttle
axex2--ltrigger
axes3--roll
axes4--pitch
axes5--rtrigger
axes6--dpad x
axes7--dpad y


*/

sensor_msgs::Joy joyvar;
mavros_msgs::State current_state;
sensor_msgs::Imu imuvar;
//mavros_msgs::AttitudeTarget att_tgt;
mavros_msgs::OverrideRCIn msg_override;
nav_msgs::Odometry odomvar;
bool fl_takeoff=false;
bool fl_manual=false ;
bool fl_auto=false ;
bool fl_no_sig=true ;
bool fl_land=false ;
float q[4], q_tgt[4];
float yaw_ang, yaw_tgt, pitch_ang, roll_ang;
float dt;
float pitch_axes,roll_axes, yaw_axes, throttle_axes;

    int pitch =1500;
    int roll =1500;
    int throttle =1500;
    int yaw =1500;

    float cmd_altitude=3,altitude, altitude_err;


void odom_cb(const nav_msgs::Odometry::ConstPtr& odom_data){
    odomvar = *odom_data;
    altitude=odomvar.pose.pose.position.z;
}

/*
void yawfn(const sensor_msgs::Imu::ConstPtr& imu_data)
{imuvar= *imu_data;
q[0] = imuvar.orientation.w;
    q[1] = imuvar.orientation.x;
    q[2] = imuvar.orientation.y;
    q[3] = imuvar.orientation.z;
yaw_ang=atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
pitch_ang=asin(2*(q[0]*q[2]-q[3]*q[1]));
roll_ang=atan2(2*(q[0]*q[1]+q[3]*q[2]),1-2*(q[2]*q[2]+q[1]*q[1]) );
}*/




void joyfn(const sensor_msgs::Joy::ConstPtr& joy){
    //ROS_INFO("1");
    joyvar = *joy;

    pitch_axes=joyvar.axes[4];
    roll_axes=joyvar.axes[3];
    yaw_axes=joyvar.axes[0];
    throttle_axes=joyvar.axes[1];
if (joyvar.axes[7] >= 0.5 && fl_takeoff==false && fl_land==false)
	{fl_manual=true;
	fl_takeoff=true;
ROS_INFO("TAKE OFF");}

if (joyvar.axes[7] <= -0.5 && fl_land==false && fl_takeoff==false)
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

    ros::init(argc, argv, "joystick_erle");
    ros::NodeHandle n;

    ros::Rate r(rate);

    dt=1/rate;

  //  ros::Subscriber imu_yaw = n.subscribe("/mavros/imu/data", 10, yawfn);
    ros::Subscriber joy_ip = n.subscribe("joy", 10, joyfn);
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>
			("mavros/local_position/odom", 10, odom_cb);

    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    //ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>
    //        ("/mavros/setpoint_position/local", 10);
    //ros::Publisher local_vel_pub = n.advertise<geometry_msgs::TwistStamped>
     //       ("/mavros/setpoint_velocity/cmd_vel", 10);
    //ros::Publisher att_pub = n.advertise<mavros_msgs::AttitudeTarget>
      //      ("/mavros/setpoint_raw/attitude", 10);
    ros::Publisher rc_override_pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);



    mavros_msgs::SetMode srv_setMode;
    mavros_msgs::CommandBool srv;
    mavros_msgs::CommandTOL srv_takeoff;



    //geometry_msgs::PoseStamped pose;
    //geometry_msgs::TwistStamped vel;
    ////////////////////////////////////////////
    /////////////////GUIDED/////////////////////
    ////////////////////////////////////////////
      while (ros::ok() )
    {







    if (fl_takeoff==true)
    {//fl_manual=false;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "ALT_HOLD";
    if(cl.call(srv_setMode)){
        ROS_INFO("setmode send ok %d value:", srv_setMode.response.success);
    }else{
        ROS_ERROR("Failed SetMode");
        return -1;
    }
    ////////////////////////////////////////////
    ///////////////////ARM//////////////////////
    ////////////////////////////////////////////
  
    srv.request.value = true;
    if(arming_cl.call(srv)){
        ROS_INFO("ARM send ok %d", srv.response.success);
    }else{
        ROS_ERROR("Failed arming or disarming");
    }
   ////////////////////////////////////////////
    /////////////////TAKEOFF////////////////////
    fl_takeoff=false;

}

////////////////////////////////////////////
    ///////////////////LAND/////////////////////
    ////////////////////////////////////////////
    
 	if (fl_land==true)
 	   {fl_manual=false;
    
     srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "LAND";
    if(cl.call(srv_setMode)){
        ROS_INFO("setmode send ok %d value:", srv_setMode.response.success);
    }else{
        ROS_ERROR("Failed SetMode");
        return -1;
    }

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

  

fl_land=false;
 		 }


int k_alt=50;

if (fl_land==false && fl_manual==true)
{altitude_err=cmd_altitude-altitude;

throttle=1500+altitude_err*k_alt;

if (throttle>=1700)
	throttle=1700;
if (throttle<=1300)
		throttle=1300;
        //printf("\n error %f  cmd_throttle %d",altitude_err,throttle);
throttle=200*(throttle_axes)+1500;
pitch=200*(-pitch_axes)+1500;
roll=200*(-roll_axes)+1500;
yaw=500*(-yaw_axes)+1500;

}

	 msg_override.channels[0] = roll;
        msg_override.channels[1] = pitch;
        msg_override.channels[2] = throttle;
        msg_override.channels[3] = yaw;
        msg_override.channels[4] = 65535;
        msg_override.channels[5] = 65535;
        msg_override.channels[6] = 65535;
        msg_override.channels[7] = 65535;



        rc_override_pub.publish(msg_override);

      ros::spinOnce();
      r.sleep();
    }

    return 0;
}
