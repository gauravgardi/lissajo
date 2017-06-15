#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry pos_feed;

float q[4], q_tgt[4];
float yaw_ang, yaw_tgt, pitch_ang, roll_ang;
float d = 0.1;

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

float** matmul(float** Rx,float** Ry)
{
	int sum;
	float** R;
	for(int i = 0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{	sum = 0;
			for(int k=0;k<3;k++)
			{
				sum+=Ry[i][k]*Rx[k][j];
			}
			R[i][j] = sum;
		}
	}
	return R;
}

int main(int argc, char **argv)
{

    //int rate = 20;

    ros::init(argc, argv, "transform");
    ros::NodeHandle nh;

    //ros::Rate r(rate);

    //dt=1/rate;
    ros::Subscriber imu_yaw = nh.subscribe("mavros/local_position/odom", 10, feedbackfn);
    

    ros::Publisher p1_pub = nh.advertise<geometry_msgs::Pose>
            ("p1/odom", 10);
    ros::Publisher p2_pub = nh.advertise<geometry_msgs::Pose>
            ("p2/odom", 10);
    ros::Publisher p3_pub = nh.advertise<geometry_msgs::Pose>
            ("p3/odom", 10);                       
    ros::Publisher p4_pub = nh.advertise<geometry_msgs::Pose>
            ("p4/odom", 10);

     geometry_msgs::Pose pos1;
     geometry_msgs::Pose pos2;
     geometry_msgs::Pose pos3;
     geometry_msgs::Pose pos4;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    float d1[3] = {d*2/sqrt(2), d*2/sqrt(2), 0};
    float d2[3] = {d*2/sqrt(2), -d*2/sqrt(2), 0};
    float d3[3] = {-d*2/sqrt(2), d*2/sqrt(2), 0};
    float d4[3] = {-d*2/sqrt(2), -d*2/sqrt(2), 0};

    float** Rz; // rotation matrix in ZYX format
    float** Ry;
    float** Rx;
    float** R;
    printf("started");
    while(ros::ok())
    {
    	Rx[0][0] = 1;
    	Rx[0][1] = 0;
    	Rx[0][2] = 0;
    	Rx[1][0] = 0;
    	Rx[1][1] = cos(roll_ang);
    	Rx[1][2] = -sin(roll_ang);
    	Rx[2][0] = 0;
    	Rx[2][1] = sin(roll_ang);
    	Rx[2][2] = cos(roll_ang);

    	Ry[0][0] = cos(pitch_ang);
    	Ry[0][1] = 0;
    	Ry[0][2] = -sin(pitch_ang);
    	Ry[1][0] = 0;
    	Ry[1][1] = 1;
    	Ry[1][2] = 0;
    	Ry[2][0] = sin(pitch_ang);
    	Ry[2][1] = 0;
    	Ry[2][2] = cos(pitch_ang);

    	Rz[0][0] = cos(yaw_ang);
    	Rz[0][1] = -sin(yaw_ang);
    	Rz[0][2] = 0;
    	Rz[1][0] = sin(yaw_ang);
    	Rz[1][1] = cos(yaw_ang);
    	Rz[1][2] = 0;
    	Rz[2][0] = 0;
    	Rz[2][1] = 0;
    	Rz[2][2] = 1;

    	R = matmul(Rx,Ry);
    	R = matmul(R,Rz);
    	printf("%f ", R[0][0]);

    	pos1.position.x = R[0][0]*d1[0] + R[0][1]*d1[1] + R[0][2]*d1[2];
    	pos1.position.y = R[1][0]*d1[0] + R[1][1]*d1[1] + R[1][2]*d1[2];
    	pos1.position.z = R[2][0]*d1[0] + R[2][1]*d1[1] + R[2][2]*d1[2];

    	pos1.position.x = R[0][0]*d2[0] + R[0][1]*d2[1] + R[0][2]*d2[2];
    	pos1.position.y = R[1][0]*d2[0] + R[1][1]*d2[1] + R[1][2]*d2[2];
    	pos1.position.z = R[2][0]*d2[0] + R[2][1]*d2[1] + R[2][2]*d2[2];

    	pos1.position.x = R[0][0]*d3[0] + R[0][1]*d3[1] + R[0][2]*d3[2];
    	pos1.position.y = R[1][0]*d3[0] + R[1][1]*d3[1] + R[1][2]*d3[2];
    	pos1.position.z = R[2][0]*d3[0] + R[2][1]*d3[1] + R[2][2]*d3[2];

    	pos1.position.x = R[0][0]*d4[0] + R[0][1]*d4[1] + R[0][2]*d4[2];
    	pos1.position.y = R[1][0]*d4[0] + R[1][1]*d4[1] + R[1][2]*d4[2];
    	pos1.position.z = R[2][0]*d4[0] + R[2][1]*d4[1] + R[2][2]*d4[2];

    	p1_pub.publish(pos1);
    	p2_pub.publish(pos2);
    	p3_pub.publish(pos3);
    	p4_pub.publish(pos4);

    	ros::spinOnce();
        rate.sleep();
    }
}