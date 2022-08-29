////morobot Service ROS Package_Ver 0.1
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h> //teb_poses...
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h> //teb markers..
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h> //bumper
#include <sensor_msgs/Joy.h> //add 
#include <sensor_msgs/Range.h> //Ultrasonic//
#include <actionlib_msgs/GoalID.h>

#include <sstream>
#include <vector>
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <time.h>
#include <thread> //thread add...
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <sys/types.h>
#include <dirent.h>
#include <error.h>
#include <cstdlib> //std::system call
#include <algorithm>
#include <signal.h>


//Service
#include "morobot_service/setlocation.h" //SRV


using namespace std;
FILE *fp;


// Active map Check //
bool m_bActive_map_check = false;



//tf_Pose.(map->base_footprint TF)..
typedef struct TF_POSE
{
    double poseTFx = 0.0;
    double poseTFy = 0.0;
    double poseTFz = 0.0;
    double poseTFqx = 0.0;
    double poseTFqy = 0.0;
    double poseTFqz = 0.0;
    double poseTFqw = 1.0;

}TF_POSE;
TF_POSE _pTF_pose;



//goal_pose...
typedef struct GOAL_POSE
{
    float goal_positionX = 0.0;
    float goal_positionY = 0.0;
    float goal_positionZ = 0.0;
    float goal_quarterX = 0.0;
    float goal_quarterY = 0.0;
    float goal_quarterZ = 0.0;
    float goal_quarterW = 1.0;

}GOAL_POSE;
GOAL_POSE _pGoal_pose;



//Publisher & msg define....
morobot_service::setlocation setlocation_cmd;
ros::ServiceServer setlocation_service;

//======================================================================================================================================
// SaveLocation : ??? ???? ??

bool SaveLocation(string str_location, 
                  float m_fposeAMCLx, float m_fposeAMCLy,
                  float m_fposeAMCLqx, float m_fposeAMCLqy, float m_fposeAMCLqz, float m_fposeAMCLqw)
{
    bool bResult = false;

    string m_strFilePathName;
    m_strFilePathName = "/home/ubuntu/catkin_ws/src/morobot/DATA/" + str_location + ".txt";    
    fp = fopen(m_strFilePathName.c_str(), "w");
    if(fp == NULL)
    { 
        ROS_INFO("file is null");
        bResult = false;
    }
    else
    {
        fprintf(fp, "0,%lf,%lf,%lf,%lf,%lf,%lf \n",
                m_fposeAMCLx, m_fposeAMCLy, m_fposeAMCLqx, m_fposeAMCLqy, m_fposeAMCLqz, m_fposeAMCLqw);
        fclose(fp);
        bResult = true;
    }

    return bResult;
}



//======================================================================================================================================
// SetLocation service : tf ??? ???? ??? ???? goal ??? ???? ???

bool SetLocation_Command(morobot_service::setlocation::Request  &req, 
					     morobot_service::setlocation::Response &res)
{
	bool bResult = false;


    
        /*
        res.command_Result = SaveLocation(req.Location, 
                                        _pAMCL_pose.poseAMCLx, _pAMCL_pose.poseAMCLy,
                                        _pAMCL_pose.poseAMCLqx, _pAMCL_pose.poseAMCLqy, _pAMCL_pose.poseAMCLqz, _pAMCL_pose.poseAMCLqw);
        
        res.goal_positionX = _pAMCL_pose.poseAMCLx;
        res.goal_positionY = _pAMCL_pose.poseAMCLy;
        res.goal_quarterX  = _pAMCL_pose.poseAMCLqx;
        res.goal_quarterY  = _pAMCL_pose.poseAMCLqy;
        res.goal_quarterZ  = _pAMCL_pose.poseAMCLqz;
        res.goal_quarterW  = _pAMCL_pose.poseAMCLqw;
        */

        res.command_Result = SaveLocation(req.Location, 
                                _pTF_pose.poseTFx,_pTF_pose.poseTFy,
                                _pTF_pose.poseTFqx,_pTF_pose.poseTFqy,_pTF_pose.poseTFqz,_pTF_pose.poseTFqw);
        
        res.goal_positionX = _pTF_pose.poseTFx;
        res.goal_positionY = _pTF_pose.poseTFy;
        res.goal_quarterX  = _pTF_pose.poseTFqx;
        res.goal_quarterY  = _pTF_pose.poseTFqy;
        res.goal_quarterZ  = _pTF_pose.poseTFqz;
        res.goal_quarterW  = _pTF_pose.poseTFqw;
    

    /*
	string Location
    ---
    float goal_positionX
    float goal_positionY
    float goal_quarterX
    float goal_quarterY
    float goal_quarterZ
    float goal_quarterW
    bool command_Result
	*/
	bResult = res.command_Result;

	return true;
}





//======================================================================================================================================
//main ??
int main (int argc, char** argv)
{

    ros::init(argc, argv, "morobot_service", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;


    //Command Service//
    ros::NodeHandle service_h;
    setlocation_service = service_h.advertiseService("setlocation_cmd", SetLocation_Command);


    //TF transform//
    tf::TransformListener listener;


    ros::Rate loop_rate(30); //30hz



    while(ros::ok())
    {
        ros::spinOnce();
	
        //Get Active map param..//
        nh.getParam("active_map", m_bActive_map_check);
        if(m_bActive_map_check)
        {
            //map to base_footprint TF Pose////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            tf::StampedTransform transform;
            try
            {
                listener.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(1.0));
                listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);

                geometry_msgs::TransformStamped ts_msg;
                tf::transformStampedTFToMsg(transform, ts_msg);

                _pTF_pose.poseTFx = ts_msg.transform.translation.x;
                _pTF_pose.poseTFy = ts_msg.transform.translation.y;
                _pTF_pose.poseTFz = ts_msg.transform.translation.z;
                _pTF_pose.poseTFqx = ts_msg.transform.rotation.x;
                _pTF_pose.poseTFqy = ts_msg.transform.rotation.y;
                _pTF_pose.poseTFqz = ts_msg.transform.rotation.z;
                _pTF_pose.poseTFqw = ts_msg.transform.rotation.w;

                
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("[TF_Transform_Error(map to base_footprint)]: %s", ex.what());
            }
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            

        }
    }


}
