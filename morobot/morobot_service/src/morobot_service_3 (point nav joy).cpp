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
#include "morobot_service/gotolocation.h" //SRV
#include "morobot_service/getlocation.h" //SRV
#include "morobot_service/setlocation.h" //SRV


#define BUF_LEN 4096
using namespace std;
FILE *fp;
char Textbuffer[BUF_LEN];

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
ros::Publisher service_pub; 

geometry_msgs::Point goal_position;
geometry_msgs::Quaternion goal_quarter;
move_base_msgs::MoveBaseActionGoal goal;


//**Command srv _ Service Server************************/
morobot_service::getlocation getlocation_cmd;
ros::ServiceServer getlocation_service;
morobot_service::setlocation setlocation_cmd;
ros::ServiceServer setlocation_service;
morobot_service::gotolocation goto_cmd;
ros::ServiceServer goto_service;

//**Command srv _ Service Client************************/
std_srvs::Empty m_request;


//goto goal id//
actionlib_msgs::GoalID goto_goal_id;

//Clear costmap Service Client//
ros::ServiceClient clear_costmap_client;

//======================================================================================================================================
// SaveLocation function

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
// SetLocation service

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
// OpenLocationFile function

bool OpenLocationFile(string str_location)
{
    bool bResult = false;
    string m_strFilePathName;

        m_strFilePathName = "/home/ubuntu/catkin_ws/src/morobot/DATA/" + str_location + ".txt";  
        fp = fopen(m_strFilePathName.c_str(), "r");  

        if(fp != NULL) //File Open Check
        {
            while(!feof(fp))
            {
                if(fgets(Textbuffer, sizeof(Textbuffer), fp) != NULL)
                {
                    char* ptr = strtok(Textbuffer, ",");
                    int icnt = 0;

                    while(ptr != NULL)
                    {   
                        ptr = strtok(NULL, ",");
                        switch(icnt)
                        {
                            case 0:
                                if(ptr != NULL)
                                {
                                    _pGoal_pose.goal_positionX = atof(ptr);
                                }
                                break;
                            case 1:
                                _pGoal_pose.goal_positionY = atof(ptr);
                                break;
                            case 2:
                                _pGoal_pose.goal_quarterX = atof(ptr);
                                break;
                            case 3:
                                _pGoal_pose.goal_quarterY = atof(ptr);
                                break;
                            case 4:
                                _pGoal_pose.goal_quarterZ = atof(ptr);  
                                break;
                            case 5:
                                _pGoal_pose.goal_quarterW = atof(ptr);
                                break;
                        }
                        icnt++;
                    }
                    bResult = true; 
                }
                else
                {
                    bResult = false; 
                }
            }                
            fclose(fp);
        }
        else
        {
            ROS_INFO("File Open Fail: %s", str_location.c_str());
            bResult = false;
        }
    

    return bResult;
}




//======================================================================================================================================
// setGoal function

void setGoal(move_base_msgs::MoveBaseActionGoal& goal)
{
    ros::Time now = ros::Time(0); //ros::Time::now();

    goal.header.frame_id="map";
    goal.header.stamp=now;
    goal.goal_id.stamp = now;
    goal.goal.target_pose.header.stamp = now;
    goal.goal.target_pose.header.frame_id = "map";
    goal.goal.target_pose.pose.position.x = _pGoal_pose.goal_positionX;
    goal.goal.target_pose.pose.position.y = _pGoal_pose.goal_positionY;
    goal.goal.target_pose.pose.position.z = _pGoal_pose.goal_positionZ;
    goal.goal.target_pose.pose.orientation.x = _pGoal_pose.goal_quarterX;
    goal.goal.target_pose.pose.orientation.y = _pGoal_pose.goal_quarterY;
    goal.goal.target_pose.pose.orientation.z = _pGoal_pose.goal_quarterZ;
    goal.goal.target_pose.pose.orientation.w = _pGoal_pose.goal_quarterW;

    service_pub.publish(goal);
    printf("setGoal call: %.5f, %.5f !!\n", _pGoal_pose.goal_positionX, _pGoal_pose.goal_positionY);
}



//======================================================================================================================================
// Goto_Command service

bool Goto_Command(morobot_service::gotolocation::Request &req, 
				  morobot_service::gotolocation::Response &res)
{
	bool bResult = false;


    bResult = OpenLocationFile(req.Location);
    printf("Goto bResult: %d \n", bResult);
    res.goal_positionX = _pGoal_pose.goal_positionX;
    res.goal_positionY = _pGoal_pose.goal_positionY;
    res.goal_quarterX  = _pGoal_pose.goal_quarterX;
    res.goal_quarterY  = _pGoal_pose.goal_quarterY;
    res.goal_quarterZ  = _pGoal_pose.goal_quarterZ;
    res.goal_quarterW  = _pGoal_pose.goal_quarterW;
    goto_goal_id.id = req.Location;

    ROS_INFO("goto_id.id: %s", goto_goal_id.id.c_str());
    //costmap clear call//
    clear_costmap_client.call(m_request);


        ROS_INFO("Goto Nomal Loop !");


        setGoal(goal);
        bResult = true;
        

    


	/*
	string Location
    int32 mark_id
    int32 movement
    ---
    float64 goal_positionX
    float64 goal_positionY
    float64 goal_quarterX
    float64 goal_quarterY
    float64 goal_quarterZ
    float64 goal_quarterW
    bool command_Result
	*/
	res.command_Result = bResult;


	return true;
}


//======================================================================================================================================
//PS4 joypad

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    //printf("JOY: %d, %d, %d, %d, %d  \n",  joy->buttons[1], joy->buttons[2], joy->buttons[3], joy->buttons[4], joy->buttons[5]);
    
    if(joy->buttons[1] == 1) //DOCK goto...
    {
        OpenLocationFile("DOCK");
        goto_goal_id.id = "DOCK";
    }
    else if(joy->buttons[2] == 1) //POINT1 goto...
    {
        OpenLocationFile("POINT1");
        goto_goal_id.id = "POINT1";
    }
    else if(joy->buttons[3] == 1) //POINT2 goto...
    {
        OpenLocationFile("POINT2");
        goto_goal_id.id = "POINT2";
    }
    
    ROS_INFO("goto_id.id: %s", goto_goal_id.id.c_str());
    //costmap clear call//
    clear_costmap_client.call(m_request);


    setGoal(goal);
    
    
    
}




//======================================================================================================================================
//main function

int main (int argc, char** argv)
{

    ros::init(argc, argv, "morobot_service", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;


    service_pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 10);


    //Joystick//
    ros::NodeHandle njoy;
    ros::Subscriber joy_sub = njoy.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);


    //Command Service//
    ros::NodeHandle service_h;
    goto_service = service_h.advertiseService("goto_cmd", Goto_Command);
    setlocation_service = service_h.advertiseService("setlocation_cmd", SetLocation_Command);


    //Clear costmaps//
    ros::NodeHandle client_h;
    clear_costmap_client = client_h.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");


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
