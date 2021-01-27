#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "control_v2/MoveJoints.h"
#include "control_v2/GetJoint.h"
#include <string.h>
using namespace std;

class Nao_control
{
public:
    // ros handler
    ros::NodeHandle nh_;
    ros::ServiceClient m_client;
    ros::ServiceClient g_client;
    //ros::ServiceClient t_client;
    control_v2::MoveJoints m_srv;
    control_v2::GetJoint g_srv;
    //control_v2::GetTransform t_srv;

    // subscriber to joint states
    //ros::Subscriber sensor_data_sub;

    Nao_control()
    {
        //sensor_data_sub=nh_.subscribe("/joint_states",1, &Nao_control::sensorCallback, this);
        m_client = nh_.serviceClient<control_v2::MoveJoints>("move_joints_service");
        g_client = nh_.serviceClient<control_v2::GetJoint>("get_joint_service");
        //t_client = nh_.serviceClient<control_v2::GetTransform>("get_transform_service");

    }
    ~Nao_control()
    {
    }

  //handler for joint states
    void sensorCallback(const sensor_msgs::JointState::ConstPtr& jointState)
    {
      
    }

    // // function to prepare a request to be send to the python service move_service.py. 
    // static void move_joints(tutorial_5::MoveJoints* srv, string names, vector<double> angles, vector<double> times){
    //     // get the string containing the joint names to move
    //     stringstream ss;
    //     ss << names;

    //     // the client chooses the input values:
    //     double maxSpeedFraction = 0.1;
    //     bool absolute = true;

    //     srv->request.names = ss.str();
    //     srv->request.angles = angles;
    //     srv->request.maxSpeedFraction = maxSpeedFraction;
    //     srv->request.times = times;
    //     srv->request.absolute = absolute;
    // }

    //  // function used for the fisrt task for instance
    // void move_both_shoulders(tutorial_5::MoveJoints* srv){
    //     // LShoulderPitch, RShoulderPitch
    //     string names = "LShoulderPitch,RShoulderPitch";
        
    //     // setting the angles
    //     vector<double> angles;
    //     angles.push_back(0.5);
    //     angles.push_back(0.5);

    //     // setting the times
    //     vector<double> times;
    //     times.push_back(5);
    //     times.push_back(5);

    //     move_joints(srv, names, angles, times);

    // }

    // // TODO: create function for each task
    int joints_control(){

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<control_v2::MoveJoints>("move_joints_service");
        control_v2::MoveJoints srv;

        //move_both_shoulders(&srv);


        if (m_client.call(srv))
        {
            ROS_INFO("response: %ld", (short int)srv.response.err_code);
        }
        else
        {
            ROS_ERROR("Failed to call service");
            return 1;
        }

        ros::spin();
        return 0;
    }

    // ############################################################################
    // position requesting for joint/chain/sensor name

    vector<double> request_positions(string name){
        vector<double> coordinates;

        // setting interested joint/chain/sensor name
        stringstream ss;
        ss << name;
        int frame = 0;
        bool useSensorValues = false;

        g_srv.request.name = ss.str();
        g_srv.request.frame = frame;
        g_srv.request.useSensorValues = useSensorValues;

        if (g_client.call(g_srv))
        {
            for(int i=0; i<6; i++){
                coordinates.push_back((double)g_srv.response.coordinates[i]);
            }
                ROS_INFO("motion completed successfully");
        }
        else
        {
            ROS_ERROR("failed to call service");
        }
        return coordinates;
    }

    // ############################################################################
    // moves robot end-effector to desired pos & orient in cartesian
    // if init_posture is set to true we first reinitialize the posture and then go to target

    int move_endEffector(string name, double x, double y, double z, 
                                      double roll, double pitch, double yaw,
                                      bool init_posture=true, bool absolute=false){

        // setting name of desired end-effector
        stringstream ss;
        ss << name;
        
        // setting frame: {FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2}.
        int frame = 0;

        // using sensor values
        bool useSensorValues = false;

        // setting desired cartesian pos & orient
        //  -> used axis mask (depends on users input)
        //  -> only pos: 7; pos & orient: 63
        int axisMask = 63;

        // setting position of joint/chain
        vector<double> target;
        if(absolute){
            target.push_back(x);
            target.push_back(y);
            target.push_back(z);
            target.push_back(roll);
            target.push_back(pitch);
            target.push_back(yaw);
        } else {
            // get current coordinates
            vector<double> current = request_positions(name);
            double dx = x + current[0];
            double dy = y + current[1];
            double dz = z + current[2];
            double droll  = roll  + current[3];
            double dpitch = pitch + current[4];
            double dyaw   = yaw   + current[5];
            target.push_back(dx);
            target.push_back(dy);
            target.push_back(dz);
            target.push_back(droll);
            target.push_back(dpitch);
            target.push_back(dyaw);
        }
        // setting desired fraction of max velocity
        double maxSpeedFraction = 0.5;

        // setting desired execution time
        //  -> the server calls fct. positionInterpolations() if times is no empty
        //  -> and uses maxSpeedFraction for fct. setPositions() otherwise
        vector<double> times;
        times.push_back(3);
        times.push_back(4);

        // sending service request
        m_srv.request.name = ss.str();
        m_srv.request.frame = frame;
        m_srv.request.axisMask = axisMask;
        m_srv.request.maxSpeedFraction = maxSpeedFraction;
        m_srv.request.times = times;
        m_srv.request.target = target;
        m_srv.request.useSensorValues = useSensorValues;
        m_srv.request.absolute = absolute;
        m_srv.request.init_posture = init_posture;

        if (m_client.call(m_srv))
        {
            ROS_INFO("response: %ld", (short int)m_srv.response.err_code);
        }
        else
        {
            ROS_ERROR("Failed to call service");
            return 1;
        }
        return 0;
    }

};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_v2");

    Nao_control ic;
    // request the actual position of joint/chain/sensor wrt frame
    //ic.request_positions("RArm");
    //ic.request_positions("LArm");
    ic.joints_control();
    //ic.move_endEffector("LArm", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
    // ic.move_endEffector("LArm", 0.05, 0.05, 0.05, 0.0, 0.0, 0.0, false);
    //ic.move_endEffector("LArm", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);

}
