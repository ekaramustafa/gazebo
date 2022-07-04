#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{

	class VelodynePlugin : public ModelPlugin
	{
	//constructor
	
	public: VelodynePlugin() {}
	
	//A node use for ROS transport
	private: std::unique_ptr<ros::NodeHandle> rosNode;
	
	//ROS subscriber
	private: ros::Subscriber rosSub;
	
	//ROS Callbackqueue that helps process messages
	private: ros::CallbackQueue rosQueue;
	
	//A thread the kepps running the rosQueue
	private: std::thread rosQueueThread;
	
	
	private: physics::ModelPtr model;
	
	private: physics::JointPtr joint;
	
	private: common::PID pid;
	
	private: transport::NodePtr node;
	
	private: transport::SubscriberPtr sub;
	
	//The load func is called by Gazebo when the plugin is inserted into sim
	
	//_model is a pointer to the model that this plugin is attached to
	
	//_sdf is a pointer to the plugin's SDF element
	public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{
	//safety check
	
	if(_model->GetJointCount() == 0)
	{
		std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
	
		return;
	}
	
	//Storing the model pointer 
	
	this->model = _model;
	
	//having rotational joint
	this->joint = _model->GetJoints()[0];
	
	//setup a p-comtroller, with a gain of 0.1
	this->pid = common::PID(0.1, 0, 0);
	
	//Apply the P-controller to the joint
	this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(),this->pid);
	
	
	//Default to zero velocity
	double velocity = 0;
	
	
	//check that the velocity element exists, then read the value
	if (_sdf->HasElement("velocity")) 
	{
		velocity = _sdf->Get<double>("velocity");
	}
	
	this->SetVelocity(velocity);
	
	//Set the joint's target velocity. This target velocity is just for demonstration purposes
	
	
	this->model->GetLinks()[0]->SetLinearVel({0,1,0});
	this->model->GetLinks()[1]->SetLinearVel({0,1,0});
	
	
	//creating node
	this->node = transport::NodePtr(new transport::Node());
	this->node->Init(this->model->GetWorld()->Name());
	
	
	//creating a topic name
	std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";
	
	
	//subscribing to the topic, and register a callback
	this->sub = this->node->Subscribe(topicName, &VelodynePlugin::OnMsg, this);
	
	//Initialize ros, if it has not already be initialized
	if(!ros::isInitialized())
	{
	
		int argc =0;
		
		char **argv = NULL;
		ros::init(argc, argv, "gazebo_client",ros::init_options::NoSigintHandler);
	}
	
	
	//Creating our ROS Node, very similar to the Gazebo Node
	this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
	
	
	//Create a named topic, and subscribe to it.
	ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>("/" + this->model->GetName()+ "/vel_cmd",1,boost::bind(&VelodynePlugin::OnRosMsg,this,_1),ros::VoidPtr(),&this->rosQueue);
	
	this->rosSub = this->rosNode->subscribe(so);
	
	
	//spin up the queue helper thread
	this->rosQueueThread = std::thread(std::bind(&VelodynePlugin::QueueThread,this));
	
	
	}
	
	//Handle an incoming message from ROS
	public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
	{
		this->SetVelocity(_msg->data);
	}
	
	
	//Ros helper function that processes messages
	//it does the same thing with ros.spin() function
	private: void QueueThread()
	{
		static const double timeout = 0.01;
		while(this->rosNode->ok())
		{
			this->rosQueue.callAvailable(ros::WallDuration(timeout));
		}
	}
	
	private: void OnMsg(ConstVector3dPtr &_msg)
	{
		this->SetVelocity(_msg->x());
	}
	
	public: void SetVelocity(const double &_vel)
	{
		this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), _vel);
	
	}

	};
	

	//Telling Gazebo about this plugin so that Gazebo can call Load on this plugin
	GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)

}
#endif
