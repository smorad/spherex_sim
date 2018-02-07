#ifndef _LIDAR_PLUGIN_HH_
#define _LIDAR_PLUGIN_HH
// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// ROS
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

/* Publish LiDAR scan data from Gazebo to ROS */

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class LidarPlugin : public GpuRaySensor
  {
    /// \brief Constructor
    public: LidarPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(const std::string &_worldName, sdf::ElementPtr _sdf)
    {
      std::cerr << "Loading lidar plugin...\n";

      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(_worldName)

      // Read from lidar topic published by 
      // Create a topic name
      std::string gazebo_lidar_topic = "~/SphereX/SphereX/SphereXMid/sensor/scan";
      std::string ros_lidar_topic = "gazebo/SphereX/lidar_stream";
      std::cerr << "Reading from " << gazebo_lidar_topic << "\n";
      std::cerr << "Writing to " << ros_lidar_topic << "\n";

      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe(gazebo_lidar_topic,
         &SphereXPlugin::OnMsg, this);

	  // Load ROS
	  // Initialize ros, if it has not already bee initialized.
		if (!ros::isInitialized())
		{
		  int argc = 0;
		  char **argv = NULL;
		  ros::init(argc, argv, "gazebo_client",
			  ros::init_options::NoSigintHandler);
		}

		// Create our ROS node. This acts in a similar manner to
		// the Gazebo node
        this->rosNode = new ros::NodeHandle("gazebo_client");
        
        // Create publisher
        this->ros_lidar_stream = this->rosNode.advertise<std_msgs::String>(ros_lidar_topc)

		// Spin up the queue helper thread.
		this->rosQueueThread =
		  std::thread(std::bind(&SphereXPlugin::QueueThread, this));
    }


	public: void OnRosMsg(const gazebo_sensor msgs::RaySensor)
	{
        std::cerr << "gazebo got msg, handling\n"; 
        this->ros_lidar_stream.publish("Testing ros publishing");
        // Translate gazebo packet to ros packet
        sensor_msgs::LaserScan scan;
        /*
        Header header;
        float32 angle_min;
        float32 angle_max;
        float32 angle_increment;
        float32 time_increment;
        float32 scan_time;
        float32 range_min;
        float32 range_max;
        float32[] ranges;
        float32[] intensities;
        std::cerr << "gazebo sensor data " << gazebo_sensor << "\n";
        scan.header.stamp = 0;
        scan.angle_min = 0//gazebo_sensor.GetAngleMin();
        scan.angle_max = 1//gazebo_sensor.GetAngleMax();
        scan.angle_increment = 0//gazebo_sensor.GetAngleResolution();
        scan.time_increment = 0;
        scan.range_min = 0;
        scan.range_max = 20;
        scan.ranges = gazebo_sensor.GetRanges();
        scan.intensities = [];
        */
	}

	/// \brief ROS helper function that processes messages
	private: void QueueThread()
	{
	  static const double timeout = 0.01;
	  while (this->rosNode->ok())
	  {
		this->rosQueue.callAvailable(ros::WallDuration(timeout));
	  }
	}

    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;


	// ROS

    /// \brief A node use for ROS transport
	private: std::unique_ptr<ros::NodeHandle> rosNode;

	/// \brief A ROS subscriber
	private: ros::Subscriber rosSub;

	/// \brief A ROS callbackqueue that helps process messages
	private: ros::CallbackQueue rosQueue;

	/// \brief A thread the keeps running the rosQueue
	private: std::thread rosQueueThread;

    private: ros::Publisher ros_lidar_stream;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(SphereXPlugin)
}
#endif
