#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo-7/gazebo/transport/PublicationTransport.hh>
#include <gazebo-7/gazebo/transport/Publication.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

namespace gazebo {
    /// \brief A plugin to control a Velodyne sensor.

    class SphereXPlugin : public ModelPlugin {
        /// \brief Constructor
    public:

        SphereXPlugin() {
        }

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.

        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
            std::cerr << "Loading spherex plugin...\n";
            // Safety check
            if (_model->GetJointCount() == 0) {
                std::cerr << "Invalid joint count, plugin not loaded\n";
                return;
            }

            // Create the node
            this->node = transport::NodePtr(new transport::Node());
            this->node->Init(_model->GetWorld()->GetName());

            // rotation setup
            this->model = _model;
            this->sdf = _sdf;
            this->SetupRot();
            this->SetupLidar();




            // Create a topic name
            std::cerr << "Reading from " << this->rot_cmd_topic << "\n";
            std::cerr << "Reading from " << this->lidar_cmd_topic << "\n";
            std::cerr << "Reading from " << this->scan_topic << "\n";

        }

        /// \brief Set the velocity of the Velodyne
        /// \param[in] _vel New target velocity

        void SetVelocity(const double &_vel) {
            // Set the joint's target velocity.
            this->model->GetJointController()->SetVelocityTarget(
                    this->joint->GetScopedName(), _vel);
        }

        /// \brief Handle incoming message
        /// \param[in] _msg Repurpose a vector3 message. This function will
        /// only use the x component.
    private:

        void VelOnMsg(ConstVector3dPtr &_msg) {
            this->SetVelocity(_msg->x());
        }

        void Sweep(ConstVector3dPtr &_msg) {
            // should be in Hz
            /* double update_rate = this->lidar_sensor->GetUpdateRate();
             for (int slice=0; slice<this->slices; slice++){
                 double theta = this->v_angular_res * slice;
                 this->lidar_sensor->SetActive(true);
             }
             this->lidar_sensor->SetActive(false);*/

            // Turn lidar on, start capturing data, sweep sensor, turn off


        }

        void WriteScan(ConstLaserScanStampedPtr &_msg) { // const boost::shared_ptr<msgs::RaySensor>
            // We got a new message from the Gazebo sensor.  Stuff a
            // corresponding ROS message and publish it.
            this->SetVelocity(3);
            sensor_msgs::LaserScan laser_msg;
            laser_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
            //laser_msg.header.frame_id = this->frame_name_;
            laser_msg.angle_min = _msg->scan().angle_min();
            laser_msg.angle_max = _msg->scan().angle_max();
            laser_msg.angle_increment = _msg->scan().angle_step();
            laser_msg.time_increment = 0; // instantaneous simulator scan
            laser_msg.scan_time = 0; // not sure whether this is correct
            laser_msg.range_min = _msg->scan().range_min();
            laser_msg.range_max = _msg->scan().range_max();
            laser_msg.ranges.resize(_msg->scan().ranges_size());
            std::copy(_msg->scan().ranges().begin(),
                    _msg->scan().ranges().end(),
                    laser_msg.ranges.begin());
            laser_msg.intensities.resize(_msg->scan().intensities_size());
            std::copy(_msg->scan().intensities().begin(),
                    _msg->scan().intensities().end(),
                    laser_msg.intensities.begin());
            this->scan_pub.publish(laser_msg);
        }

        void SetupRot() {

            this->joint = this->model->GetJoints()[0];
            this->pid = common::PID(0.1, 0, 0);
            this->model->GetJointController()->SetVelocityPID(
                    this->joint->GetScopedName(), this->pid);
            double velocity = 0;
            //if (this->sdf->HasElement("velocity"))
            //    velocity = this->sdf->Get<double>("velocity");

            this->SetVelocity(velocity);

            // Subscribe to the topic, and register a callback
            this->vel_sub = this->node->Subscribe(rot_cmd_topic,
                    &SphereXPlugin::VelOnMsg, this);

        }

        void SetupLidar() {
            if (!ros::isInitialized()) {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }
            this->lidar_sensor = sensors::get_sensor("sensor");
            this->lidar_sensor->SetActive(false);
            std::cerr << "model is " << this->lidar_sensor << "\n";
            //this->lidar_cmd_sub = this->node->Subscribe(lidar_cmd_topic,
            //        &SphereXPlugin::WriteScan, this);
            this->scan_sub = this->node->Subscribe(scan_topic,
                    &SphereXPlugin::WriteScan, this);
            //this->scan_pub = this->node->Advertise<msgs::RaySensor>(this->output_scan_topic);
            this->scan_pub = this->ros_node.advertise<sensor_msgs::LaserScan>(output_scan_topic, 1000);

        }

        // rot stuff
        double full_rot = 2 * 3.1415;
        int slices = 64;
        double v_angular_res = full_rot / slices;


        sensors::SensorPtr lidar_sensor;
        transport::NodePtr node;
        transport::SubscriberPtr vel_sub;
        transport::SubscriberPtr lidar_cmd_sub;
        transport::SubscriberPtr scan_sub;
        ros::NodeHandle ros_node;
        ros::Publisher scan_pub;
        physics::ModelPtr model;
        sdf::ElementPtr sdf;
        physics::JointPtr joint;
        common::PID pid;
        std::string rot_cmd_topic = "~/SphereX/vel_cmd";
        std::string lidar_cmd_topic = "~/SphereX/lidar";
        std::string scan_topic = "~/SphereX/SphereX/SphereXMid/sensor/scan";
        // CANNOT HAVE ~/ prefix or gazebo will overwrite it
        std::string output_scan_topic = "raw_lidar_scan";
    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(SphereXPlugin)
}
#endif
