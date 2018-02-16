#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <math.h>
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
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

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
            std::cerr << "Reading from " << this->scan_cmd_topic << "\n";
            std::cerr << "Reading from " << this->scan_topic << "\n";

        }

        /// \brief Set the velocity of the Velodyne
        /// \param[in] _vel New target velocity

        void SetVelocity(const double &_vel) {
            // Set the joint's target velocity.
            this->model->GetJointController()->SetVelocityTarget(
                    this->joint->GetScopedName(), _vel);
        }

        void Sweep(const std_msgs::String::ConstPtr& _msg) {
            // in radians
            std::cerr << "Reached sweep" << std::flush;
            // in rad/s
            // runs at 5hz, so 12
            /*
            double circle = 2 * 3.1415;
            double step_size = 0.3;
            for (int step = 0; step < circle; step += step_size) {
                this->lidar_sensor->SetActive(true);
                // Update causing segfaults
                //this->lidar_sensor->Update(true);
                this->model->GetJointController()->SetJointPosition(
                        this->joint->GetScopedName(), step);
                //this->lidar_sen
            }*/
        }
        /// \brief Handle incoming message
        /// \param[in] _msg Repurpose a vector3 message. This function will
        /// only use the x component.
    private:

        void VelOnMsg(ConstVector3dPtr & _msg) {
            this->SetVelocity(_msg->x());
        }

        void WriteScan(ConstLaserScanStampedPtr &_msg) { // const boost::shared_ptr<msgs::RaySensor>
            // We got a new message from the Gazebo sensor.  Stuff a
            // corresponding ROS message and publish it.

            //std::cerr << "Reached writescan: is active:" <<this->lidar_sensor->IsActive()<<std::endl;
            sensor_msgs::LaserScan laser_msg;
            laser_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
            //laser_msg.header.frame_id = this->frame_name_;
            laser_msg.angle_min = _msg->scan().angle_min();
            laser_msg.angle_max = _msg->scan().angle_max();
            // runs at 30hz but does vertical instead of horizontal
            laser_msg.angle_increment =
                    laser_msg.time_increment = 0; // instantaneous simulator scan
            // repurpose scan_time as angle theta between scan plane and ref frame

            laser_msg.scan_time = this->joint->GetAngle(0).Radian(); // not sure whether this is correct
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
            // disable scanner so we publish precisely one scan per part of the
            // circle
            //this->lidar_sensor->SetActive(false);
        }

        void WriteCloud(ConstLaserScanStampedPtr &_msg) {
            sensor_msgs::PointCloud cloud_msg;
            cloud_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
            int slices = _msg->scan().count();
            int vslices = _msg->scan().vertical_count();
            double theta_step = _msg->scan().angle_step();
            double phi_step = _msg->scan().vertical_angle_step();
            std::cerr << "write cloud" << std::endl;
            cloud_msg.points.resize(slices * vslices);
            for (int vslice = 0; vslice < vslices; ++vslice) {
                for (int slice = 0; slice < slices; ++slice) {
                    geometry_msgs::Point32 p;
                    double theta = slice * theta_step;
                    double phi = vslice * phi_step;
                    // index the jth vslice row and the ith slice column
                    std::cerr<<"vslice idx "<<vslice<< " slice idx "<<slice << " pt idx "<<vslice * _msg->scan().count() + slice 
                            << " theta "<<theta<< " phi "<< phi << std::endl;
                    double r = _msg->scan().ranges().Get(vslice * _msg->scan().count() + slice);
                    p.x = r * cos(theta) * cos(phi);//r * sin(phi) * cos(theta);
                    p.y = r * sin(theta) * cos(phi);//r * sin(phi) * sin(theta);
                    p.z = r * sin(phi);//r * cos(phi);
                    cloud_msg.points.push_back(p);
                }
            }
            this->cloud_pub.publish(cloud_msg);

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
            this->lidar_sensor->SetActive(true);
            this->scan_sub = this->node->Subscribe(scan_topic,
                    &SphereXPlugin::WriteScan, this);
            this->cloud_sub = this->node->Subscribe(scan_topic,
                    &SphereXPlugin::WriteCloud, this);
            // Workaround for class method callback
            //this->scan_cmd_sub = this->ros_node.subscribe(scan_cmd_topic, 1000, &SphereXPlugin::Sweep, this);
            this->scan_pub = this->ros_node.advertise<sensor_msgs::LaserScan>(output_scan_topic, 1000);
            this->cloud_pub = this->ros_node.advertise<sensor_msgs::PointCloud>(cloud_pub_topic, 1000);

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
        transport::SubscriberPtr cloud_sub;
        ros::NodeHandle ros_node;
        ros::Publisher scan_pub;
        ros::Publisher cloud_pub;
        ros::Subscriber scan_cmd_sub;
        physics::ModelPtr model;
        sdf::ElementPtr sdf;
        physics::JointPtr joint;
        common::PID pid;
        std::string rot_cmd_topic = "~/SphereX/vel_cmd";
        std::string scan_cmd_topic = "lidar_scan_cmd";
        std::string scan_topic = "~/SphereX/SphereX/SphereXMid/sensor/scan";
        // CANNOT HAVE ~/ prefix or gazebo will overwrite it
        std::string output_scan_topic = "raw_lidar_stream";
        std::string cloud_pub_topic = "raw_pointcloud_stream";
    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(SphereXPlugin)
}
#endif
