#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <math.h>
#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/ModelState.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo-7/gazebo/transport/PublicationTransport.hh>
#include <gazebo-7/gazebo/transport/Publication.hh>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <ros/subscribe_options.h>
#include "ros/callback_queue.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>





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

            this->model = _model;
            this->sdf = _sdf;
            this->SetupRot();
            this->SetupLidar();
            this->SetupCamera();
            this->SetupThruster();
            this->SetupIMU();

            // Bind physics update event to onupdate
            //this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            //        std::bind(&SphereXPlugin::OnUpdate, this));

        }

        // Runs every dynamics simulation iteration

        void OnUpdate() {
            // Apply thruster force
            if (!this->thrust_link){
                std::cerr << "Detecting thrust link..." << std::endl;
                this->thrust_link = this->model->GetLink("SphereX::SphereXMid");
            }
            this->thrust_link->AddForce(thrust_vec);
        }

        /// \brief Set the velocity of the Velodyne
        /// \param[in] _vel New target velocity

        void SetVelocity(const double &_vel) {
            // Set the joint's target velocity.
            this->model->GetJointController()->SetVelocityTarget(
                    this->joint->GetScopedName(), _vel);
        }

        void ApplyImpulse(const std_msgs::Float64MultiArray::Ptr &_msg) {
            // Apply impulse in the body frame
            // Msg in the form <x, y, z, duration, local>
            // Tickrate is 0.001s

            common::Time t;
            auto link = this->model->GetLink("SphereX::SphereXMid");
            auto model = this->model.get();
            //std::cerr << "Model: " << model->GetName() << std::endl;
            //std::cerr << "links size " << model->GetLinks().size() << std::endl;
            //for (int i = 0; i < model->GetLinks().size(); ++i) {
            //    std::cerr << "link: " << model->GetLinks().at(i)->GetName() << std::endl;
            //}
            if (!link) {
                std::cerr << "Not applying force, linkptr is " << link << std::endl;
                return;
            }
            // f = m * dv / t
            float force = 3 * _msg->data[3] / 0.001;
            math::Vector3 f(force * _msg->data[0], force * _msg->data[1], force * _msg->data[2]);
            if (_msg->data[4]) {
                link->AddForce(f);
            } else {
                link->AddRelativeForce(f);

            }
            //t.Sleep(_msg->data[3]);
            //link->SetForce(math::Vector3(0, 0, 0));
        }

        /// \brief Handle incoming message
        /// \param[in] _msg Repurpose a vector3 message. This function will
        /// only use the x component.
    private:

        void VelOnMsg(ConstVector3dPtr& _msg) {
            this->SetVelocity(_msg->x());
        }

        /*
        void WriteIMU(ros::Time t) {
            // Generate IMU data and apply timestamp to it

            sensor_msgs::Imu msg_out;
            msg_out.header.stamp = t;

            geometry_msgs::Vector3 ang_vel_out;
            ang_vel_out.x = ang_vel_in.X();
            ang_vel_out.y = ang_vel_in.Y();
            ang_vel_out.z = ang_vel_in.Z();
            msg_out.angular_velocity = ang_vel_out;

            auto ori_in = this->imu->Pose().Rot();
            geometry_msgs::Quaternion ori_out;
            ori_out.w = ori_in.W();
            ori_out.x = ori_in.X();
            ori_out.y = ori_in.Y();
            ori_out.z = ori_in.Z();
            msg_out.orientation = ori_out;
            
            auto lin_in = this->imu->LinearAcceleration(false);
            geometry_msgs::Vector3 lin_out;
            msg_out.linear_acceleration = lin_out;
            
            // no cov mats published
            this->imu_pub.publish(msg_out);

        }*/

        void WriteCam1(ConstImageStampedPtr& _msg) {
            /* Read a gazebo img and send a sensor_msg image to ros*/
            // Sometimes a null msg is sent, gazebo bug?
            if (_msg == nullptr) {
                std::cerr << "Camera received null image ptr: " << _msg << std::endl;
                return;
            }
            common::Image in_img;
            sensor_msgs::Image out_img;
            // unwrap stamped img msg to img
            msgs::Set(in_img, _msg->image());
            //std::cerr << "Got px fmt: " << common::PixelFormatNames[in_img.GetPixelFormat()] << std::endl;


            int bufsize = in_img.GetWidth() * in_img.GetHeight() * in_img.GetBPP() / 8;
            //std::cerr << "bufsize " << bufsize << std::endl;
            unsigned int outsize = 0; // bytes
            // GetData expects a ptr to a char*
            // Alloca will free after function scope
            unsigned char** data_ptr = (unsigned char**) alloca(bufsize);

            out_img.header.stamp = ros::Time().now();
            // record IMU data for image reconstruction
            out_img.height = in_img.GetHeight();
            out_img.width = in_img.GetWidth();
            out_img.step = in_img.GetBPP() * in_img.GetWidth();
            out_img.encoding = common::PixelFormatNames[in_img.GetPixelFormat()];
            in_img.GetData(data_ptr, outsize);
            //std::vector <unsigned char> v(*data, *data + sizeof(*data) / sizeof(*data[0]));
            //std::cout << "data " << data_ptr << std::endl;
            //out_img.data = v;
            for (int i = 0; i < outsize / sizeof (char*); ++i) {
                // deref pointer to 1d char array
                out_img.data.push_back((*data_ptr)[i]);
            }
            //std::cerr << "got data of size " << outsize << std::endl;
            //std::cerr << "actually of size " << out_img.data.size() << std::endl;

            this->cam1_pub.publish(out_img);
            //this->WriteIMU(out_img.header.stamp);
        }

        void WriteIMU(ros::Time t) {
            sensor_msgs::Imu imu_msg;
            math::Pose pose = this->model->GetWorldPose();
            math::Vector3 acc = this->model->GetWorldLinearAccel();
            math::Vector3 vel = this->model->GetWorldAngularVel();

            imu_msg.header.stamp = t;

            imu_msg.orientation.w = pose.rot.w;
            imu_msg.orientation.x = pose.rot.x;
            imu_msg.orientation.y = pose.rot.y;
            imu_msg.orientation.z = pose.rot.z;

            imu_msg.angular_velocity.x = vel.x;
            imu_msg.angular_velocity.y = vel.y;
            imu_msg.angular_velocity.z = vel.z;

            imu_msg.linear_acceleration.x = acc.x;
            imu_msg.linear_acceleration.y = acc.y;
            imu_msg.linear_acceleration.z = acc.z;



            this->imu_pub.publish(imu_msg);


        }

        void WriteCloud(ConstLaserScanStampedPtr &_msg) {
            sensor_msgs::PointCloud cloud_msg;
            // Send out a IMU update at the same time
            ros::Time time = ros::Time(_msg->time().sec(), _msg->time().nsec());
            // IMU
            this->WriteIMU(time);
            // Cloud
            cloud_msg.header.stamp = time;
            int slices = _msg->scan().count();
            int vslices = _msg->scan().vertical_count();
            double theta_step = _msg->scan().angle_step();
            double phi_step = _msg->scan().vertical_angle_step();

            for (int vslice = 0; vslice < vslices; ++vslice) {
                for (int slice = 0; slice < slices; ++slice) {
                    geometry_msgs::Point32 p;
                    double theta = slice * theta_step;
                    double phi = vslice * phi_step;
                    // index the jth vslice row and the ith slice column
                    double r = _msg->scan().ranges().Get(vslice * _msg->scan().count() + slice);
                    p.x = r * cos(theta) * cos(phi); //r * sin(phi) * cos(theta);
                    p.y = r * sin(theta) * cos(phi); //r * sin(phi) * sin(theta);
                    p.z = r * sin(phi); //r * cos(phiOnRosMsg);
                    cloud_msg.points.push_back(p);
                }
            }
            this->cloud_pub.publish(cloud_msg);

        }

        void WriteStampedPos(const boost::shared_ptr<physics::ModelState> state) {

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
            /*this->scan_sub = this->node->Subscribe(scan_topic,
                    &SphereXPlugin::WriteScan, this);*/
            this->cloud_sub = this->node->Subscribe(scan_topic,
                    &SphereXPlugin::WriteCloud, this);
            //this->scan_pub = this->ros_node.advertise<sensor_msgs::LaserScan>(output_scan_topic, 1000);
            this->cloud_pub = this->ros_node.advertise<sensor_msgs::PointCloud>(cloud_pub_topic, 10000);
        }

        void SetupIMU() {
            this->imu = sensors::get_sensor("imu");
            this->imu->SetActive(true);
            this->imu_pub = this->ros_node.advertise<sensor_msgs::Imu>(imu_pub_topic, 10000);
        }

        void SetupCamera() {
            this->cam1 = sensors::get_sensor("cam1");
            this->cam1->SetActive(true);
            this->cam1_sub = this->node->Subscribe(cam1_sub_topic,
                    &SphereXPlugin::WriteCam1, this);
            this->cam1_pub = this->ros_node.advertise<sensor_msgs::Image>(cam1_pub_topic, 10000);
        }

        void SetupThruster() {
            this->thrust_vec = math::Vector3(0.0, 0.0, 0.0);
            this->thruster_cmd_sub = this->ros_node.subscribe(
                    this->thruster_cmd_sub_topic,
                    10000,
                    &SphereXPlugin::ApplyImpulse,
                    this
                    );

        }

        // rot stuff
        double full_rot = 2 * 3.1415;
        int slices = 64;
        double v_angular_res = full_rot / slices;

        transport::NodePtr node;
        ros::NodeHandle ros_node;


        // lidar
        sensors::SensorPtr lidar_sensor;
        transport::SubscriberPtr lidar_cmd_sub;
        //transport::SubscriberPtr scan_sub;
        //ros::Publisher scan_pub;
        ros::Publisher cloud_pub;
        transport::SubscriberPtr cloud_sub;
        std::string scan_topic = "~/SphereX/SphereX/SphereXMid/sensor/scan";
        std::string cloud_pub_topic = "raw_pointcloud_stream";



        // camera
        transport::SubscriberPtr cam1_sub;
        sensors::SensorPtr cam1;
        ros::Publisher cam1_pub;
        //std::string cam1_sub_topic = "~/SphereX/SphereX/SphereXMid/cam1/image";
        std::string cam1_sub_topic = "/gazebo/default/SphereX/SphereX/SphereXMid/cam1/image";
        std::string cam1_pub_topic = "cam1_stream";


        // movement
        physics::ModelPtr model;
        sdf::ElementPtr sdf;

        // imu
        sensors::SensorPtr imu;
        ros::Publisher imu_pub;
        std::string imu_pub_topic = "imu_stream";

        // rot
        physics::JointPtr joint;
        common::PID pid;
        transport::SubscriberPtr vel_sub;
        std::string rot_cmd_topic = "~/SphereX/vel_cmd";


        // thrust
        ros::Subscriber thruster_cmd_sub;
        math::Vector3 thrust_vec;
        physics::LinkPtr thrust_link;
        private: event::ConnectionPtr updateConnection;
        // CANNOT HAVE ~/ prefix or gazebo will overwrite it
        std::string thruster_cmd_sub_topic = "thruster_cmd";



        //std::string output_scan_topic = "raw_lidar_stream";

    private:
        ros::CallbackQueue rosQueue;
        std::thread rosQueueThread;


    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(SphereXPlugin)
}
#endif
