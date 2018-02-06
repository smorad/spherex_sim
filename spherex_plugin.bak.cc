#ifndef __SPHEREX_PLUGIN_HH_
#define __SPHEREX_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
    class SphereXPlugin : public ModelPlugin
    {
        public: SphereXPlugin() {}

        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
            std::cerr << "\nThe SphereX plugin is attached to the model" << 
                _model->GetName() << "]\n";
        	//Safety check
            if (_model->GetJointCount() == 0){
            	std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
                return;
            }



			// Store the model pointer for convenience.
			this->model = _model;

			// Get the first joint. We are making an assumption about the model
			// having one joint that is the rotational joint.
			this->joint = _model->GetJoints()[0];

			// Setup a P-controller, with a gain of 0.1.
			this->pid = common::PID(0.1, 0, 0);

			// Apply the P-controller to the joint.
			this->model->GetJointController()->SetVelocityPID(
				this->joint->GetScopedName(), this->pid);

            double velocity = 0;
            if (_sdf->HasElement("velocity")){
                velocity = _sdf->Get<double>("velocity");
            }

            this->SetVelocity(velocity);

            // Instantiate producer and consumer
            this->node = transport::NodePtr(new transport::Node());
            this->node->Init(this->model->GetWorld()->GetName());
            std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";
            this->sub = this->node->Subscribe(topicName, &SphereXPlugin::OnMsg, this);
		}

        public: void SetVelocity(const double &_vel){
            this->model->GetJointController()->SetVelocityTarget(
                this->joint->GetScopedName(), _vel);
        }
                

        private: void OnMsg(ConstVector3dPtr &_msg){
            this->SetVelocity(_msg->x());
        }
	  /// \brief Pointer to the model.
	  private: physics::ModelPtr model;

	  /// \brief Pointer to the joint.
	  private: physics::JointPtr joint;

	  /// \brief A PID controller for the joint.
	  private: common::PID pid;

        private: transport::NodePtr node;
        private: transport::SubscriberPtr sub;
	};
   GZ_REGISTER_MODEL_PLUGIN(SphereXPlugin)
}
#endif
