#ifndef __SPHEREX_PLUGIN_HH_
#define __SPHEREX_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

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

			// Set the joint's target velocity. This target velocity is just
			// for demonstration purposes.
			this->model->GetJointController()->SetVelocityTarget(
				this->joint->GetScopedName(), 1.0);
		}
	  /// \brief Pointer to the model.
	  private: physics::ModelPtr model;

	  /// \brief Pointer to the joint.
	  private: physics::JointPtr joint;

	  /// \brief A PID controller for the joint.
	  private: common::PID pid;
	};
   GZ_REGISTER_MODEL_PLUGIN(SphereXPlugin)
}
#endif
