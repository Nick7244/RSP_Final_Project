#include <ball_plugin/ball_plugin.hpp>
#include <ur5_kendama_msgs/ball_position.h>

namespace gazebo {

    // Empty constructor and desctuctor
    BallPlugin::BallPlugin(){}
    BallPlugin::~BallPlugin(){}
    
    // Load the plugin
    void BallPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        world_ptr_ = _parent->GetWorld();
        gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "BallPlugin"));
        
        // Get parameters for the tower names
        gazebo_ros_->getParameter<std::string>(ball_name, "ball", "ball");
        
        // Create the triangulation publisher
        pub = gazebo_ros_->node()->advertise<ur5_kendama_msgs::ball_position>("ball_position",10);
        //gazebo_ros_->getParameter<std::string>(towerA_name_, "towerA", "towerA");
        //gazebo_ros_->getParameter<std::string>(towerB_name_, "towerB", "towerB");
        gazebo_ros_->getParameter<std::string>(ball_name, "ball", "ball");

        // Setup the plugin callback
        update_connetion_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&BallPlugin::UpdateChild, this));
        
    }

    // Callback function for plugin
    void BallPlugin::UpdateChild()
    {
        // Get gazebo models
        physics::ModelPtr ball_ptr = world_ptr_->GetModel(ball_name);
        
        // Get model positions
        math::Vector3 ball_pos = ball_ptr->GetWorldPose().pos;

        ur5_kendama_msgs::ball_position myMessage;

        myMessage.x.data = ball_pos.x;
        myMessage.y.data = ball_pos.y;
        myMessage.z.data = ball_pos.z;

        pub.publish(myMessage);
    }

    GZ_REGISTER_MODEL_PLUGIN ( BallPlugin )
    
}