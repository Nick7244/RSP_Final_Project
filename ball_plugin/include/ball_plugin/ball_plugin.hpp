#include <gazebo_plugins/gazebo_ros_utils.h>

namespace gazebo {

    class BallPlugin : public ModelPlugin {

        public:
            BallPlugin();
            ~BallPlugin();
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            void UpdateChild();

        private:
            physics::WorldPtr world_ptr_;
            GazeboRosPtr gazebo_ros_;
            event::ConnectionPtr update_connetion_;
            std::string ball_name;
            std::string towerA_name_;
            std::string towerB_name_;
            ros::Publisher pub;
            
  };

}