#include <ros/ros.h>
namespace grasp_objects{
class GraspObjects{
    public:
    
    explicit GraspObjects(ros::NodeHandle nh);

    ~GraspObjects();

    private:
    //! ROS node handle.
    ros::NodeHandle nodeHandle_;

};
}