#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>


std::string _reference_frame = "laser_link";
std::string _scan_topic = "/scan";
std::string _zone_topic = "/zone";
ros::Publisher safe_pub;
ros::Publisher polygon_sub;

std_msgs::Bool isZone;
int unsafe_counter=0;
int _unsafe_tolerance;
int skip_scan = 1;
int skip_counter = 0;
double _min_x, _max_x, _min_y, _max_y;
double *sin_lut;
double *cos_lut;
bool initial = true;

bool isWithinBox(double dist, double angle)
{

    double x = dist*cos(angle); //Rotated 30 degree
    double y = dist*sin(angle); //Rotated 30 degree
    //ROS_INFO("obstacle detected %f %f %f",cos_lut[0],x,y);
  
    if((x > _min_x) && (x < _max_x) && (y < _max_y) && (y > _min_y)){
        return true;
    }else{
        return false;
    }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    double ref_angle = scan_msg->angle_min;
    double incremental_angle = scan_msg->angle_increment;
    bool is_safe = true;
   
    if(skip_counter == skip_scan)
    {        
        for (unsigned int x=0;x< scan_msg->ranges.size();x++)
        {   
            if(scan_msg->ranges[x] > 0.005)
            {
                if (isWithinBox(scan_msg->ranges[x],ref_angle)){
                    is_safe = false;
                    unsafe_counter++;
                    break;
                }
            }
            ref_angle += incremental_angle;
        }
        if(is_safe)
        {
            unsafe_counter = 0;
        }

        if(unsafe_counter > _unsafe_tolerance)
        {
            isZone.data = true;            
            ROS_WARN_STREAM(_zone_topic << " detected");
        }          
        else
        {
            isZone.data = false;
            //ROS_WARN("no zone etected");
        }
            
        safe_pub.publish(isZone);
        skip_counter = 0;
    }
    skip_counter++;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_safety");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    pn.param("reference_frame", _reference_frame, std::string("laser_link"));
    pn.param("scan_topic", _scan_topic, std::string("/scan"));
    pn.param("zone_topic", _zone_topic, std::string("/zone"));
    pn.param("x_max", _max_x, double(0.3));
    pn.param("x_min", _min_x, double(0.0));
    pn.param("y_max", _max_y, double(0.3));
    pn.param("y_min", _min_y, double(-0.3));
    pn.param("unsafe_tolerance", _unsafe_tolerance, int(3));

    //ROS_INFO("Parameter lower_threshold: %f", _obstacle_thresh);

    ros::Subscriber laserSub = n.subscribe(_scan_topic, 10, scanCallback);
    safe_pub = n.advertise<std_msgs::Bool>(_zone_topic, 1, true);

    ros::spin();
    return 0;
}
