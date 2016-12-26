#include "obstacle_detection_node.h"

/* Some of the concepts for this node were copied from:
 * https://github.com/tysik/obstacle_detector/blob/master/src/obstacle_detector.cpp
 * */

typedef zenith_obstacle_detector::Obstacles Obstacles;
using zenith_obstacle_detector::SegmentObstacle;
using sensor_msgs::LaserScan;
typedef std::list<geometry_msgs::Point> PointList;
typedef std::list<zenith_obstacle_detector::SegmentObstacle> SegmentList;

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_detector");
    ObstacleDetection od;
    return 0;
}

/*--------------------------------------------------------------------
 * ObstacleDetection()
 * Constructor.
 *------------------------------------------------------------------*/

ObstacleDetection::ObstacleDetection()
{
    // Setup ROS subscription object
    scan_sub_ = nh_.subscribe("/mybot/front_laser/scan", 10, &ObstacleDetection::laserCallback, this);   
    // Setup ROS publihs object
    obstacles_pub_ = nh_.advertise<Obstacles>("obstacles", 10);
    // Run ObstacleDetection object
    ros::spin();
} // end ObstacleDetection()

/*--------------------------------------------------------------------
 * ~ObstacleDetection()
 * Destructor.
 *------------------------------------------------------------------*/

ObstacleDetection::~ObstacleDetection()
{
} // end ~ObstacleDetection()

/*--------------------------------------------------------------------
 * publishMessage()
 * Publish the message.
 *------------------------------------------------------------------*/

void ObstacleDetection::publishObstacles()
{
    Obstacles obstacles;
    obstacles.header.stamp = ros::Time::now();
    for (SegmentList::const_iterator ci = segments_.begin(); ci != segments_.end(); ++ci)
    {
        SegmentObstacle segment;
        segment.first_point.x = ci->first_point.x;
        segment.first_point.y = ci->first_point.y;
        segment.last_point.x = ci->last_point.x;
        segment.last_point.y = ci->last_point.y;
        obstacles.segments.push_back(segment);
    }
    obstacles_pub_.publish(obstacles);
} // end publishObstacles()

/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/

void ObstacleDetection::laserCallback(const LaserScan::ConstPtr& scan)
{
    //scan->ranges[] are laser readings
    double phi = scan->angle_min;
    //for (const float r : scan->ranges) {  // c++ 11 style
    int scansize = sizeof(scan->ranges)/sizeof(scan->ranges.front());
    bool start_seg = false;
    bool end_seg = true;
    for (int i; i < scansize; i++)
    {
        double r = scan->ranges[i];
        // ROS_INFO should be DEBUG.
        ROS_INFO("phi: %f", phi);
        ROS_INFO("r: %f", r);
        geometry_msgs::Point point;
        // TODO CONVERT TO CART
        point.x = r;
        point.y = phi;
        if (r < scan->range_max)
        {
            
        }
        input_points_.push_back(point);
        phi += scan->angle_increment;
    }
    detectObstacles();
    publishObstacles();
} // end publishCallback()

void ObstacleDetection::detectObstacles()
{
    for (PointList::const_iterator ci = input_points_.begin(); ci != input_points_.end(); ++ci)
    {
        std::cout << ci->x << " " << ci->y << "\n";
        SegmentObstacle segment;
        segment.first_point.x = ci->x;
        segment.first_point.y = ci->y;
        segment.last_point.x = ci->x;
        segment.last_point.y = ci->y;
        segments_.push_back(segment);
    }
}
