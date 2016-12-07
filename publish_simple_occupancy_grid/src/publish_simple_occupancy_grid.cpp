#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_simple_occupancy_grid");

    ros::NodeHandle n;
    ros::Publisher occupancy_pub = n.advertise<nav_msgs::OccupancyGrid>("simple_occupancy_grid", 1);

    int width = 10;
    int height = 102;
    double resolution = 0.1;
    nav_msgs::OccupancyGrid grid;

    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = width;
    grid.info.height = height;
    grid.data.resize(width * height);

    for (unsigned int i = 0; i < grid.data.size(); i++)
    {
        grid.data[i] = (i / width) - 1;
    }

    ros::Rate r(1.0);
    while (n.ok())
    {
        occupancy_pub.publish(grid);
        r.sleep();
    }
}
