#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_simple_occupancy_grid");

    ros::NodeHandle n;
    ros::Publisher occupancy_pub = n.advertise<nav_msgs::OccupancyGrid>("simple_occupancy_grid", 1);

    int width = 10, height = 102;
    double resolution = 0.1;
    nav_msgs::OccupancyGrid grid;

    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = width;
    grid.info.height = height;
    grid.data.resize(width * height);

    int value = -1;
    for (unsigned int i = 0; i < grid.data.size(); i++)
    {
        grid.data[i] = value;

        if ((i + 1) % width == 0 && i > 0)
            value++;
    }

    ros::Rate r(1.0);
    while (n.ok())
    {
        occupancy_pub.publish(grid);
        r.sleep();
    }
}