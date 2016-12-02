#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>

const int MAX_MAP_SIZE = 40;

nav_msgs::OccupancyGrid costmap;

void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &cmap)
{
    costmap = *cmap;
}

void drawOccupancyGrid()
{
    int width = costmap.info.width;
    int height = costmap.info.height;

    if (width > MAX_MAP_SIZE || height > MAX_MAP_SIZE)
    {
        std::cout << "Sorry, this occupancy grid is too large to draw." << std::endl;
        return;
    }

    std::cout << "Data:" << std::endl;

    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j) {
            std::cout <<  std::setw(3) << (int) costmap.data[i * width + j] << " ";
        }
        std::cout << std::endl;
    }
}

int main (int argc , char** argv)
{
    ros::init(argc, argv, "access_occupancygrid");

    ros::NodeHandle n;
    ros::Subscriber costmap_subscriber =  n.subscribe<nav_msgs::OccupancyGrid>("map", 1, &costmapCallback);

    ros::Rate rate(5);
    while (n.ok())
    {
        std::cout << "=== OccupancyGrid ===" << std::endl;
        std::cout << "= Header =" << std::endl;
        std::cout << "Seq: " << costmap.header.seq << std::endl;
        std::cout << "Time Stamp: " << costmap.header.stamp << std::endl;
        std::cout << "Frame ID: " << costmap.header.frame_id << std::endl;
        std::cout << "= Info =" << std::endl;
        std::cout << "Load Time: " << costmap.info.map_load_time << std::endl;
        std::cout << "Resolution: " << costmap.info.resolution << std::endl;
        std::cout << "Width: " << costmap.info.width << std::endl;
        std::cout << "Height: " << costmap.info.height << std::endl;
        std::cout << "Origin \n\tposition:"
            << " x " << costmap.info.origin.position.x
            << " y " << costmap.info.origin.position.y
            << " z " << costmap.info.origin.position.z
            << std::endl;
        std::cout << "\torientation:"
            << " x " << costmap.info.origin.orientation.x
            << " y " << costmap.info.origin.orientation.y
            << " z " << costmap.info.origin.orientation.z
            << " w " << costmap.info.origin.orientation.w
            << std::endl;

        drawOccupancyGrid();

        std::cout << std::endl;

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
