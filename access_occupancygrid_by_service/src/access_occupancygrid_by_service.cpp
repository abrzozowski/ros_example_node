#include <ros/ros.h>
#include <nav_msgs/GetMap.h>

const int MAX_MAP_SIZE = 40;

void requestMap(ros::NodeHandle &n);
void readMap(const nav_msgs::OccupancyGrid& occupancygrid);
void drawOccupancyGrid(const nav_msgs::OccupancyGrid& occupancygrid);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "access_occupancygrid_by_service");

    ros::NodeHandle n;

    ros::Rate rate(5);
    while (n.ok())
    {
        requestMap(n);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

void requestMap(ros::NodeHandle &n)
{
    nav_msgs::GetMap::Request req;
    nav_msgs::GetMap::Response res;

    while (!ros::service::waitForService("static_map", ros::Duration(3.0)))
    {
         std::cout << "Waiting for service static_map" << std::endl;
    }

    ros::ServiceClient mapClient = n.serviceClient<nav_msgs::GetMap>("static_map");

    if (mapClient.call(req, res))
    {
        readMap(res.map);
    }
    else
    {
        std::cout << "Failed to call static_map service" << std::endl;
    }
}

void readMap(const nav_msgs::OccupancyGrid& occupancygrid)
{
    std::cout << "=== OccupancyGrid ===" << std::endl;
    std::cout << "= Header =" << std::endl;
    std::cout << "Seq: " << occupancygrid.header.seq << std::endl;
    std::cout << "Time Stamp: " << occupancygrid.header.stamp << std::endl;
    std::cout << "Frame ID: " << occupancygrid.header.frame_id << std::endl;
    std::cout << "= Info =" << std::endl;
    std::cout << "Load Time: " << occupancygrid.info.map_load_time << std::endl;
    std::cout << "Resolution: " << occupancygrid.info.resolution << std::endl;
    std::cout << "Width: " << occupancygrid.info.width << std::endl;
    std::cout << "Height: " << occupancygrid.info.height << std::endl;
    std::cout << "Origin \n\tposition:"
        << " x " << occupancygrid.info.origin.position.x
        << " y " << occupancygrid.info.origin.position.y
        << " z " << occupancygrid.info.origin.position.z
        << std::endl;
    std::cout << "\torientation:"
        << " x " << occupancygrid.info.origin.orientation.x
        << " y " << occupancygrid.info.origin.orientation.y
        << " z " << occupancygrid.info.origin.orientation.z
        << " w " << occupancygrid.info.origin.orientation.w
        << std::endl;

    drawOccupancyGrid(occupancygrid);

    std::cout << std::endl;
}

void drawOccupancyGrid(const nav_msgs::OccupancyGrid& occupancygrid)
{
    int width = occupancygrid.info.width;
    int height = occupancygrid.info.height;

    if (width > MAX_MAP_SIZE || height > MAX_MAP_SIZE)
    {
        std::cout << "Sorry, this occupancy grid is too large to draw." << std::endl;
        return;
    }

    std::cout << "Data:" << std::endl;

    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j) {
            std::cout <<  std::setw(3) << (int) occupancygrid.data[i * width + j] << " ";
        }
        std::cout << std::endl;
    }
}
