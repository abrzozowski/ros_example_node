#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

const int MAX_MAP_SIZE = 40;

void drawCostmap2d(costmap_2d::Costmap2D *costmap)
{
    int width = costmap->getSizeInCellsX();
    int height = costmap->getSizeInCellsY();
    unsigned char *char_map = costmap->getCharMap();

    if (width > MAX_MAP_SIZE || height > MAX_MAP_SIZE)
    {
        std::cout << "Sorry, this costmap is too large to draw." << std::endl;
        return;
    }

    std::cout << "Data:" << std::endl;

    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j) {
            std::cout <<  std::setw(3) << (int) char_map[i * width + j] << " "; /*costmap->getCost(j, i)*/
        }
        std::cout << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "access_costmap");

    tf::TransformListener tf(ros::Duration(10));
    costmap_2d::Costmap2DROS costmap_ros("costmap", tf);

    ros::NodeHandle n;

    ros::Rate rate(5);
    while (n.ok())
    {
        costmap_2d::Costmap2D *costmap = costmap_ros.getCostmap();

        std::cout << "=== Costmap2DROS ===" << std::endl;
        std::cout << "Base frame: " << costmap_ros.getBaseFrameID() << std::endl;
        std::cout << "Global frame ID: " << costmap_ros.getGlobalFrameID() << std::endl;
        std::cout << "Is current: " << costmap_ros.isCurrent() << std::endl;
        std::cout << "=== Costmap2D ===" << std::endl;
        std::cout << "Default value: " << costmap->getDefaultValue() << std::endl;
        std::cout << "Origin X: " << costmap->getOriginX() << std::endl;
        std::cout << "Origin Y: " << costmap->getOriginY() << std::endl;
        std::cout << "Resolution: " << costmap->getResolution() << std::endl;
        std::cout << "Size in cells X: " << costmap->getSizeInCellsX() << std::endl;
        std::cout << "Size in cells Y: " << costmap->getSizeInCellsY() << std::endl;
        std::cout << "Size in meters X: " << costmap->getSizeInMetersX() << std::endl;
        std::cout << "Size in meters Y: " << costmap->getSizeInMetersY() << std::endl;

        drawCostmap2d(costmap);

        std::cout << std::endl;

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
