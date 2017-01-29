#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <boost/ref.hpp>
#include <boost/thread.hpp>

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose, tf::Transform& transform);
void broadcastTF(const tf::Transform& transform);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_change_initialpose_rviz");
    ros::NodeHandle node;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);

    ros::Subscriber initial_pose_sub = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1,
                                    boost::bind(initialPoseCallback, _1, boost::ref(transform)));
    boost::thread broadcaster_thread(boost::bind(broadcastTF, boost::cref(transform)));

    ros::spin();

    broadcaster_thread.join();

    return 0;
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose, tf::Transform& transform)
{
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(pose->pose.pose.position.x, pose->pose.pose.position.y, 0.0));
    q.setW(pose->pose.pose.orientation.w);
    q.setX(pose->pose.pose.orientation.x);
    q.setY(pose->pose.pose.orientation.y);
    q.setZ(pose->pose.pose.orientation.z);

    transform.setRotation(q);
}

void broadcastTF(const tf::Transform& transform)
{
    ros::NodeHandle n;
    ros::Rate rate(50.0);

    while (n.ok())
    {
        static tf::TransformBroadcaster br;
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
        rate.sleep();
    }
}
