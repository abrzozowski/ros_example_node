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
    q.setRPY(0, 0, pose->pose.pose.orientation.z);
    transform.setRotation(q);
    geometry_msgs::PoseStamped start;
    start.header.stamp = pose->header.stamp;
    start.header.frame_id = pose->header.frame_id;
    start.pose.position = pose->pose.pose.position;
    start.pose.orientation = pose->pose.pose.orientation;
}

void broadcastTF(const tf::Transform& transform)
{
    ros::NodeHandle n;
    ros::Rate rate(5.0);

    while (n.ok())
    {
        static tf::TransformBroadcaster br;
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
        rate.sleep();
    }
}
