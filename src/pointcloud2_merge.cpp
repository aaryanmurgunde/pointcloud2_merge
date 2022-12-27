#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>


class Pointcloud2Merge
{
public:
  Pointcloud2Merge();

private:
  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef sensor_msgs::PointCloud2 PointCloudMsgT;
  typedef message_filters::sync_policies::ApproximateTime<PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                          PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                          PointCloudMsgT, PointCloudMsgT>
      SyncPolicyT;

  ros::NodeHandle node_handle_, private_node_handle_;
  message_filters::Subscriber<PointCloudMsgT> *cloud_subscribers_[8];
  message_filters::Synchronizer<SyncPolicyT> *cloud_synchronizer_;
  ros::Subscriber config_subscriber_;
  ros::Publisher cloud_publisher_;
  tf::TransformListener tf_listener_;

  size_t input_topics_size_;
  std::string input_topics_;
  std::string output_frame_id_;
  double resolution_;

  void pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                           const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                           const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                           const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8);
};

Pointcloud2Merge::Pointcloud2Merge() : node_handle_(), private_node_handle_("~"), tf_listener_()
{
  private_node_handle_.param("input_topics", input_topics_, std::string("[/points_alpha, /points_beta]"));
  private_node_handle_.param("output_frame_id", output_frame_id_, std::string("velodyne"));
  private_node_handle_.param("resolution", resolution_, 0.025);

  YAML::Node topics = YAML::Load(input_topics_);
  input_topics_size_ = topics.size();
  if (input_topics_size_ < 2 || 8 < input_topics_size_)
  {
    ROS_ERROR("The size of input_topics must be between 2 and 8");
    ros::shutdown();
  }
  for (size_t i = 0; i < 8; ++i)
  {
    if (i < input_topics_size_)
    {
      cloud_subscribers_[i] =
          new message_filters::Subscriber<PointCloudMsgT>(node_handle_, topics[i].as<std::string>(), 1);
    }
    else
    {
      cloud_subscribers_[i] =
          new message_filters::Subscriber<PointCloudMsgT>(node_handle_, topics[0].as<std::string>(), 1);
    }
  }
  cloud_synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(
      SyncPolicyT(10), *cloud_subscribers_[0], *cloud_subscribers_[1], *cloud_subscribers_[2], *cloud_subscribers_[3],
      *cloud_subscribers_[4], *cloud_subscribers_[5], *cloud_subscribers_[6], *cloud_subscribers_[7]);
  cloud_synchronizer_->registerCallback(
      boost::bind(&Pointcloud2Merge::pointcloud_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));
  cloud_publisher_ = node_handle_.advertise<PointCloudMsgT>("/points_concat", 1);
}

void Pointcloud2Merge::pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                                             const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                                             const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                                             const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8)
{
  assert(2 <= input_topics_size_ && input_topics_size_ <= 8);

  ros::WallTime start2 = ros::WallTime::now();

  PointCloudMsgT::ConstPtr msgs[8] = { msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8 };
  PointCloudT::Ptr cloud_concatenated(new PointCloudT);

  // transform points
  try
  {
    for (size_t i = 0; i < input_topics_size_; ++i)
    {
      pcl::PCLPointCloud2Ptr cloud1(new pcl::PCLPointCloud2);
      PointCloudT::Ptr cloud2(new PointCloudT);

      pcl_conversions::toPCL(*msgs[i], *cloud1);

      pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
      filter.setInputCloud (cloud1);
      filter.setLeafSize (resolution_, resolution_, resolution_);
      filter.filter (*cloud1);

      pcl::fromPCLPointCloud2(*cloud1, *cloud2);

      tf_listener_.waitForTransform(output_frame_id_, msgs[i]->header.frame_id, ros::Time(0), ros::Duration(1.0));
      pcl_ros::transformPointCloud(output_frame_id_, ros::Time(0), *cloud2, msgs[i]->header.frame_id, *cloud2, tf_listener_);

      *cloud_concatenated += *cloud2;
    }
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // Downsample to reduce points
  pcl::VoxelGrid<PointT> filter;
  filter.setInputCloud (cloud_concatenated);
  filter.setLeafSize (resolution_, resolution_, resolution_);
  filter.filter (*cloud_concatenated);

  // publsh points
  cloud_concatenated->header = pcl_conversions::toPCL(msgs[0]->header);
  cloud_concatenated->header.frame_id = output_frame_id_;
  cloud_publisher_.publish(cloud_concatenated);

  ROS_DEBUG("Processed PointCloud in %lf ms", (ros::WallTime::now() - start2).toSec() * 1000.0);
  ROS_DEBUG("________________________________________________");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud2_merge");
  Pointcloud2Merge node;
  ros::spin();
  return 0;
}