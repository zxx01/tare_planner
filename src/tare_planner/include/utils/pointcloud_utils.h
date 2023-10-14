//
// Created by caochao on 06/03/20.
//
#pragma once

// PCL
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <utils/misc_utils.h>

namespace pointcloud_utils_ns
{
  class VerticalSurfaceExtractor;
  template <typename PCLPointType>
  class PointCloudDownsizer;
  template <typename PCLPointType>
  struct PCLCloud;
} // namespace pointcloud_utils_ns

class pointcloud_utils_ns::VerticalSurfaceExtractor
{
private:
  double kRadiusThreshold;
  double kZDiffMax;
  double kZDiffMin;
  int kNeighborThreshold;
  pcl::PointCloud<pcl::PointXYZI>::Ptr extractor_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr extractor_kdtree_;

public:
  explicit VerticalSurfaceExtractor();
  ~VerticalSurfaceExtractor() = default;
  void SetRadiusThreshold(double radius_threshold)
  {
    kRadiusThreshold = radius_threshold;
  }
  void SetZDiffMax(double z_diff_max)
  {
    kZDiffMax = z_diff_max;
  }
  void SetZDiffMin(double z_diff_min)
  {
    kZDiffMin = z_diff_min;
  }
  void SetNeighborThreshold(int neighbor_threshold)
  {
    kNeighborThreshold = neighbor_threshold;
  }

  template <class PCLPointType>
  void ExtractVerticalSurface(typename pcl::PointCloud<PCLPointType>::Ptr &cloud, double z_max = DBL_MAX,
                              double z_min = -DBL_MAX)
  {
    if (cloud->points.empty())
    {
      return;
    }
    pcl::copyPointCloud(*cloud, *extractor_cloud_);
    for (auto &point : extractor_cloud_->points)
    {
      point.intensity = point.z;
      point.z = 0.0;
    }
    extractor_kdtree_->setInputCloud(extractor_cloud_);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    std::vector<int> neighbor_indices;
    std::vector<float> neighbor_sqdist;
    for (int i = 0; i < extractor_cloud_->points.size(); i++)
    {
      pcl::PointXYZI point = extractor_cloud_->points[i];
      if (point.intensity > z_max || point.intensity < z_min)
        continue;
      extractor_kdtree_->radiusSearch(point, kRadiusThreshold, neighbor_indices, neighbor_sqdist);
      bool is_vertical = false;
      int neighbor_count = 0;
      for (const auto &idx : neighbor_indices)
      {
        double z_diff = std::abs(point.intensity - extractor_cloud_->points[idx].intensity);
        if (z_diff > kZDiffMin && z_diff < kZDiffMax)
        {
          neighbor_count++;
          if (neighbor_count >= kNeighborThreshold)
          {
            is_vertical = true;
            break;
          }
        }
      }
      if (is_vertical)
      {
        inliers->indices.push_back(i);
      }
    }
    pcl::ExtractIndices<PCLPointType> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud);
  }

  /**
   * @brief 给定输入点云，提取竖直面点云
   *
   * @tparam InputPCLPointType
   * @tparam OutputPCLPointType
   * @param cloud_in 输入点云
   * @param cloud_out 输出的竖直面点云
   * @param z_max 输出点云上界
   * @param z_min 输出点云下界
   */
  template <class InputPCLPointType, class OutputPCLPointType>
  void ExtractVerticalSurface(typename pcl::PointCloud<InputPCLPointType>::Ptr &cloud_in,
                              typename pcl::PointCloud<OutputPCLPointType>::Ptr &cloud_out, double z_max = DBL_MAX,
                              double z_min = -DBL_MAX)
  { // 检查输入点云是否为空
    if (cloud_in->points.empty())
    {
      return;
    }

    // 复制输入点云数据到局部变量`extractor_cloud_`
    pcl::copyPointCloud(*cloud_in, *extractor_cloud_);

    // 将点云中每个点的 intensity 属性设置为原始点的 z 坐标值，并将 z 坐标值置为 0.0
    // 目的是把点投影到x-y平面上
    for (auto &point : extractor_cloud_->points)
    {
      point.intensity = point.z;
      point.z = 0.0;
    }

    // 使用 Kd树 构建点云索引
    extractor_kdtree_->setInputCloud(extractor_cloud_);

    // 清空输出点云
    cloud_out->clear();

    // 创建存储邻域点索引和距离的向量
    std::vector<int> neighbor_indices;
    std::vector<float> neighbor_sqdist;

    // 遍历提取点云中的每个点
    for (int i = 0; i < extractor_cloud_->points.size(); i++)
    {
      pcl::PointXYZI point = extractor_cloud_->points[i];

      // 检查点的 intensity 属性，即原始点高度是否在指定的范围内
      if (point.intensity > z_max || point.intensity < z_min)
        continue;

      // 在指定半径kRadiusThreshold内搜索邻域点
      extractor_kdtree_->radiusSearch(point, kRadiusThreshold, neighbor_indices, neighbor_sqdist);
      bool is_vertical = false;
      int neighbor_count = 0;

      // 遍历邻域点，检查其 z 坐标值差异是否在预定范围内
      for (const auto &idx : neighbor_indices)
      {
        double z_diff = std::abs(point.intensity - extractor_cloud_->points[idx].intensity);
        // 找出z方向上有差异的点，而不是基本在同一平面上的点
        if (z_diff > kZDiffMin && z_diff < kZDiffMax)
        {
          neighbor_count++;
          if (neighbor_count >= kNeighborThreshold)
          {
            is_vertical = true;
            break;
          }
        }
      }

      // 如果邻域点满足条件，则将当前点添加到输出点云中
      if (is_vertical)
      {
        OutputPCLPointType point_out;
        point_out.x = cloud_in->points[i].x;
        point_out.y = cloud_in->points[i].y;
        point_out.z = cloud_in->points[i].z;
        cloud_out->points.push_back(point_out);
      }
    }
  }
};

template <typename PCLPointType>
class pointcloud_utils_ns::PointCloudDownsizer
{
private:
  pcl::VoxelGrid<PCLPointType> pointcloud_downsize_filter_;

public:
  explicit PointCloudDownsizer()
  {
  }
  ~PointCloudDownsizer() = default;
  void Downsize(typename pcl::PointCloud<PCLPointType>::Ptr &cloud, double leaf_size_x, double leaf_size_y,
                double leaf_size_z)
  {
    if (cloud->points.empty())
    {
      return;
    }
    pointcloud_downsize_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
    pointcloud_downsize_filter_.setInputCloud(cloud);
    pointcloud_downsize_filter_.filter(*cloud);
  }
};

/**
 * @brief 一个模板结构PCLCloud，用于管理和发布PCL（Point Cloud Library）点云数据。这个结构包含了以下成员：
 *          pub_cloud_topic_: 用于发布点云数据的ROS话题名称。
 *          frame_id_: 点云数据的坐标系（frame_id）。
 *          cloud_: 一个指向pcl::PointCloud的智能指针，用于存储PCL点云数据。
 *          cloud_pub_: 用于发布点云数据的ROS Publisher对象。
 * @tparam PCLPointType
 */
template <typename PCLPointType>
struct pointcloud_utils_ns::PCLCloud
{
  std::string pub_cloud_topic_;
  std::string frame_id_;
  typename pcl::PointCloud<PCLPointType>::Ptr cloud_;
  ros::Publisher cloud_pub_;
  PCLCloud(ros::NodeHandle *nh, std::string pub_cloud_topic, std::string frame_id)
      : pub_cloud_topic_(pub_cloud_topic), frame_id_(frame_id)
  {
    cloud_ = typename pcl::PointCloud<PCLPointType>::Ptr(new pcl::PointCloud<PCLPointType>);
    cloud_pub_ = nh->advertise<sensor_msgs::PointCloud2>(pub_cloud_topic_, 2);
  }
  PCLCloud(ros::NodeHandle &nh, std::string pub_cloud_topic, std::string frame_id)
      : pub_cloud_topic_(pub_cloud_topic), frame_id_(frame_id)
  {
    cloud_ = typename pcl::PointCloud<PCLPointType>::Ptr(new pcl::PointCloud<PCLPointType>);
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(pub_cloud_topic_, 2);
  }
  ~PCLCloud() = default;
  void Publish()
  {
    misc_utils_ns::PublishCloud<pcl::PointCloud<PCLPointType>>(cloud_pub_, *cloud_, frame_id_);
  }
  typedef std::shared_ptr<PCLCloud<PCLPointType>> Ptr;
};
