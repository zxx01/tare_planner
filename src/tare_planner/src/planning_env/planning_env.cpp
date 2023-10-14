/**
 * @file planning_env.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that manages the world representation using point clouds
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <planning_env/planning_env.h>
#include <viewpoint_manager/viewpoint_manager.h>

namespace planning_env_ns
{
  void PlanningEnvParameters::ReadParameters(ros::NodeHandle &nh)
  {
    kSurfaceCloudDwzLeafSize = misc_utils_ns::getParam<double>(nh, "kSurfaceCloudDwzLeafSize", 0.2);
    kCollisionCloudDwzLeafSize = misc_utils_ns::getParam<double>(nh, "kCollisionCloudDwzLeafSize", 0.2);
    kKeyposeGraphCollisionCheckRadius =
        misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeCollisionCheckRadius", 0.4);
    kKeyposeGraphCollisionCheckPointNumThr =
        misc_utils_ns::getParam<int>(nh, "keypose_graph/kAddEdgeCollisionCheckPointNumThr", 1);

    kKeyposeCloudStackNum = misc_utils_ns::getParam<int>(nh, "kKeyposeCloudStackNum", 5);

    kPointCloudRowNum = misc_utils_ns::getParam<int>(nh, "kPointCloudRowNum", 20);
    kPointCloudColNum = misc_utils_ns::getParam<int>(nh, "kPointCloudColNum", 20);
    kPointCloudLevelNum = misc_utils_ns::getParam<int>(nh, "kPointCloudLevelNum", 10);
    kMaxCellPointNum = misc_utils_ns::getParam<int>(nh, "kMaxCellPointNum", 100000);
    kPointCloudCellSize = misc_utils_ns::getParam<double>(nh, "kPointCloudCellSize", 24.0);
    kPointCloudCellHeight = misc_utils_ns::getParam<double>(nh, "kPointCloudCellHeight", 3.0);
    kPointCloudManagerNeighborCellNum = misc_utils_ns::getParam<int>(nh, "kPointCloudManagerNeighborCellNum", 5);
    kCoverCloudZSqueezeRatio = misc_utils_ns::getParam<double>(nh, "kCoverCloudZSqueezeRatio", 2.0);

    kUseFrontier = misc_utils_ns::getParam<bool>(nh, "kUseFrontier", false);
    kFrontierClusterTolerance = misc_utils_ns::getParam<double>(nh, "kFrontierClusterTolerance", 1.0);
    kFrontierClusterMinSize = misc_utils_ns::getParam<int>(nh, "kFrontierClusterMinSize", 30);

    kUseCoverageBoundaryOnFrontier = misc_utils_ns::getParam<bool>(nh, "kUseCoverageBoundaryOnFrontier", false);
    kUseCoverageBoundaryOnObjectSurface = misc_utils_ns::getParam<bool>(nh, "kUseCoverageBoundaryOnObjectSurface", false);

    int viewpoint_number = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/number_x", 40);
    double viewpoint_resolution = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_x", 1.0);
    double local_planning_horizon_half_size = viewpoint_number * viewpoint_resolution / 2;
    double sensor_range = misc_utils_ns::getParam<double>(nh, "kSensorRange", 15);

    kExtractFrontierRange.x() = local_planning_horizon_half_size + sensor_range * 2;
    kExtractFrontierRange.y() = local_planning_horizon_half_size + sensor_range * 2;
    kExtractFrontierRange.z() = 2;
  }

  PlanningEnv::PlanningEnv(ros::NodeHandle nh, ros::NodeHandle nh_private, std::string world_frame_id)
      : keypose_cloud_count_(0), vertical_surface_extractor_(), vertical_frontier_extractor_(), robot_position_update_(false)
  {
    parameters_.ReadParameters(nh_private);
    keypose_cloud_stack_.resize(parameters_.kKeyposeCloudStackNum);
    for (int i = 0; i < keypose_cloud_stack_.size(); i++)
    {
      keypose_cloud_stack_[i].reset(new pcl::PointCloud<PlannerCloudPointType>());
    }

    vertical_surface_cloud_stack_.resize(parameters_.kKeyposeCloudStackNum);
    for (int i = 0; i < vertical_surface_cloud_stack_.size(); i++)
    {
      vertical_surface_cloud_stack_[i].reset(new pcl::PointCloud<PlannerCloudPointType>());
    }
    keypose_cloud_ =
        std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "keypose_cloud", world_frame_id);
    stacked_cloud_ =
        std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "stacked_cloud", world_frame_id);
    stacked_vertical_surface_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(
        nh, "stacked_vertical_surface_cloud", world_frame_id);

    stacked_vertical_surface_cloud_kdtree_ =
        pcl::KdTreeFLANN<PlannerCloudPointType>::Ptr(new pcl::KdTreeFLANN<PlannerCloudPointType>());
    vertical_surface_cloud_ =
        std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "coverage_cloud", world_frame_id);

    diff_cloud_ =
        std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "diff_cloud", world_frame_id);

    collision_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    terrain_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "terrain_cloud", world_frame_id);

    planner_cloud_ =
        std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "planner_cloud", world_frame_id);
    pointcloud_manager_ = std::make_unique<pointcloud_manager_ns::PointCloudManager>(
        parameters_.kPointCloudRowNum, parameters_.kPointCloudColNum, parameters_.kPointCloudLevelNum,
        parameters_.kMaxCellPointNum, parameters_.kPointCloudCellSize, parameters_.kPointCloudCellHeight,
        parameters_.kPointCloudManagerNeighborCellNum);
    pointcloud_manager_->SetCloudDwzFilterLeafSize() = parameters_.kSurfaceCloudDwzLeafSize;

    rolling_occupancy_grid_ = std::make_unique<rolling_occupancy_grid_ns::RollingOccupancyGrid>(nh_private);

    squeezed_planner_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(
        nh, "squeezed_planner_cloud", world_frame_id);
    squeezed_planner_cloud_kdtree_ =
        pcl::KdTreeFLANN<PlannerCloudPointType>::Ptr(new pcl::KdTreeFLANN<PlannerCloudPointType>());

    uncovered_cloud_ =
        std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "uncovered_cloud", world_frame_id);
    uncovered_frontier_cloud_ =
        std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "uncovered_frontier_cloud", world_frame_id);
    frontier_cloud_ =
        std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "frontier_cloud", world_frame_id);
    filtered_frontier_cloud_ =
        std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "filtered_frontier_cloud", world_frame_id);
    occupied_cloud_ =
        std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "occupied_cloud", world_frame_id);
    free_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "free_cloud", world_frame_id);
    unknown_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "unknown_cloud", world_frame_id);

    rolling_occupancy_grid_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
        nh, "rolling_occupancy_grid_cloud", world_frame_id);

    rolling_frontier_cloud_ =
        std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "rolling_frontier_cloud", world_frame_id);

    rolling_filtered_frontier_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
        nh, "rolling_filtered_frontier_cloud", world_frame_id);

    rolled_in_occupancy_cloud_ =
        std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "rolled_in_occupancy_cloud", world_frame_id);
    rolled_out_occupancy_cloud_ =
        std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "rolled_out_occupancy_cloud", world_frame_id);

    pointcloud_manager_occupancy_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
        nh, "pointcloud_manager_occupancy_cloud_", world_frame_id);

    kdtree_frontier_cloud_ = pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>);
    kdtree_rolling_frontier_cloud_ = pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>);

    // Todo: parameterize
    vertical_surface_extractor_.SetRadiusThreshold(0.2);
    vertical_surface_extractor_.SetZDiffMax(2.0);
    vertical_surface_extractor_.SetZDiffMin(parameters_.kSurfaceCloudDwzLeafSize);
    vertical_frontier_extractor_.SetNeighborThreshold(2);

    Eigen::Vector3d rolling_occupancy_grid_resolution = rolling_occupancy_grid_->GetResolution();
    double vertical_frontier_neighbor_search_radius =
        std::max(rolling_occupancy_grid_resolution.x(), rolling_occupancy_grid_resolution.y());
    vertical_frontier_neighbor_search_radius =
        std::max(vertical_frontier_neighbor_search_radius, rolling_occupancy_grid_resolution.z());
    vertical_frontier_extractor_.SetRadiusThreshold(vertical_frontier_neighbor_search_radius);
    double z_diff_max = vertical_frontier_neighbor_search_radius * 5;
    double z_diff_min = vertical_frontier_neighbor_search_radius;
    vertical_frontier_extractor_.SetZDiffMax(z_diff_max);
    vertical_frontier_extractor_.SetZDiffMin(z_diff_min);
    vertical_frontier_extractor_.SetNeighborThreshold(2);
  }

  /**
   * @brief 将vertical_surface_cloud_stack_的点云数据并入collision_cloud_并进行下采样
   *
   */
  void PlanningEnv::UpdateCollisionCloud()
  {
    collision_cloud_->clear();
    for (int i = 0; i < parameters_.kKeyposeCloudStackNum; i++)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::copyPointCloud<PlannerCloudPointType, pcl::PointXYZI>(*vertical_surface_cloud_stack_[i], *cloud_tmp);
      *(collision_cloud_) += *cloud_tmp;
    }
    collision_cloud_downsizer_.Downsize(collision_cloud_, parameters_.kCollisionCloudDwzLeafSize,
                                        parameters_.kCollisionCloudDwzLeafSize, parameters_.kCollisionCloudDwzLeafSize);
  }

  /**
   * @brief 更新前沿点云数据的过程，包括提取未知或边界区域的Frontier点云、
   *        在启用边界的情况下提取覆盖范围内的点云、提取垂直面点云并对其进行聚类操作，最终将处理后的前沿点云数据发布出去。
   *
   */
  void PlanningEnv::UpdateFrontiers()
  {
    // 检查是否使用Frontier
    if (parameters_.kUseFrontier)
    {
      // 存储上一次机器人位置
      prev_robot_position_ = robot_position_;
      // 获取Frontier点云
      rolling_occupancy_grid_->GetFrontier(frontier_cloud_->cloud_, robot_position_, parameters_.kExtractFrontierRange);

      // 检查Frontier点云是否为空
      if (!frontier_cloud_->cloud_->points.empty())
      {
        // 如果启用了覆盖范围边界（parameters_.kUseCoverageBoundaryOnFrontier），则将Frontier点云数据中位于指定范围内的点云提取出来。
        if (parameters_.kUseCoverageBoundaryOnFrontier)
        {
          GetCoverageCloudWithinBoundary<pcl::PointXYZI>(frontier_cloud_->cloud_);
        }

        // 在frontier点云中提取垂直面，将结果保存到filtered_frontier_cloud_->cloud_中
        vertical_frontier_extractor_.ExtractVerticalSurface<pcl::PointXYZI, pcl::PointXYZI>(
            frontier_cloud_->cloud_, filtered_frontier_cloud_->cloud_);
      }

      // Cluster frontiers
      if (!filtered_frontier_cloud_->cloud_->points.empty())
      {
        // 创建Kd树，用于聚类
        kdtree_frontier_cloud_->setInputCloud(filtered_frontier_cloud_->cloud_);

        // 存储聚类索引
        std::vector<pcl::PointIndices> cluster_indices;

        // 创建Euclidean聚类对象，进行垂直面点云下的聚类
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(parameters_.kFrontierClusterTolerance); // 设置聚类的容差范围
        ec.setMinClusterSize(1);
        ec.setMaxClusterSize(10000);
        ec.setSearchMethod(kdtree_frontier_cloud_); // 设置聚类使用的搜索方法
        ec.setInputCloud(filtered_frontier_cloud_->cloud_);
        ec.extract(cluster_indices);

        // 创建点云索引对象
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        int cluster_count = 0;

        // 遍历聚类索引，对每个聚类进行处理，过滤掉小聚类
        for (int i = 0; i < cluster_indices.size(); i++)
        {
          // 检查聚类大小是否满足最小要求
          if (cluster_indices[i].indices.size() < parameters_.kFrontierClusterMinSize)
          {
            continue;
          }

          // 遍历此聚类簇内的每个点
          for (int j = 0; j < cluster_indices[i].indices.size(); j++)
          {
            int point_ind = cluster_indices[i].indices[j];
            // 将聚类内的点的 intensity 属性设置为聚类计数值
            filtered_frontier_cloud_->cloud_->points[point_ind].intensity = cluster_count;
            // 将点索引添加到inliers中，用于后续提取操作
            inliers->indices.push_back(point_ind);
          }
          cluster_count++;
        }

        // 使用提取器提取指定索引（过滤掉小聚类）的点云，将提取结果保存到filtered_frontier_cloud_->cloud_中
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(filtered_frontier_cloud_->cloud_);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*(filtered_frontier_cloud_->cloud_));
        filtered_frontier_cloud_->Publish();
      }
    }
  }

  void PlanningEnv::UpdateTerrainCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
  {
    if (cloud->points.empty())
    {
      ROS_WARN("Terrain cloud empty");
    }
    else
    {
      terrain_cloud_->cloud_ = cloud;
    }
  }

  bool PlanningEnv::InCollision(double x, double y, double z) const
  {
    if (stacked_cloud_->cloud_->points.empty())
    {
      ROS_WARN("PlanningEnv::InCollision(): collision cloud empty, not checking collision");
      return false;
    }
    PlannerCloudPointType check_point;
    check_point.x = x;
    check_point.y = y;
    check_point.z = z;
    std::vector<int> neighbor_indices;
    std::vector<float> neighbor_sqdist;
    stacked_vertical_surface_cloud_kdtree_->radiusSearch(check_point, parameters_.kKeyposeGraphCollisionCheckRadius,
                                                         neighbor_indices, neighbor_sqdist);
    if (neighbor_indices.size() > parameters_.kKeyposeGraphCollisionCheckPointNumThr)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  /**
   * @brief 更新已覆盖区域，同时也对覆盖区域进行了膨胀操作，确保覆盖的区域更加充分，
   *        被覆盖的区域是通过将点云颜色设置为绿色，即RGB的G通道设置为255而实现。
   *
   * @param robot_viewpoint
   * @param viewpoint_manager
   */
  void PlanningEnv::UpdateCoveredArea(const lidar_model_ns::LiDARModel &robot_viewpoint,
                                      const std::shared_ptr<viewpoint_manager_ns::ViewPointManager> &viewpoint_manager)
  {
    if (planner_cloud_->cloud_->points.empty())
    {
      std::cout << "Planning cloud empty, cannot update covered area" << std::endl;
      return;
    }

    // 获取机器人视点的位置、传感器范围、覆盖遮挡阈值和覆盖膨胀半径
    geometry_msgs::Point robot_position = robot_viewpoint.getPosition();
    double sensor_range = viewpoint_manager->GetSensorRange();
    double coverage_occlusion_thr = viewpoint_manager->GetCoverageOcclusionThr();
    double coverage_dilation_radius = viewpoint_manager->GetCoverageDilationRadius();

    std::vector<int> covered_point_indices;
    double vertical_fov_ratio = 0.3; // bigger fov than viewpoints
    double diff_z_max = sensor_range * vertical_fov_ratio;
    double xy_dist_threshold = 3 * (parameters_.kSurfaceCloudDwzLeafSize / 2) / 0.3;
    double z_diff_threshold = 3 * parameters_.kSurfaceCloudDwzLeafSize;

    int consider_point = 0;
    // 对每一个planner_cloud_中的point, 检查：1. 是否可被当前机器人位置看到，2. 是否被已经visited的视点看到
    for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
    {
      // 如果点的 g 属性（可能代表一个标志或状态）大于 0，将其设置为 255，表示已覆盖，然后继续到下一个点。
      PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
      if (point.g > 0)
      {
        planner_cloud_->cloud_->points[i].g = 255;
        continue;
      }
      consider_point++;

      // 1. 检查机器人视点是否能看到该planner_cloud_点
      // 如果planner_cloud_点的高度在 diff_z_max 内（即垂直方向的差异小于 diff_z_max），并且在机器人视点的水平视野范围和传感器范围内，
      if (std::abs(point.z - robot_position.z) < diff_z_max)
      {
        if (misc_utils_ns::InFOVSimple(Eigen::Vector3d(point.x, point.y, point.z),
                                       Eigen::Vector3d(robot_position.x, robot_position.y, robot_position.z),
                                       vertical_fov_ratio, sensor_range, xy_dist_threshold, z_diff_threshold))
        {
          if (robot_viewpoint.CheckVisibility<PlannerCloudPointType>(point, coverage_occlusion_thr))
          {
            planner_cloud_->cloud_->points[i].g = 255;
            covered_point_indices.push_back(i);
            continue;
          }
        }
      }

      // mark covered by visited viewpoints
      // 2. 检查点是否被候选视点所访问过，并且是否可以被这些候选视点看到。
      for (const auto &viewpoint_ind : viewpoint_manager->candidate_indices_)
      {
        if (viewpoint_manager->ViewPointVisited(viewpoint_ind))
        {
          if (viewpoint_manager->VisibleByViewPoint<PlannerCloudPointType>(point, viewpoint_ind))
          {
            planner_cloud_->cloud_->points[i].g = 255;
            covered_point_indices.push_back(i);
            break;
          }
        }
      }
    }

    // Dilate the covered area 膨胀已覆盖区域
    // 创建一个新的点云对象 squeezed_planner_cloud_，其中的点是根据 planner_cloud_->cloud_ 中的点进行了压缩（改变了高度值）。使用这个新点云构建了一个 kd 树。
    squeezed_planner_cloud_->cloud_->clear();
    for (const auto &point : planner_cloud_->cloud_->points)
    {
      PlannerCloudPointType squeezed_point = point;
      squeezed_point.z = point.z / parameters_.kCoverCloudZSqueezeRatio;
      squeezed_planner_cloud_->cloud_->points.push_back(squeezed_point);
    }
    squeezed_planner_cloud_kdtree_->setInputCloud(squeezed_planner_cloud_->cloud_);

    // 遍历 covered_point_indices 中的索引，对于每个索引，找到它周围半径为 coverage_dilation_radius 范围内的点，将它们标记为已覆盖
    for (const auto &ind : covered_point_indices)
    {
      PlannerCloudPointType point = planner_cloud_->cloud_->points[ind];
      std::vector<int> nearby_indices;
      std::vector<float> nearby_sqdist;
      squeezed_planner_cloud_kdtree_->radiusSearch(point, coverage_dilation_radius, nearby_indices, nearby_sqdist);
      if (!nearby_indices.empty())
      {
        for (const auto &idx : nearby_indices)
        {
          MY_ASSERT(idx >= 0 && idx < planner_cloud_->cloud_->points.size());
          planner_cloud_->cloud_->points[idx].g = 255;
        }
      }
    }

    // 遍历 planner_cloud_->cloud_ 中的每个点，如果其 g 属性大于 0，将其在 pointcloud_manager_ 中的对应索引位置标记为已覆盖。
    // 最终修改的是pointcloud_grid_中的points
    for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
    {
      PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
      if (point.g > 0)
      {
        int cloud_idx = 0;
        int cloud_point_idx = 0;
        pointcloud_manager_->GetCloudPointIndex(i, cloud_idx, cloud_point_idx);
        pointcloud_manager_->UpdateCoveredCloudPoints(cloud_idx, cloud_point_idx);
      }
    }

    std::cerr << "update_covered_consider_point: " << consider_point << std::endl;
  }

  /**
   * @brief 获取未覆盖区域（即未被视点覆盖到的区域），并计算未覆盖点的数量以及未覆盖边界点的数量，
   *        同时计算每个候选视点可看到的uncovered_poin和uncovered_frontier_point的数量
   *
   * @param viewpoint_manager
   * @param uncovered_point_num
   * @param uncovered_frontier_point_num
   */
  void PlanningEnv::GetUncoveredArea(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager> &viewpoint_manager,
                                     int &uncovered_point_num, int &uncovered_frontier_point_num)
  {
    // Clear viewpoint covered point list 清除视点已覆盖点列表中的内容，以便后续重新填充
    for (const auto &viewpoint_ind : viewpoint_manager->candidate_indices_)
    {
      viewpoint_manager->ResetViewPointCoveredPointList(viewpoint_ind);
    }

    // Get uncovered points
    uncovered_cloud_->cloud_->clear();
    uncovered_frontier_cloud_->cloud_->clear();
    uncovered_point_num = 0;
    uncovered_frontier_point_num = 0;

    // 遍历整个规划点云（planner_cloud_）中的每个点
    int consider_point_cnt = 0;
    for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
    {
      // 对于每个点，如果其 g 值大于 0，表示已被标记为覆盖的点，直接跳过。
      PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
      if (point.g > 0)
      {
        continue;
      }
      consider_point_cnt++;
      bool observed = false;
      // 对于每个候选视点的索引 viewpoint_ind，检查是否存在未被访问的视点，并且该点在该视点的可见范围内。
      // 如果满足条件，将该点添加到该视点的未覆盖点列表中，并将 observed 标志设置为 true。
      for (const auto &viewpoint_ind : viewpoint_manager->candidate_indices_)
      {
        if (!viewpoint_manager->ViewPointVisited(viewpoint_ind))
        {
          if (viewpoint_manager->VisibleByViewPoint<PlannerCloudPointType>(point, viewpoint_ind))
          {
            viewpoint_manager->AddUncoveredPoint(viewpoint_ind, uncovered_point_num);
            observed = true;
          }
        }
      }

      // observed==true, 表示该点被至少一个视点观察到，将该点添加到 uncovered_cloud_ 中，增加 uncovered_point_num 的计数。
      if (observed)
      {
        pcl::PointXYZI uncovered_point;
        uncovered_point.x = point.x;
        uncovered_point.y = point.y;
        uncovered_point.z = point.z;
        uncovered_point.intensity = i;
        uncovered_cloud_->cloud_->points.push_back(uncovered_point);
        uncovered_point_num++;
      }
    }

    // Check uncovered frontiers
    int consider_frontier_cnt = filtered_frontier_cloud_->cloud_->points.size();
    if (parameters_.kUseFrontier)
    {
      // 如果需要检查边界点，遍历过滤后的边界点云（filtered_frontier_cloud_）中的每个点。
      for (int i = 0; i < filtered_frontier_cloud_->cloud_->points.size(); i++)
      {
        pcl::PointXYZI point = filtered_frontier_cloud_->cloud_->points[i];
        bool observed = false;

        // 查是否存在未被访问的视点，并且该点在该视点的可见范围内。
        // 如果满足条件，将该点添加到相应视点的未覆盖边界点列表中，并将 observed 标志设置为 true
        for (const auto &viewpoint_ind : viewpoint_manager->candidate_indices_)
        {
          if (!viewpoint_manager->ViewPointVisited(viewpoint_ind))
          {
            if (viewpoint_manager->VisibleByViewPoint<pcl::PointXYZI>(point, viewpoint_ind))
            {
              viewpoint_manager->AddUncoveredFrontierPoint(viewpoint_ind, uncovered_frontier_point_num);
              observed = true;
            }
          }
        }
        // 如果 observed 为 true，表示该边界点被至少一个视点观察到，
        // 将该点添加到 uncovered_frontier_cloud_ 中，增加 uncovered_frontier_point_num 的计数。
        if (observed)
        {
          pcl::PointXYZI uncovered_frontier_point;
          uncovered_frontier_point.x = point.x;
          uncovered_frontier_point.y = point.y;
          uncovered_frontier_point.z = point.z;
          uncovered_frontier_point.intensity = i;
          uncovered_frontier_cloud_->cloud_->points.push_back(uncovered_frontier_point);
          uncovered_frontier_point_num++;
        }
      }
    }

    std::cerr << "consider_viewpoint_cnt: " << viewpoint_manager->candidate_indices_.size() << std::endl;
    std::cerr << "consider_keypose_point_cnt: " << consider_point_cnt << std::endl;
    std::cerr << "consider_frontier_cnt: " << consider_frontier_cnt << std::endl;
  }

  void PlanningEnv::GetVisualizationPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud)
  {
    pointcloud_manager_->GetVisualizationPointCloud(vis_cloud);
  }

  void PlanningEnv::PublishStackedCloud()
  {
    stacked_cloud_->Publish();
  }

  void PlanningEnv::PublishUncoveredCloud()
  {
    uncovered_cloud_->Publish();
  }

  void PlanningEnv::PublishUncoveredFrontierCloud()
  {
    uncovered_frontier_cloud_->Publish();
  }

} // namespace planning_env_ns