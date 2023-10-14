/**
 * @file viewpoint_manager.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that manages the viewpoints inside the local planning horizon
 * @version 0.1
 * @date 2020-06-10
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "viewpoint_manager/viewpoint_manager.h"

namespace viewpoint_manager_ns
{
  bool ViewPointManagerParameter::ReadParameters(ros::NodeHandle &nh)
  {
    kUseFrontier = misc_utils_ns::getParam<bool>(nh, "kUseFrontier", false);

    dimension_ = 2;

    kNumber.x() = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/number_x", 80);
    kNumber.y() = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/number_y", 80);
    kNumber.z() = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/number_z", 40);
    kViewPointNumber = kNumber.x() * kNumber.y() * kNumber.z();
    kRolloverStepsize = kNumber / 5; // 每个维度经过 kNumber / 5 个 vp 则需要滚动一次

    kResolution.x() = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_x", 0.5);
    kResolution.y() = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_y", 0.5);
    kResolution.z() = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_z", 0.5);

    kConnectivityHeightDiffThr = misc_utils_ns::getParam<double>(nh, "kConnectivityHeightDiffThr", 0.25);
    kViewPointCollisionMargin = misc_utils_ns::getParam<double>(nh, "kViewPointCollisionMargin", 0.5);
    kViewPointCollisionMarginZPlus = misc_utils_ns::getParam<double>(nh, "kViewPointCollisionMarginZPlus", 0.5);
    kViewPointCollisionMarginZMinus = misc_utils_ns::getParam<double>(nh, "kViewPointCollisionMarginZMinus", 0.5);
    kCollisionGridZScale = misc_utils_ns::getParam<double>(nh, "kCollisionGridZScale", 2.0);
    kCollisionGridResolution.x() = misc_utils_ns::getParam<double>(nh, "kCollisionGridResolutionX", 0.5);
    kCollisionGridResolution.y() = misc_utils_ns::getParam<double>(nh, "kCollisionGridResolutionY", 0.5);
    kCollisionGridResolution.z() = misc_utils_ns::getParam<double>(nh, "kCollisionGridResolutionZ", 0.5);
    kLineOfSightStopAtNearestObstacle = misc_utils_ns::getParam<bool>(nh, "kLineOfSightStopAtNearestObstacle", true);
    kCheckDynamicObstacleCollision = misc_utils_ns::getParam<bool>(nh, "kCheckDynamicObstacleCollision", true);
    kCollisionFrameCountMax = misc_utils_ns::getParam<int>(nh, "kCollisionFrameCountMax", 3);
    kViewPointHeightFromTerrain = misc_utils_ns::getParam<double>(nh, "kViewPointHeightFromTerrain", 0.75);
    kViewPointHeightFromTerrainChangeThreshold =
        misc_utils_ns::getParam<double>(nh, "kViewPointHeightFromTerrainChangeThreshold", 0.6);

    kCollisionPointThr = misc_utils_ns::getParam<int>(nh, "kCollisionPointThr", 3);

    for (int i = 0; i < dimension_; i++)
    {
      LocalPlanningHorizonSize(i) = kNumber(i) * kResolution(i);
    }

    kCollisionGridSize = Eigen::Vector3i::Ones();
    for (int i = 0; i < dimension_; i++)
    {
      kCollisionGridSize(i) =
          ceil((kNumber(i) * kResolution(i) + kViewPointCollisionMargin * 2) / kCollisionGridResolution(i));
    }

    kCoverageOcclusionThr = misc_utils_ns::getParam<double>(nh, "kCoverageOcclusionThr", 1.0);
    kCoverageDilationRadius = misc_utils_ns::getParam<double>(nh, "kCoverageDilationRadius", 1.0);
    kCoveragePointCloudResolution = misc_utils_ns::getParam<double>(nh, "kSurfaceCloudDwzLeafSize", 1.0);
    kSensorRange = misc_utils_ns::getParam<double>(nh, "kSensorRange", 10.0);
    kNeighborRange = misc_utils_ns::getParam<double>(nh, "kNeighborRange", 3.0);

    kVerticalFOVRatio = tan(M_PI / 15);
    kDiffZMax = kSensorRange * kVerticalFOVRatio;
    kInFovXYDistThreshold = 3 * (kCoveragePointCloudResolution / 2) / tan(M_PI / 15);
    kInFovZDiffThreshold = 3 * kCoveragePointCloudResolution;

    return true;
  }

  ViewPointManager::ViewPointManager(ros::NodeHandle &nh) : initialized_(false)
  {
    vp_.ReadParameters(nh);

    kdtree_viewpoint_candidate_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtree_viewpoint_in_collision_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    viewpoint_candidate_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    viewpoint_in_collision_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    // 使用RollingGrid维护vp
    grid_ = std::make_unique<rolling_grid_ns::RollingGrid>(vp_.kNumber);
    origin_ = Eigen::Vector3d::Zero();

    viewpoints_.resize(vp_.kViewPointNumber);
    for (int x = 0; x < vp_.kNumber.x(); x++)
    {
      for (int y = 0; y < vp_.kNumber.y(); y++)
      {
        for (int z = 0; z < vp_.kNumber.z(); z++)
        {
          Eigen::Vector3i sub(x, y, z);
          int ind = grid_->Sub2Ind(sub);
          viewpoints_[ind] = viewpoint_ns::ViewPoint();
        }
      }
    }

    graph_index_map_.resize(vp_.kViewPointNumber);
    for (auto &ind : graph_index_map_)
    {
      ind = -1;
    }

    ComputeConnectedNeighborIndices(); // 计算每个视点的26邻居
    ComputeInRangeNeighborIndices();   // 计算每个视点的一定范围内的邻居
    GetCollisionCorrespondence();

    local_planning_horizon_size_ = Eigen::Vector3d::Zero();
    for (int i = 0; i < vp_.dimension_; i++)
    {
      local_planning_horizon_size_(i) = vp_.kNumber(i) * vp_.kResolution(i);
    }
  }

  /**
   * @brief 计算每个视点连接的26邻居的索引和对应的距离。
   *
   */
  void ViewPointManager::ComputeConnectedNeighborIndices()
  {
    connected_neighbor_indices_.resize(vp_.kViewPointNumber);
    connected_neighbor_dist_.resize(vp_.kViewPointNumber);

    // 定义了一个名为idx_addon的容器，其中包含了视点的连接邻居的相对坐标。
    std::vector<Eigen::Vector3i> idx_addon;
    for (int x = -1; x <= 1; x++)
    {
      for (int y = -1; y <= 1; y++)
      {
        for (int z = -1; z <= 1; z++)
        {
          if (x == 0 && y == 0 && z == 0)
            continue;
          idx_addon.push_back(Eigen::Vector3i(x, y, z));
        }
      }
    }

    // 计算每个vp有效的邻居和相应的到邻居的距离
    for (int x = 0; x < vp_.kNumber.x(); x++)
    {
      for (int y = 0; y < vp_.kNumber.y(); y++)
      {
        for (int z = 0; z < vp_.kNumber.z(); z++)
        {
          Eigen::Vector3i sub(x, y, z);
          int ind = grid_->Sub2Ind(sub);
          for (int i = 0; i < idx_addon.size(); i++)
          {
            Eigen::Vector3i neighbor_sub = sub + idx_addon[i];
            if (grid_->InRange(neighbor_sub))
            {
              connected_neighbor_indices_[ind].push_back(grid_->Sub2Ind(neighbor_sub));
              double dist = sqrt(vp_.kResolution.x() * vp_.kResolution.x() * std::abs(idx_addon[i].x()) +
                                 vp_.kResolution.y() * vp_.kResolution.y() * std::abs(idx_addon[i].y()) +
                                 vp_.kResolution.z() * vp_.kResolution.z() * std::abs(idx_addon[i].z()));
              connected_neighbor_dist_[ind].push_back(dist);
            }
          }
        }
      }
    }
  }

  /**
   * @brief 计算每个视点的一定距离范围内邻居索引，通过构建Kd树和最近邻搜索操作，将范围内邻居视点的索引保存在in_range_neighbor_indices_中
   *
   */
  void ViewPointManager::ComputeInRangeNeighborIndices()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // 将每个视点转换为实际的三维空间坐标（point）
    for (int i = 0; i < vp_.kViewPointNumber; i++)
    {
      Eigen::Vector3i sub = grid_->Ind2Sub(i);
      pcl::PointXYZ point;
      point.x = sub.x() * vp_.kResolution.x() + vp_.kResolution.x() / 2.0;
      point.y = sub.y() * vp_.kResolution.y() + vp_.kResolution.y() / 2.0;
      point.z = sub.z() * vp_.kResolution.z() + vp_.kResolution.z() / 2.0;
      cloud->points.push_back(point);
    }

    // 构造kdtree进行范围搜索
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree =
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    kdtree->setInputCloud(cloud);
    in_range_neighbor_indices_.resize(vp_.kViewPointNumber);
    std::vector<int> in_range_indices;
    std::vector<float> in_range_sqdist;
    for (int i = 0; i < in_range_neighbor_indices_.size(); i++)
    {
      pcl::PointXYZ point = cloud->points[i];
      kdtree->radiusSearch(point, vp_.kNeighborRange, in_range_indices, in_range_sqdist);
      for (const auto &ind : in_range_indices)
      {
        in_range_neighbor_indices_[i].push_back(ind);
      }
    }
  }

  /**
   * @brief 获取碰撞网格和视点之间的对应关系，碰撞网格中每个grid存储其vp_.kViewPointCollisionMargin范围内的视点
   *
   */
  void ViewPointManager::GetCollisionCorrespondence()
  {
    misc_utils_ns::Timer timer("get collision grid correspondence");
    timer.Start();

    collision_grid_origin_ = Eigen::Vector3d::Zero();
    for (int i = 0; i < vp_.dimension_; i++)
    {
      collision_grid_origin_(i) -= vp_.kViewPointCollisionMargin;
    }
    std::vector<int> viewpoint_index_correspondence;
    collision_grid_ = std::make_unique<grid_ns::Grid<std::vector<int>>>(
        vp_.kCollisionGridSize, viewpoint_index_correspondence, collision_grid_origin_, vp_.kCollisionGridResolution, 2);
    collision_point_count_.resize(collision_grid_->GetCellNumber(), 0);

    pcl::PointCloud<pcl::PointXYZI>::Ptr viewpoint_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>());

    // Get viewpoint cloud
    // int count = 0;
    // 遍历视点网格的每个网格坐标，将每个视点的3D坐标点（point）添加到viewpoint_cloud中。
    // 每个视点的z坐标乘以vp_.kCollisionGridZScale
    for (int x = 0; x < vp_.kNumber.x(); x++)
    {
      for (int y = 0; y < vp_.kNumber.y(); y++)
      {
        for (int z = 0; z < vp_.kNumber.z(); z++)
        {
          int ind = grid_->Sub2Ind(Eigen::Vector3i(x, y, z));
          pcl::PointXYZI point;
          point.x = (x + 0.5) * vp_.kResolution.x();
          point.y = (y + 0.5) * vp_.kResolution.y();
          point.z = (z + 0.5) * vp_.kResolution.z();
          point.z *= vp_.kCollisionGridZScale;
          point.intensity = ind;
          viewpoint_cloud->points.push_back(point);
        }
      }
    }
    // std::cout << "computing collision grid viewpoint cloud size: " << viewpoint_cloud->points.size() << std::endl;
    kdtree->setInputCloud(viewpoint_cloud);
    std::vector<int> nearby_viewpoint_indices;
    std::vector<float> nearby_viewpoint_sqdist;
    int count = 0;
    for (int x = 0; x < vp_.kCollisionGridSize.x(); x++)
    {
      for (int y = 0; y < vp_.kCollisionGridSize.y(); y++)
      {
        for (int z = 0; z < vp_.kCollisionGridSize.z(); z++)
        {
          Eigen::Vector3d query_point_position = collision_grid_->Sub2Pos(x, y, z);
          pcl::PointXYZI query_point;
          query_point.x = query_point_position.x();
          query_point.y = query_point_position.y();
          query_point.z = query_point_position.z();
          query_point.z *= vp_.kCollisionGridZScale;
          kdtree->radiusSearch(query_point, vp_.kViewPointCollisionMargin, nearby_viewpoint_indices,
                               nearby_viewpoint_sqdist);
          int grid_ind = collision_grid_->Sub2Ind(x, y, z);
          for (int i = 0; i < nearby_viewpoint_indices.size(); i++)
          {
            int ind = nearby_viewpoint_indices[i];
            int viewpoint_ind = (int)(viewpoint_cloud->points[ind].intensity);
            MY_ASSERT(viewpoint_ind >= 0 && viewpoint_ind < vp_.kViewPointNumber);
            collision_grid_->GetCell(grid_ind).push_back(viewpoint_ind);
          }
        }
      }
    }

    timer.Stop(false);
  }

  /**
   * @brief 更新机器人的位置，并根据机器人位置的变化来更新视点的位置和其他属性，以适应机器人运动时视点的变化。
   *
   * @param robot_position 传入的最新的机器人位置
   * @return true 进行了滚动更新
   * @return false 没有进行滚动更新
   */
  bool ViewPointManager::UpdateRobotPosition(const Eigen::Vector3d &robot_position)
  {
    robot_position_ = robot_position;

    // 第一次调用该函数，需要进行初始化操作。
    if (!initialized_)
    {
      initialized_ = true;

      // 更新视点网格的原点origin_
      UpdateOrigin();

      // 遍历所有视点的网格坐标，并根据视点网格坐标计算视点的位置。
      for (int x = 0; x < vp_.kNumber.x(); x++)
      {
        for (int y = 0; y < vp_.kNumber.y(); y++)
        {
          for (int z = 0; z < vp_.kNumber.z(); z++)
          {
            int ind = grid_->Sub2Ind(Eigen::Vector3i(x, y, z));
            geometry_msgs::Point position;
            position.x = origin_.x() + x * vp_.kResolution.x() + vp_.kResolution.x() / 2.0;
            position.y = origin_.y() + y * vp_.kResolution.y() + vp_.kResolution.y() / 2.0;
            position.z = robot_position.z();

            // 设置视点的位置和重置视点的其他属性
            SetViewPointPosition(ind, position, true);
            ResetViewPoint(ind, true);
          }
        }
      }
    }

    Eigen::Vector3i robot_grid_sub;
    Eigen::Vector3d diff = robot_position_ - origin_;
    Eigen::Vector3i sub = Eigen::Vector3i::Zero();
    // 根据机器人位置的变化，计算机器人在视点网格中的位置robot_grid_sub(相对于滚动步长的)。
    for (int i = 0; i < vp_.dimension_; i++)
    {
      robot_grid_sub(i) = diff(i) > 0 ? static_cast<int>(diff(i) / (vp_.kRolloverStepsize(i) * vp_.kResolution(i))) : -1;
    }

    // 计算robot_grid_sub与视点网格的中心位置的偏移sub_diff（差了多少个滚动步长）。
    Eigen::Vector3i sub_diff = Eigen::Vector3i::Zero();
    for (int i = 0; i < vp_.dimension_; i++)
    {
      sub_diff(i) = (vp_.kNumber(i) / vp_.kRolloverStepsize(i)) / 2 - robot_grid_sub(i);
    }

    // 不需要滚动更新，直接退出
    if (sub_diff.x() == 0 && sub_diff.y() == 0 && sub_diff.z() == 0)
    {
      return false;
    }

    // 根据sub_diff计算出一个步进量rollover_step，用于表示视点网格的滚动。每个步进量都是 vp_.kRolloverStepsize 的整数倍，即需要滚过多少个视点。
    Eigen::Vector3i rollover_step;
    rollover_step.x() = std::abs(sub_diff.x()) > 0 ? vp_.kRolloverStepsize.x() * ((sub_diff.x() > 0) ? 1 : -1) * std::abs(sub_diff.x()) : 0;
    rollover_step.y() = std::abs(sub_diff.y()) > 0 ? vp_.kRolloverStepsize.y() * ((sub_diff.y() > 0) ? 1 : -1) * std::abs(sub_diff.y()) : 0;
    rollover_step.z() = std::abs(sub_diff.z()) > 0 ? vp_.kRolloverStepsize.z() * ((sub_diff.z() > 0) ? 1 : -1) * std::abs(sub_diff.z()) : 0;

    // std::cout << "rolling x: " << rollover_step.x() << " y: " << rollover_step.y() << " z: " << rollover_step.z()
    //           << std::endl;
    // 对视点网格进行滚动操作，以便更新视点的位置
    grid_->Roll(rollover_step);

    misc_utils_ns::Timer reset_timer("reset viewpoint");
    reset_timer.Start();

    //   origin_ = origin_ - rollover_step.cast<double>() * vp_.kResolution;
    origin_.x() -= rollover_step.x() * vp_.kResolution.x();
    origin_.y() -= rollover_step.y() * vp_.kResolution.y();
    origin_.z() -= rollover_step.z() * vp_.kResolution.z();

    // 在进行网格滚动后，函数获取更新后的视点索引列表updated_viewpoint_indices_，并根据这些索引，遍历每个视点，计算其新的位置，
    // 并调用SetViewPointPosition()和ResetViewPoint()函数来更新视点的位置和其他属性。
    grid_->GetUpdatedIndices(updated_viewpoint_indices_);
    for (const auto &ind : updated_viewpoint_indices_)
    {
      MY_ASSERT(grid_->InRange(ind));
      Eigen::Vector3i sub = grid_->Ind2Sub(ind);
      geometry_msgs::Point new_position;
      new_position.x = origin_.x() + sub.x() * vp_.kResolution.x() + vp_.kResolution.x() / 2.0;
      new_position.y = origin_.y() + sub.y() * vp_.kResolution.y() + vp_.kResolution.y() / 2.0;
      new_position.z = robot_position_.z();
      SetViewPointPosition(ind, new_position);
      ResetViewPoint(ind);
    }
    reset_timer.Stop(false);
    return true;
  }

  /**
   * @brief 更新原点位置，只用于初始化过程
   *
   */
  void ViewPointManager::UpdateOrigin()
  {
    for (int i = 0; i < vp_.dimension_; i++)
    {
      origin_(i) = robot_position_(i) - (vp_.kResolution(i) * vp_.kNumber(i)) / 2.0;
    }
  }

  int ViewPointManager::GetViewPointArrayInd(int viewpoint_ind, bool use_array_ind) const
  {
    MY_ASSERT(grid_->InRange(viewpoint_ind));
    return (use_array_ind ? viewpoint_ind : grid_->GetArrayInd(viewpoint_ind));
  }

  int ViewPointManager::GetViewPointInd(int viewpoint_array_ind) const
  {
    return grid_->GetInd(viewpoint_array_ind);
  }

  Eigen::Vector3i ViewPointManager::GetViewPointSub(Eigen::Vector3d position)
  {
    Eigen::Vector3d diff = position - origin_;
    Eigen::Vector3i sub = Eigen::Vector3i::Zero();
    for (int i = 0; i < vp_.dimension_; i++)
    {
      sub(i) = diff(i) > 0 ? static_cast<int>(diff(i) / vp_.kResolution(i)) : -1;
    }
    return sub;
  }

  int ViewPointManager::GetViewPointInd(Eigen::Vector3d position)
  {
    Eigen::Vector3i sub = GetViewPointSub(position);
    if (grid_->InRange(sub))
    {
      return grid_->Sub2Ind(sub);
    }
    else
    {
      return -1;
    }
  }

  void ViewPointManager::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &vis_cloud)
  {
    vis_cloud->clear();
    for (int i = 0; i < vp_.kViewPointNumber; i++)
    {
      if (IsViewPointCandidate(i, true))
      {
        geometry_msgs::Point position = GetViewPointPosition(i, true);
        pcl::PointXYZI vis_point;
        vis_point.x = position.x;
        vis_point.y = position.y;
        vis_point.z = position.z;
        if (ViewPointVisited(i, true))
        {
          vis_point.intensity = -1.0;
        }
        else
        {
          vis_point.intensity = GetViewPointCoveredPointNum(i, true);
          vis_point.intensity += i * 1.0 / 10000.0;
        }
        // if (viewpoints_[i].InCurrentFrameLineOfSight())
        // {
        //   vis_point.intensity = 100;
        // }
        // else
        // {
        //   vis_point.intensity = -1;
        // }
        vis_cloud->points.push_back(vis_point);
      }
    }
  }

  /**
   * @brief 检查视点是否与碰撞点云发生碰撞，并相应地更新视点的碰撞状态和碰撞帧计数。
   *
   * @param collision_cloud
   */
  void ViewPointManager::CheckViewPointCollisionWithCollisionGrid(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &collision_cloud)
  {
    // 遍历所有已知的视点，检查它们是否在碰撞中。如果视点在碰撞中，增加该视点的碰撞帧计数。
    for (int i = 0; i < viewpoints_.size(); i++)
    {
      if (ViewPointInCollision(i, true))
      {
        AddViewPointCollisionFrameCount(i, true);
      }
    }
    std::fill(collision_point_count_.begin(), collision_point_count_.end(), 0);

    // 设置碰撞格网的原点位置，该位置是当前RollingGrid的原点位置减去视点碰撞边界的大小。
    collision_grid_origin_ = origin_ - Eigen::Vector3d::Ones() * vp_.kViewPointCollisionMargin;
    collision_grid_->SetOrigin(collision_grid_origin_);

    // 将collision_cloud的所有点分配到collision_grid_中：
    // 遍历输入的碰撞点云中的每个点。将点的坐标转换为碰撞格网中的网格坐标，然后检查该坐标是否在碰撞格网的范围内。
    for (const auto &point : collision_cloud->points)
    {
      // 如果点在碰撞格网内，获取相应的碰撞格网索引，并将该格网中的包含的碰撞点云的点数量加一。
      Eigen::Vector3i collision_grid_sub = collision_grid_->Pos2Sub(point.x, point.y, point.z);
      if (collision_grid_->InRange(collision_grid_sub))
      {
        int collision_grid_ind = collision_grid_->Sub2Ind(collision_grid_sub);
        collision_point_count_[collision_grid_ind]++;

        // 如果某个碰撞格网的碰撞点数量超过了阈值 vp_.kCollisionPointThr，说明该网格内的点云含量较多，则获取该格网内的视点索引。
        // 遍历视点索引，计算碰撞网格的高度与视点的高度之差 z_diff。
        if (collision_point_count_[collision_grid_ind] >= vp_.kCollisionPointThr)
        {
          std::vector<int> collision_viewpoint_indices = collision_grid_->GetCellValue(collision_grid_ind);
          for (int i = 0; i < collision_viewpoint_indices.size(); i++)
          {
            int viewpoint_ind = collision_viewpoint_indices[i];
            MY_ASSERT(viewpoint_ind >= 0 && viewpoint_ind < vp_.kViewPointNumber);
            double z_diff = point.z - GetViewPointHeight(viewpoint_ind);

            // 如果 z_diff 在一定范围内（上下视点碰撞边界），将该视点标记为在碰撞中，并重置其碰撞帧计数。
            // 也就是如果一直检测到该视点是属于碰撞的，那该视点的碰撞帧计数会一直重置为0
            if ((z_diff >= 0 && z_diff <= vp_.kViewPointCollisionMarginZPlus) ||
                (z_diff < 0 && z_diff >= -vp_.kViewPointCollisionMarginZMinus))
            {
              SetViewPointCollision(viewpoint_ind, true);
              ResetViewPointCollisionFrameCount(viewpoint_ind);
            }
          }
        }
      }
    }
  }

  /**
   * @brief 判断给定位置是否与视点规划中的碰撞状态有关联，以便在规划过程中避免选择处于碰撞状态的位置作为视点。
   *
   * @param position  指定的位置
   * @return true
   * @return false
   */
  bool ViewPointManager::InCollision(const Eigen::Vector3d &position)
  {
    // 获取给定位置的视点索引
    int viewpoint_ind = GetViewPointInd(position);

    bool node_in_collision = false;
    // 检查视点索引是否在合法范围内。同时，检查给定位置的高度和对应视点的高度之差是否小于一个阈值
    if (InRange(viewpoint_ind) && std::abs(GetViewPointHeight(viewpoint_ind) - position.z()) <
                                      std::max(vp_.kResolution.x(), vp_.kResolution.y()) * 2)
    {
      // 如果给定位置的视点索引在合法范围内且高度满足条件，同时视点处于碰撞状态（使用 ViewPointInCollision 函数判断），则返回 true，表示给定位置处于碰撞状态。
      if (ViewPointInCollision(viewpoint_ind))
      {
        return true;
      }
    }
    return false;
  }

  bool ViewPointManager::InCurrentFrameLineOfSight(const Eigen::Vector3d &position)
  {
    int viewpoint_ind = GetViewPointInd(position);
    bool in_line_of_sight = false;
    if (InRange(viewpoint_ind))
    {
      if (ViewPointInCurrentFrameLineOfSight(viewpoint_ind))
      {
        return true;
      }
    }
    return false;
  }

  void ViewPointManager::CheckViewPointBoundaryCollision()
  {
    // Check for the polygon boundary and nogo zones
    for (int i = 0; i < vp_.kViewPointNumber; i++)
    {
      geometry_msgs::Point viewpoint_position = GetViewPointPosition(i, true);
      if ((!viewpoint_boundary_.points.empty() &&
           !misc_utils_ns::PointInPolygon(viewpoint_position, viewpoint_boundary_)))
      {
        SetViewPointCollision(i, true, true);
        continue;
      }
      for (int j = 0; j < nogo_boundary_.size(); j++)
      {
        if (!nogo_boundary_[j].points.empty() && misc_utils_ns::PointInPolygon(viewpoint_position, nogo_boundary_[j]))
        {
          SetViewPointCollision(i, true, true);

          break;
        }
      }
    }
  }

  void ViewPointManager::CheckViewPointCollision(const pcl::PointCloud<pcl::PointXYZI>::Ptr &collision_cloud)
  {
    CheckViewPointCollisionWithCollisionGrid(collision_cloud);
    CheckViewPointBoundaryCollision();
  }

  void ViewPointManager::CheckViewPointCollisionWithTerrain(const pcl::PointCloud<pcl::PointXYZI>::Ptr &terrain_cloud,
                                                            double collision_threshold)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr collision_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for (const auto &point : terrain_cloud->points)
    {
      if (point.intensity > collision_threshold)
      {
        collision_cloud->points.push_back(point);
      }
    }
    CheckViewPointCollisionWithCollisionGrid(collision_cloud);
  }

  /**
   * @brief 通过射线投射方法判断两个网格单元之间的视点是否存在直线可见性（从start_sub看），并根据配置参数和碰撞状态对该射线上的视点进行相应的设置。
   *
   * @param start_sub 起点
   * @param end_sub   终点
   * @param max_sub   范围上界
   * @param min_sub   范围下界
   */
  void ViewPointManager::CheckViewPointLineOfSightHelper(const Eigen::Vector3i &start_sub, const Eigen::Vector3i &end_sub,
                                                         const Eigen::Vector3i &max_sub, const Eigen::Vector3i &min_sub)
  {
    // 检查起始和结束坐标是否相同：如果起始坐标和结束坐标相同，直接返回，不进行检查。
    if (end_sub == start_sub)
      return;
    int viewpoint_ind = grid_->Sub2Ind(end_sub);

    // 根据结束坐标获取视点索引和位置：将结束坐标转换为网格索引，获取相应视点的位置。
    geometry_msgs::Point viewpoint_position = GetViewPointPosition(viewpoint_ind);

    // 使用 RayCast 函数对起始坐标和结束坐标之间的网格单元进行射线投射，将投射到的网格单元保存在 ray_cast_cells 中
    std::vector<Eigen::Vector3i> ray_cast_cells;
    misc_utils_ns::RayCast(start_sub, end_sub, max_sub, min_sub, ray_cast_cells);
    if (ray_cast_cells.size() > 1)
    {
      // 如果 vp_.kLineOfSightStopAtNearestObstacle 设置为 true，则遍历射线投射结果，
      // 在没遇到碰撞视点以前，所检测的所有视点的碰撞属性都设置为false，一旦遇到碰撞的视点且碰撞帧计数为0，将直线可见性设置为 false。
      // 清除视点的错误碰撞状态：如果启用了动态障碍物碰撞检测并且碰撞帧数超过阈值（计数越大，说明越久没有检测到该观测点是碰撞的），将该视点的碰撞状态设置为 false。
      if (vp_.kLineOfSightStopAtNearestObstacle)
      {
        bool occlude = false;
        for (int i = 1; i < ray_cast_cells.size(); i++)
        {
          int viewpoint_ind = grid_->Sub2Ind(ray_cast_cells[i]);
          if (ViewPointInCollision(viewpoint_ind) && GetViewPointCollisionFrameCount(viewpoint_ind) == 0)
          {
            occlude = true;
            break;
          }
          if (!occlude)
          {
            SetViewPointInLineOfSight(viewpoint_ind, true);
            if (vp_.kCheckDynamicObstacleCollision &&
                GetViewPointCollisionFrameCount(viewpoint_ind) > vp_.kCollisionFrameCountMax)

            {
              SetViewPointCollision(viewpoint_ind, false);
            }
          }
        }
      }
      // 如果 vp_.kLineOfSightStopAtNearestObstacle 设置为 false，则从射线投射结果的末尾开始遍历，遇到碰撞的视点且碰撞帧计数为0的，将碰撞状态设置为 true。
      // 如果碰撞状态发生变化并且碰撞状态恢复正常，将直线可见性设置为 true。
      else
      {
        bool hit_obstacle = false;
        bool in_line_of_sight = false;
        for (int i = ray_cast_cells.size() - 1; i >= 0; i--)
        {
          int viewpoint_ind = grid_->Sub2Ind(ray_cast_cells[i]);

          // 真正处于碰撞状态的视点
          if (ViewPointInCollision(viewpoint_ind) && GetViewPointCollisionFrameCount(viewpoint_ind) == 0)
          {
            hit_obstacle = true;
          }

          // 位于起点和碰撞状态的视点之间的视点
          if (hit_obstacle && !ViewPointInCollision(viewpoint_ind))
          {
            in_line_of_sight = true;
          }

          // 清除视点的错误碰撞状态
          if (hit_obstacle && ViewPointInCollision(viewpoint_ind) &&
              GetViewPointCollisionFrameCount(viewpoint_ind) > vp_.kCollisionFrameCountMax)

          {
            in_line_of_sight = true;
            if (vp_.kCheckDynamicObstacleCollision)
            {
              SetViewPointCollision(viewpoint_ind, false);
            }
          }

          // 位于起点和碰撞状态的视点之间的视点都设置为直线可见
          if (in_line_of_sight)
          {
            SetViewPointInLineOfSight(viewpoint_ind, true);
          }
        }

        // 如果一直没检测到碰撞视点，说明整条射线上的视点都是直线可见的
        if (!hit_obstacle)
        {
          for (int i = ray_cast_cells.size() - 1; i >= 0; i--)
          {
            int viewpoint_ind = grid_->Sub2Ind(ray_cast_cells[i]);
            SetViewPointInLineOfSight(viewpoint_ind, true);
          }
        }
      }

      // Set in current frame line of sight 当前帧内直线可见性，从第一个点往边界看，只到遇见第一个碰撞视点，中间所有视点都是当前帧内直线可见的
      bool occlude = false;
      for (int i = 1; i < ray_cast_cells.size(); i++)
      {
        int viewpoint_ind = grid_->Sub2Ind(ray_cast_cells[i]);
        if (ViewPointInCollision(viewpoint_ind) && GetViewPointCollisionFrameCount(viewpoint_ind) == 0)
        {
          occlude = true;
          break;
        }
        if (!occlude)
        {
          SetViewPointInCurrentFrameLineOfSight(viewpoint_ind, true);
        }
      }
    }
  }

  /**
   * @brief 检查机器人当前位置到各个视点的直线可见性
   *
   */
  void ViewPointManager::CheckViewPointLineOfSight()
  {
    if (!initialized_)
      return;

    // 初始化全为false
    for (int i = 0; i < viewpoints_.size(); i++)
    {
      SetViewPointInCurrentFrameLineOfSight(i, false, true);
    }

    // 通过机器人的位置获取机器人所在的视点索引，并将其直线可见性标记设置为 true。
    Eigen::Vector3i robot_sub = GetViewPointSub(robot_position_);
    MY_ASSERT(grid_->InRange(robot_sub));
    int robot_viewpoint_ind = grid_->Sub2Ind(robot_sub);
    SetViewPointInLineOfSight(robot_viewpoint_ind, true);
    SetViewPointInCurrentFrameLineOfSight(robot_viewpoint_ind, true);

    // 初始化一个数组 checked，用于跟踪已经检查过的网格单元
    std::vector<bool> checked(vp_.kViewPointNumber, false);
    std::vector<Eigen::Vector3i> ray_cast_cells;
    Eigen::Vector3i max_sub(vp_.kNumber.x() - 1, vp_.kNumber.y() - 1, vp_.kNumber.z() - 1);
    Eigen::Vector3i min_sub(0, 0, 0);

    int x_indices[2] = {0, vp_.kNumber.x() - 1};
    int y_indices[2] = {0, vp_.kNumber.y() - 1};
    int z_indices[2] = {0, vp_.kNumber.z() - 1};

    // 对每个维度上的不同坐标进行遍历：在每个维度上，分别对两个坐标（一个最小坐标、一个最大坐标）进行遍历。
    for (int xi = 0; xi < 2; xi++)
    {
      for (int y = 0; y < vp_.kNumber.y(); y++)
      {
        for (int z = 0; z < vp_.kNumber.z(); z++)
        {
          int x = x_indices[xi];
          Eigen::Vector3i end_sub(x, y, z);
          int array_ind = grid_->GetArrayInd(end_sub);
          if (!checked[array_ind])
          {
            // 检查两个网格单元之间是否存在直线可见性。
            CheckViewPointLineOfSightHelper(robot_sub, end_sub, max_sub, min_sub);
            // 在每个坐标的遍历结束后，将对应的网格单元标记为已检查，以避免重复检查。
            checked[array_ind] = true;
          }
        }
      }
    }

    for (int x = 0; x < vp_.kNumber.x(); x++)
    {
      for (int yi = 0; yi < 2; yi++)
      {
        for (int z = 0; z < vp_.kNumber.z(); z++)
        {
          int y = y_indices[yi];
          Eigen::Vector3i end_sub(x, y, z);
          int array_ind = grid_->GetArrayInd(end_sub);
          if (!checked[array_ind])
          {
            CheckViewPointLineOfSightHelper(robot_sub, end_sub, max_sub, min_sub);
            checked[array_ind] = true;
          }
        }
      }
    }

    for (int x = 0; x < vp_.kNumber.x(); x++)
    {
      for (int y = 0; y < vp_.kNumber.y(); y++)
      {
        for (int zi = 0; zi < 2; zi++)
        {
          int z = z_indices[zi];
          Eigen::Vector3i end_sub(x, y, z);
          int array_ind = grid_->GetArrayInd(end_sub);
          if (!checked[array_ind])
          {
            CheckViewPointLineOfSightHelper(robot_sub, end_sub, max_sub, min_sub);
            checked[array_ind] = true;
          }
        }
      }
    }
  }

  void ViewPointManager::CheckViewPointInFOV()
  {
    if (!initialized_)
      return;
    Eigen::Vector3i robot_sub = GetViewPointSub(robot_position_);
    MY_ASSERT(grid_->InRange(robot_sub));
    for (int i = 0; i < vp_.kViewPointNumber; i++)
    {
      geometry_msgs::Point viewpoint_position = GetViewPointPosition(i, true);
      if (!InRobotFOV(Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z)))
      {
        SetViewPointInLineOfSight(i, false, true);
      }
    }
    int robot_viewpoint_ind = grid_->Sub2Ind(robot_sub);
    SetViewPointInLineOfSight(robot_viewpoint_ind, true);
  }

  bool ViewPointManager::InFOV(const Eigen::Vector3d &point_position, const Eigen::Vector3d &viewpoint_position)
  {
    Eigen::Vector3d diff = point_position - viewpoint_position;
    double xy_diff = sqrt(diff.x() * diff.x() + diff.y() * diff.y());
    double z_diff = std::abs(diff.z());
    if (z_diff < vp_.kVerticalFOVRatio * xy_diff)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  /**
   * @brief 检查给定点是否在给定视点的视野范围和传感器范围内，以及是否满足垂直视角的条件，从而判断点是否在视点的视野范围内
   *
   * @param point_position
   * @param viewpoint_position
   * @return true
   * @return false
   */
  bool ViewPointManager::InFOVAndRange(const Eigen::Vector3d &point_position, const Eigen::Vector3d &viewpoint_position)
  {
    Eigen::Vector3d diff = point_position - viewpoint_position;
    double z_diff = std::abs(diff.z());
    if (z_diff > vp_.kDiffZMax)
    {
      return false;
    }
    double xy_diff = sqrt(diff.x() * diff.x() + diff.y() * diff.y());
    if (xy_diff > vp_.kSensorRange)
    {
      return false;
    }
    if (z_diff < vp_.kVerticalFOVRatio * xy_diff)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool ViewPointManager::InRobotFOV(const Eigen::Vector3d &position)
  {
    return InFOV(position, robot_position_);
  }

  void ViewPointManager::CheckViewPointConnectivity()
  {
    if (!initialized_)
      return;
    Eigen::Vector3i robot_sub = GetViewPointSub(robot_position_);
    MY_ASSERT(grid_->InRange(robot_sub));
    int robot_ind = grid_->Sub2Ind(robot_sub);
    int robot_array_ind = grid_->GetArrayInd(robot_sub);

    // 检查机器人是否在碰撞中：如果机器人所在的视点处于碰撞状态（ViewPointInCollision），
    // 则需要找到距离机器人最近且不在碰撞状态且具有直线可见性的视点。这是为了保证规划能够从一个合适的非碰撞视点开始。
    if (ViewPointInCollision(robot_ind))
    {
      // std::cout << "ViewPointManager::CheckViewPointConnectivity: robot in collision" << std::endl;
      // return;
      // Find the nearest viewpoint that is not in collision
      bool found_collision_free_viewpoint = false;
      double min_dist_to_robot = DBL_MAX;
      for (int i = 0; i < vp_.kViewPointNumber; i++)
      {
        int array_ind = grid_->GetArrayInd(i);
        if (!ViewPointInCollision(i))
        {
          geometry_msgs::Point position = GetViewPointPosition(i);
          Eigen::Vector3d viewpoint_position(position.x, position.y, position.z);
          double dist_to_robot = (viewpoint_position - robot_position_).norm();
          if (dist_to_robot < min_dist_to_robot)
          {
            min_dist_to_robot = dist_to_robot;
            robot_ind = i;
            robot_array_ind = array_ind;
            found_collision_free_viewpoint = true;
          }
        }
      }
      if (!found_collision_free_viewpoint)
      {
        std::cout << "All viewpoints in collision, exisiting" << std::endl;
        return;
      }
    }

    // 将所有视点的连接性标记都设置为 false
    for (auto &viewpoint : viewpoints_)
    {
      viewpoint.SetConnected(false);
    }

    // 创建一个长度为 vp_.kViewPointNumber 的数组 checked，用于标记已经检查过的视点。
    std::vector<bool> checked(vp_.kViewPointNumber, false);
    // 将机器人所在的视点标记为已连接，并将其添加到队列中。初始化时，已经将它的连接性标记设置为 true。
    checked[robot_ind] = true;
    SetViewPointConnected(robot_ind, true);

    // 使用队列进行广度优先搜索：使用队列实现广度优先搜索，从机器人视点开始，依次遍历与当前视点相邻且具有直线可见性的视点。
    std::list<int> queue;
    queue.push_back(robot_ind);
    int connected_viewpoint_count = 1;
    while (!queue.empty())
    {
      int cur_ind = queue.front();
      queue.pop_front();

      // 遍历26邻居点
      for (int i = 0; i < connected_neighbor_indices_[cur_ind].size(); i++)
      {
        int neighbor_ind = connected_neighbor_indices_[cur_ind][i];
        if (!grid_->InRange(neighbor_ind))
        {
          std::cout << "ViewPointManager::CheckViewPointConnectivity: neighbor ind out of bound" << std::endl;
          continue;
        }
        if (!checked[neighbor_ind] && !ViewPointInCollision(neighbor_ind) && ViewPointInLineOfSight(neighbor_ind))
        {
          // 如果相邻视点的高度差小于 vp_.kConnectivityHeightDiffThr，则将其连接性标记设置为 true，并将其添加到队列中。
          if (std::abs(GetViewPointHeight(cur_ind) - GetViewPointHeight(neighbor_ind)) < vp_.kConnectivityHeightDiffThr)
          {
            SetViewPointConnected(neighbor_ind, true);
            connected_viewpoint_count++;
            queue.push_back(neighbor_ind);
          }
        }
        checked[neighbor_ind] = true;
      }
    }
  }

  /**
   * @brief 根据给定的位置列表，更新视点及其邻居视点的访问状态, 将视点的visited_状态设置为true
   *
   * @param positions 给定的位置列表
   */
  void ViewPointManager::UpdateViewPointVisited(const std::vector<Eigen::Vector3d> &positions)
  {
    if (!initialized_)
      return;

    for (const auto &position : positions)
    {
      // 检查位置是否在局部规划范围内。如果位置不在局部规划范围内，跳过当前位置的处理。
      if (!InLocalPlanningHorizon(position))
      {
        continue;
      }

      // 更新当前视点及其邻居视点的访问状态：如果视点子索引在范围内，将其转换为视点索引，并使用 SetViewPointVisited 函数将当前视点的访问状态设置为 true。
      // 然后，遍历当前视点的邻居视点索引，并对每个邻居视点也进行相同的操作，将其访问状态设置为 true。
      Eigen::Vector3i viewpoint_sub = GetViewPointSub(position);
      if (grid_->InRange(viewpoint_sub))
      {
        int viewpoint_ind = grid_->Sub2Ind(viewpoint_sub);
        SetViewPointVisited(viewpoint_ind, true);
        for (const auto &neighbor_viewpoint_ind : in_range_neighbor_indices_[viewpoint_ind])
        {
          MY_ASSERT(grid_->InRange(neighbor_viewpoint_ind));
          SetViewPointVisited(neighbor_viewpoint_ind, true);
          int neighbor_array_ind = grid_->GetArrayInd(neighbor_viewpoint_ind);
        }
      }
    }
  }

  /**
   * @brief 根据给定的 grid_world 对象，更新视点的访问状态。
   *        它检查每个视点所对应的grid_world的网格单元在网格世界中的状态，如果该网格单元状态是COVERED_BY_OTHERS，就将这网格内的视点的visited_设置为true
   *
   * @param grid_world
   */
  void ViewPointManager::UpdateViewPointVisited(std::unique_ptr<grid_world_ns::GridWorld> const &grid_world)
  {
    for (int i = 0; i < viewpoints_.size(); i++)
    {
      geometry_msgs::Point viewpoint_position = GetViewPointPosition(i, true);
      int cell_ind = grid_world->GetCellInd(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z);
      if (grid_world->IndInBound((cell_ind)))
      {
        grid_world_ns::CellStatus cell_status = grid_world->GetCellStatus(cell_ind);
        if (cell_status == grid_world_ns::CellStatus::COVERED_BY_OTHERS)
        {
          SetViewPointVisited(i, true, true);
        }
      }
    }
  }

  /**
   * @brief 根据地形点云和相邻视点的高度信息来调整视点的高度，以适应地形的变化。
   *        确保机器人附近的视点高度与机器人的高度一致，同时利用地形点云来更新其他视点的高度。
   *
   * @param terrain_cloud
   * @param terrain_height_threshold
   */
  void ViewPointManager::SetViewPointHeightWithTerrain(const pcl::PointCloud<pcl::PointXYZI>::Ptr &terrain_cloud,
                                                       double terrain_height_threshold)
  {
    // Set the height of the viewpoint nearby the robot to be the height of the robot, in case there is no terrain cloud
    // within the blind spot.
    // 获取机器人附近的视点并设置高度：首先，获取机器人附近的视点的网格索引。然后，获取机器人附近的视点的位置，并将其高度设置为机器人的高度，
    // 以确保在盲区内没有地形点云时视点高度与机器人高度一致。此外，如果视点尚未设置地形高度，或者与机器人高度的变化超过了阈值，将相邻视点的高度也设置为机器人的高度。
    if (!initialized_)
      return;
    Eigen::Vector3i robot_sub = GetViewPointSub(robot_position_);
    int robot_ind = grid_->Sub2Ind(robot_sub);
    MY_ASSERT(grid_->InRange(robot_sub));
    geometry_msgs::Point robot_viewpoint_position = GetViewPointPosition(robot_ind);

    if (!ViewPointHasTerrainHeight(robot_ind) ||
        std::abs(robot_viewpoint_position.z - robot_position_.z()) > vp_.kViewPointHeightFromTerrainChangeThreshold)
    {
      robot_viewpoint_position.z = robot_position_.z();
      SetViewPointPosition(robot_ind, robot_viewpoint_position);
      for (int i = 0; i < in_range_neighbor_indices_[robot_ind].size(); i++)
      {
        int neighbor_ind = in_range_neighbor_indices_[robot_ind][i];
        MY_ASSERT(grid_->InRange(neighbor_ind));
        if (!ViewPointHasTerrainHeight(neighbor_ind) ||
            std::abs(GetViewPointHeight(neighbor_ind) - robot_position_.z()) > 0.6)
        {
          SetViewPointHeight(neighbor_ind, robot_position_.z());
        }
      }
    }

    // Set the height of other viewpoints
    // 遍历地形点云中的每个点，如果该点的强度（intensity）超过地形高度阈值，则跳过。然后，获取点的位置的网格索引，计算目标高度，并将该高度应用于视点。
    // 如果该视点尚未设置地形高度，或者目标高度低于当前高度，则将视点的高度设置为目标高度，并将视点标记为已设置地形高度。
    for (const auto &terrain_point : terrain_cloud->points)
    {
      if (terrain_point.intensity > terrain_height_threshold)
      {
        continue;
      }
      Eigen::Vector3i viewpoint_sub = GetViewPointSub(Eigen::Vector3d(terrain_point.x, terrain_point.y, terrain_point.z));
      if (grid_->InRange(viewpoint_sub))
      {
        int viewpoint_ind = grid_->Sub2Ind(viewpoint_sub);
        double target_height = terrain_point.z + vp_.kViewPointHeightFromTerrain;
        // If the viewpoint has not been set height with terrain points, or if there is a terrain point with a lower
        // height
        if (!ViewPointHasTerrainHeight(viewpoint_ind) || target_height < GetViewPointHeight(viewpoint_ind))
        {
          if (std::abs(target_height - GetViewPointHeight(viewpoint_ind)) >
              vp_.kViewPointHeightFromTerrainChangeThreshold)
          {
            ResetViewPoint(viewpoint_ind);
          }
          SetViewPointHeight(viewpoint_ind, target_height);
          SetViewPointHasTerrainHeight(viewpoint_ind, true);
        }
      }
    }

    // For viewpoints that are not set heights with terrain directly, use neighbors' heights
    // 对于未直接设置地形高度的视点，遍历所有视点。对于每个视点，检查其相邻视点中是否有已设置地形高度的视点。
    // 如果找到，则将该视点的高度设置为相邻视点的高度。如果相邻视点的高度与当前视点的高度变化超过阈值，将重置当前视点并使用相邻视点的高度。
    for (int i = 0; i < vp_.kViewPointNumber; i++)
    {
      if (!ViewPointHasTerrainHeight(i))
      {
        for (const auto &neighbor_ind : in_range_neighbor_indices_[i])
        {
          MY_ASSERT(grid_->InRange(neighbor_ind));
          if (ViewPointHasTerrainHeight(neighbor_ind))
          {
            double neighbor_height = GetViewPointHeight(neighbor_ind);
            if (std::abs(neighbor_height - GetViewPointHeight(i)) > vp_.kViewPointHeightFromTerrainChangeThreshold)
            {
              geometry_msgs::Point viewpoint_position = GetViewPointPosition(i);
              viewpoint_position.z = neighbor_height;
              ResetViewPoint(i);
              SetViewPointPosition(i, viewpoint_position);
            }
            else
            {
              SetViewPointHeight(i, neighbor_height);
            }
          }
        }
      }
    }
  }

  // Reset viewpoint
  void ViewPointManager::ResetViewPoint(int viewpoint_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    viewpoints_[array_ind].Reset();
  }

  void ViewPointManager::ResetViewPointCoverage()
  {
    for (auto &viewpoint : viewpoints_)
    {
      viewpoint.ResetCoverage();
    }
  }

  // Collision
  /**
   * @brief 判断指定视点是否是碰撞视点
   *
   * @param viewpoint_ind
   * @param use_array_ind
   * @return true
   * @return false
   */
  bool ViewPointManager::ViewPointInCollision(int viewpoint_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    return viewpoints_[array_ind].InCollision();
  }
  void ViewPointManager::SetViewPointCollision(int viewpoint_ind, bool in_collision, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    viewpoints_[array_ind].SetInCollision(in_collision);
  }
  // Line of Sight
  bool ViewPointManager::ViewPointInLineOfSight(int viewpoint_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    return viewpoints_[array_ind].InLineOfSight();
  }
  void ViewPointManager::SetViewPointInLineOfSight(int viewpoint_ind, bool in_line_of_sight, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    viewpoints_[array_ind].SetInLineOfSight(in_line_of_sight);
  }
  // Connectivity
  bool ViewPointManager::ViewPointConnected(int viewpoint_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    return viewpoints_[array_ind].Connected();
  }
  void ViewPointManager::SetViewPointConnected(int viewpoint_ind, bool connected, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    viewpoints_[array_ind].SetConnected(connected);
  }
  // Visited
  bool ViewPointManager::ViewPointVisited(int viewpoint_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    return viewpoints_[array_ind].Visited();
  }
  void ViewPointManager::SetViewPointVisited(int viewpoint_ind, bool visited, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    viewpoints_[array_ind].SetVisited(visited);
  }
  // Selected
  bool ViewPointManager::ViewPointSelected(int viewpoint_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    return viewpoints_[array_ind].Selected();
  }
  void ViewPointManager::SetViewPointSelected(int viewpoint_ind, bool selected, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    viewpoints_[array_ind].SetSelected(selected);
  }
  // Candidacy
  bool ViewPointManager::IsViewPointCandidate(int viewpoint_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    return viewpoints_[array_ind].IsCandidate();
  }
  void ViewPointManager::SetViewPointCandidate(int viewpoint_ind, bool candidate, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    viewpoints_[array_ind].SetCandidate(candidate);
  }
  // Terrain Height
  bool ViewPointManager::ViewPointHasTerrainHeight(int viewpoint_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    return viewpoints_[array_ind].HasTerrainHeight();
  }
  void ViewPointManager::SetViewPointHasTerrainHeight(int viewpoint_ind, bool has_terrain_height, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    viewpoints_[array_ind].SetHasTerrainHeight(has_terrain_height);
  }
  // In exploring cell
  bool ViewPointManager::ViewPointInExploringCell(int viewpoint_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    return viewpoints_[array_ind].InExploringCell();
  }
  void ViewPointManager::SetViewPointInExploringCell(int viewpoint_ind, bool in_exploring_cell, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    viewpoints_[array_ind].SetInExploringCell(in_exploring_cell);
  }
  // Height
  double ViewPointManager::GetViewPointHeight(int viewpoint_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    return viewpoints_[array_ind].GetHeight();
  }
  void ViewPointManager::SetViewPointHeight(int viewpoint_ind, double height, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    viewpoints_[array_ind].SetHeight(height);
  }
  // In current frame line of sight
  bool ViewPointManager::ViewPointInCurrentFrameLineOfSight(int viewpoint_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    return viewpoints_[array_ind].InCurrentFrameLineOfSight();
  }
  void ViewPointManager::SetViewPointInCurrentFrameLineOfSight(int viewpoint_ind, bool in_current_frame_line_of_sight,
                                                               bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    viewpoints_[array_ind].SetInCurrentFrameLineOfSight(in_current_frame_line_of_sight);
  }
  // Position
  geometry_msgs::Point ViewPointManager::GetViewPointPosition(int viewpoint_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    return viewpoints_[array_ind].GetPosition();
  }

  void ViewPointManager::SetViewPointPosition(int viewpoint_ind, geometry_msgs::Point position, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    viewpoints_[array_ind].SetPosition(position);
  }

  // Cell Ind
  int ViewPointManager::GetViewPointCellInd(int viewpoint_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    return viewpoints_[array_ind].GetCellInd();
  }
  void ViewPointManager::SetViewPointCellInd(int viewpoint_ind, int cell_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    viewpoints_[array_ind].SetCellInd(cell_ind);
  }
  // Collision frame count
  int ViewPointManager::GetViewPointCollisionFrameCount(int viewpoint_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    return viewpoints_[array_ind].GetCollisionFrameCount();
  }
  void ViewPointManager::AddViewPointCollisionFrameCount(int viewpoint_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    viewpoints_[array_ind].AddCollisionFrame();
  }
  void ViewPointManager::ResetViewPointCollisionFrameCount(int viewpoint_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    viewpoints_[array_ind].ResetCollisionFrameCount();
  }
  // Covered point list
  void ViewPointManager::ResetViewPointCoveredPointList(int viewpoint_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    viewpoints_[array_ind].ResetCoveredPointList();
    viewpoints_[array_ind].ResetCoveredFrontierPointList();
  }
  void ViewPointManager::AddUncoveredPoint(int viewpoint_ind, int point_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    viewpoints_[array_ind].AddCoveredPoint(point_ind);
  }
  void ViewPointManager::AddUncoveredFrontierPoint(int viewpoint_ind, int point_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    viewpoints_[array_ind].AddCoveredFrontierPoint(point_ind);
  }
  const std::vector<int> &ViewPointManager::GetViewPointCoveredPointList(int viewpoint_ind, bool use_array_ind) const
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    return viewpoints_[array_ind].GetCoveredPointList();
  }
  const std::vector<int> &ViewPointManager::GetViewPointCoveredFrontierPointList(int viewpoint_ind,
                                                                                 bool use_array_ind) const
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    return viewpoints_[array_ind].GetCoveredFrontierPointList();
  }

  int ViewPointManager::GetViewPointCoveredPointNum(int viewpoint_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    return viewpoints_[array_ind].GetCoveredPointNum();
  }

  int ViewPointManager::GetViewPointCoveredFrontierPointNum(int viewpoint_ind, bool use_array_ind)
  {
    int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
    return viewpoints_[array_ind].GetCoveredFrontierPointNum();
  }

  /**
   * @brief 获取 viewpoint_index 能看到 给定的point_list 中点的数量
   *
   * @param point_list      给定的待统计的点列表范围
   * @param viewpoint_index 给定的待检查的视点
   * @param use_array_ind
   * @return int
   */
  int ViewPointManager::GetViewPointCoveredPointNum(const std::vector<bool> &point_list, int viewpoint_index,
                                                    bool use_array_ind)
  {
    int covered_point_num = 0;
    for (const auto &point_ind : GetViewPointCoveredPointList(viewpoint_index, use_array_ind))
    {
      MY_ASSERT(misc_utils_ns::InRange<bool>(point_list, point_ind));
      if (!point_list[point_ind])
      {
        covered_point_num++;
      }
    }
    return covered_point_num;
  }

  int ViewPointManager::GetViewPointCoveredFrontierPointNum(const std::vector<bool> &frontier_point_list,
                                                            int viewpoint_index, bool use_array_ind)
  {
    int covered_frontier_point_num = 0;
    for (const auto &point_ind : GetViewPointCoveredFrontierPointList(viewpoint_index, use_array_ind))
    {
      MY_ASSERT(misc_utils_ns::InRange<bool>(frontier_point_list, point_ind));
      if (!frontier_point_list[point_ind])
      {
        covered_frontier_point_num++;
      }
    }
    return covered_frontier_point_num;
  }

  void ViewPointManager::UpdateViewPointCoveredPoint(std::vector<bool> &point_list, int viewpoint_index,
                                                     bool use_array_ind)
  {
    for (const auto &point_ind : GetViewPointCoveredPointList(viewpoint_index, use_array_ind))
    {
      MY_ASSERT(misc_utils_ns::InRange<bool>(point_list, point_ind));
      point_list[point_ind] = true;
    }
  }
  void ViewPointManager::UpdateViewPointCoveredFrontierPoint(std::vector<bool> &frontier_point_list, int viewpoint_index,
                                                             bool use_array_ind)
  {
    for (const auto &point_ind : GetViewPointCoveredFrontierPointList(viewpoint_index, use_array_ind))
    {
      MY_ASSERT(misc_utils_ns::InRange<bool>(frontier_point_list, point_ind));
      frontier_point_list[point_ind] = true;
    }
  }

  /**
   * @brief 根据一系列条件确定候选视点，并将它们的位置信息保存在点云中，同时构建相应的 k-d 树和候选视点图。
   *
   * @return int
   */
  int ViewPointManager::GetViewPointCandidate()
  {
    // 清除已有数据
    viewpoint_candidate_cloud_->clear();
    viewpoint_in_collision_cloud_->clear();
    candidate_indices_.clear();

    // 遍历所有视点
    for (int i = 0; i < vp_.kViewPointNumber; i++)
    {
      // 设置候选视点标记：首先，将当前视点的候选视点标记初始设置为 false
      SetViewPointCandidate(i, false);

      /*
        检查条件：通过一系列条件判断确定当前视点是否为候选视点
        1. 视点不在碰撞中
        2. 视点在当前位置具有直线可见性
        3. 视点在当前的连接区域中
      */
      if (!ViewPointInCollision(i) && ViewPointInLineOfSight(i) && ViewPointConnected(i))
      {
        // 设置候选视点标记
        SetViewPointCandidate(i, true);
        candidate_indices_.push_back(i);
        geometry_msgs::Point viewpoint_position = GetViewPointPosition(i);

        // 构建候选视点的点云
        pcl::PointXYZI point;
        point.x = viewpoint_position.x;
        point.y = viewpoint_position.y;
        point.z = viewpoint_position.z;
        viewpoint_candidate_cloud_->points.push_back(point);
      }

      // 构建碰撞视点的点云：对于碰撞中的视点，将其位置信息以及碰撞帧数信息添加到 viewpoint_in_collision_cloud_ 点云中
      if (ViewPointInCollision(i))
      {
        geometry_msgs::Point viewpoint_position = GetViewPointPosition(i);
        pcl::PointXYZI point;
        point.x = viewpoint_position.x;
        point.y = viewpoint_position.y;
        point.z = viewpoint_position.z;
        point.intensity = GetViewPointCollisionFrameCount(i); // 碰撞帧数
        viewpoint_in_collision_cloud_->points.push_back(point);
      }
    }
    // std::cout << "candidate viewpoint num: " << candidate_indices_.size() << std::endl;
    // 构建 k-d 树：如果候选视点的索引列表不为空，将候选视点的点云作为输入，构建 k-d 树
    if (!candidate_indices_.empty())
    {
      kdtree_viewpoint_candidate_->setInputCloud(viewpoint_candidate_cloud_);
    }

    // 如果碰撞视点的点云不为空，也构建 k-d 树
    if (!viewpoint_in_collision_cloud_->points.empty())
    {
      kdtree_viewpoint_in_collision_->setInputCloud(viewpoint_in_collision_cloud_);
    }

    // Construct a graph of all the viewpoints
    GetCandidateViewPointGraph(candidate_viewpoint_graph_, candidate_viewpoint_dist_, candidate_viewpoint_position_);

    return candidate_indices_.size();
  }

  /**
   * @brief 计算从起始候选视点到目标候选视点的最短路径
   *
   * @param start_viewpoint_ind
   * @param target_viewpoint_ind
   * @return nav_msgs::Path
   */
  nav_msgs::Path ViewPointManager::GetViewPointShortestPath(int start_viewpoint_ind, int target_viewpoint_ind)
  {
    nav_msgs::Path path;
    if (!InRange(start_viewpoint_ind))
    {
      ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath start viewpoint ind: " << start_viewpoint_ind
                                                                                         << " not in range");
      return path;
    }
    if (!InRange(target_viewpoint_ind))
    {
      ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath target viewpoint ind: " << target_viewpoint_ind
                                                                                          << " not in range");
      return path;
    }

    // 将viewpoint_id 转换成 candidate_viewpoint_id, 使用 candidate_viewpoint_id 在candidate_viewpoint中寻找路径
    int start_graph_ind = graph_index_map_[start_viewpoint_ind];
    int target_graph_ind = graph_index_map_[target_viewpoint_ind];

    std::vector<int> path_graph_indices;
    double path_length =
        misc_utils_ns::AStarSearch(candidate_viewpoint_graph_, candidate_viewpoint_dist_, candidate_viewpoint_position_,
                                   start_graph_ind, target_graph_ind, true, path_graph_indices);
    if (path_graph_indices.size() >= 2)
    {
      for (int i = 0; i < path_graph_indices.size(); i++)
      {
        int graph_idx = path_graph_indices[i];
        int ind = candidate_indices_[graph_idx]; // 将 candidate_viewpoint_id  转换成 viewpoint_id
        geometry_msgs::PoseStamped pose;
        pose.pose.position = GetViewPointPosition(ind);
        path.poses.push_back(pose);
      }
    }
    return path;
  }

  /**
   * @brief 计算从起始候选视点到目标候选视点的最短路径
   *
   * @param start_position
   * @param target_position
   * @return nav_msgs::Path
   */
  nav_msgs::Path ViewPointManager::GetViewPointShortestPath(const Eigen::Vector3d &start_position,
                                                            const Eigen::Vector3d &target_position)
  {
    nav_msgs::Path path;
    if (!InLocalPlanningHorizon(start_position))
    {
      ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath start position " << start_position.transpose()
                                                                                   << " not in local planning horizon");
      return path;
    }
    if (!InLocalPlanningHorizon(target_position))
    {
      ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath target position " << target_position.transpose()
                                                                                    << " not in local planning horizon");
      return path;
    }
    int start_viewpoint_ind = GetNearestCandidateViewPointInd(start_position);
    int target_viewpoint_ind = GetNearestCandidateViewPointInd(target_position);

    return GetViewPointShortestPath(start_viewpoint_ind, target_viewpoint_ind);
  }

  bool ViewPointManager::GetViewPointShortestPathWithMaxLength(const Eigen::Vector3d &start_position,
                                                               const Eigen::Vector3d &target_position,
                                                               double max_path_length, nav_msgs::Path &path)
  {
    if (!InLocalPlanningHorizon(start_position))
    {
      ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath start position " << start_position.transpose()
                                                                                   << " not in local planning horizon");
      return false;
    }
    if (!InLocalPlanningHorizon(target_position))
    {
      ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath target position " << target_position.transpose()
                                                                                    << " not in local planning horizon");
      return false;
    }
    int start_viewpoint_ind = GetNearestCandidateViewPointInd(start_position);
    int target_viewpoint_ind = GetNearestCandidateViewPointInd(target_position);
    int start_graph_ind = graph_index_map_[start_viewpoint_ind];
    int target_graph_ind = graph_index_map_[target_viewpoint_ind];

    std::vector<int> path_graph_indices;
    double shortest_path_dist = 0;
    bool found_path = misc_utils_ns::AStarSearchWithMaxPathLength(
        candidate_viewpoint_graph_, candidate_viewpoint_dist_, candidate_viewpoint_position_, start_graph_ind,
        target_graph_ind, true, path_graph_indices, shortest_path_dist, max_path_length);

    if (found_path && path_graph_indices.size() >= 2)
    {
      for (int i = 0; i < path_graph_indices.size(); i++)
      {
        int graph_idx = path_graph_indices[i];
        int ind = candidate_indices_[graph_idx];
        geometry_msgs::PoseStamped pose;
        pose.pose.position = GetViewPointPosition(ind);
        path.poses.push_back(pose);
      }
    }
    return found_path;
  }

  /**
   * @brief 根据grid_world的状态更新候选视点的in_exploring_cell_属性
   *
   * @param grid_world
   */
  void ViewPointManager::UpdateCandidateViewPointCellStatus(std::unique_ptr<grid_world_ns::GridWorld> const &grid_world)
  {
    for (const auto &ind : candidate_indices_)
    {
      int cell_ind = GetViewPointCellInd(ind);
      if (grid_world->IndInBound(cell_ind))
      {
        grid_world_ns::CellStatus cell_status = grid_world->GetCellStatus(cell_ind);
        if (cell_status == grid_world_ns::CellStatus::UNSEEN || cell_status == grid_world_ns::CellStatus::EXPLORING)
        {
          SetViewPointInExploringCell(ind, true);
        }
        else
        {
          SetViewPointInExploringCell(ind, false);
        }
      }
      else
      {
        ROS_WARN_STREAM("ViewPointManager::UpdateCandidateViewPointCellStatus: cell ind " << cell_ind << " out of bound");
      }
    }
  }

  /**
   * @brief 构建一个候选视点图，其中每个节点表示一个候选视点，节点之间的边表示视点之间的连接关系。
   *
   * @param graph 候选视点图
   * @param dist  候选视点图相应的距离
   * @param positions 各个候选视点的位置
   */
  void ViewPointManager::GetCandidateViewPointGraph(std::vector<std::vector<int>> &graph,
                                                    std::vector<std::vector<double>> &dist,
                                                    std::vector<geometry_msgs::Point> &positions)
  {
    graph.clear();
    dist.clear();
    positions.clear();
    if (candidate_indices_.empty())
    {
      return;
    }
    graph.resize(candidate_indices_.size());
    dist.resize(graph.size());

    // 构建索引映射：遍历所有候选视点的索引，将每个索引与其在候选视点列表中的位置对应起来，以便后续使用。
    for (int i = 0; i < candidate_indices_.size(); i++)
    {
      int ind = candidate_indices_[i];
      graph_index_map_[ind] = i;
    }

    // Build the graph
    // 构建图结构：对每个候选视点执行以下步骤。
    /*
      获取当前候选视点的索引：获取当前候选视点的索引值。
      记录位置信息：将当前候选视点的位置信息添加到 positions 列表中。
      遍历连接的邻居视点：对于当前候选视点连接的每个邻居视点执行以下步骤。
        获取邻居视点的索引和距离：获取邻居视点的索引和与当前视点的距离。
        判断邻居视点是否也是候选视点：如果邻居视点也是候选视点，将其在候选视点列表中的位置添加到当前视点的图结构中，同时将距离添加到对应的距离列表中。

    */
    for (int i = 0; i < candidate_indices_.size(); i++)
    {
      int cur_ind = candidate_indices_[i];
      positions.push_back(GetViewPointPosition(cur_ind));
      for (int j = 0; j < connected_neighbor_indices_[cur_ind].size(); j++)
      {
        int neighbor_ind = connected_neighbor_indices_[cur_ind][j];
        double neighbor_dist = connected_neighbor_dist_[cur_ind][j];
        if (IsViewPointCandidate(neighbor_ind))
        {
          graph[i].push_back(graph_index_map_[neighbor_ind]);
          dist[i].push_back(neighbor_dist);
        }
      }
    }
  }

  /**
   * @brief 从候选视点中找到距离给定位置最近的候选视点索引。
   *
   * @param position
   * @return int
   */
  int ViewPointManager::GetNearestCandidateViewPointInd(const Eigen::Vector3d &position)
  {
    int viewpoint_ind = GetViewPointInd(position);
    if (InRange(viewpoint_ind))
    {
      if (IsViewPointCandidate(viewpoint_ind))
      {
        return viewpoint_ind;
      }
    }
    if (!candidate_indices_.empty())
    {
      // Find the closest viewpoint that is a candidate viewpoint
      double min_dist = DBL_MAX;
      int min_dist_ind = -1;
      geometry_msgs::Point query_position;
      query_position.x = position.x();
      query_position.y = position.y();
      query_position.z = position.z();
      for (const auto &cur_viewpoint_ind : candidate_indices_)
      {
        geometry_msgs::Point cur_position = GetViewPointPosition(cur_viewpoint_ind);
        double dist =
            misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(cur_position, query_position);
        if (dist < min_dist)
        {
          min_dist = dist;
          min_dist_ind = cur_viewpoint_ind;
        }
      }
      return min_dist_ind;
    }
    else
    {
      std::cout << "Candidate viewpoint empty, can't find nearest candidate viewpoints to the position" << std::endl;
      return -1;
    }
  }

  void ViewPointManager::GetCollisionViewPointVisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
  {
    cloud->clear();
    for (const auto &point : viewpoint_in_collision_cloud_->points)
    {
      cloud->points.push_back(point);
    }
  }

  /**
   * @brief 判断给定的位置是否在局部规划范围内
   *
   * @param position  给定的位置
   * @return true
   * @return false
   */
  bool ViewPointManager::InLocalPlanningHorizon(const Eigen::Vector3d &position)
  {
    int viewpoint_ind = GetViewPointInd(position);
    if (InRange(viewpoint_ind))
    {
      // 计算最大高度差：计算最大允许的高度差，这里使用了 vp_.kResolution.x() 和 vp_.kResolution.y() 作为分辨率，并乘以一个系数 2
      double max_z_diff = std::max(vp_.kResolution.x(), vp_.kResolution.y()) * 2;
      geometry_msgs::Point viewpoint_position = GetViewPointPosition(viewpoint_ind);

      /*
      判断是否在局部规划范围内：
        1. 检查当前位置与视点位置的高度差是否小于之前计算的最大高度差。
        2. 判断当前位置对应的视点是否是候选视点, 或者是否位于碰撞中
      */
      if (std::abs(viewpoint_position.z - position.z()) < max_z_diff &&
          (IsViewPointCandidate(viewpoint_ind) || ViewPointInCollision(viewpoint_ind)))
      {
        return true;
      }
    }
    return false;
  }

} // namespace viewpoint_manager_ns