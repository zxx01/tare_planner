/**
 * @file rolling_occupancy_grid.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a rolling occupancy grid
 * @version 0.1
 * @date 2021-06-16
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "rolling_occupancy_grid/rolling_occupancy_grid.h"

namespace rolling_occupancy_grid_ns
{
  RollingOccupancyGrid::RollingOccupancyGrid(ros::NodeHandle &nh) : initialized_(false), dimension_(3)
  {
    double pointcloud_cell_size = misc_utils_ns::getParam<double>(nh, "kPointCloudCellSize", 18);
    double pointcloud_cell_height = misc_utils_ns::getParam<double>(nh, "kPointCloudCellHeight", 1.8);
    int pointcloud_cell_neighbor_number = misc_utils_ns::getParam<int>(nh, "kPointCloudManagerNeighborCellNum", 5);
    range_.x() = pointcloud_cell_size * pointcloud_cell_neighbor_number;
    range_.y() = pointcloud_cell_size * pointcloud_cell_neighbor_number;
    range_.z() = pointcloud_cell_height * pointcloud_cell_neighbor_number;

    resolution_.x() = misc_utils_ns::getParam<double>(nh, "rolling_occupancy_grid/resolution_x", 0.3);
    resolution_.y() = misc_utils_ns::getParam<double>(nh, "rolling_occupancy_grid/resolution_y", 0.3);
    resolution_.z() = misc_utils_ns::getParam<double>(nh, "rolling_occupancy_grid/resolution_z", 0.3);

    rollover_range_.x() = pointcloud_cell_size;
    rollover_range_.y() = pointcloud_cell_size;
    rollover_range_.z() = pointcloud_cell_height;

    for (int i = 0; i < dimension_; i++)
    {
      grid_size_(i) = static_cast<int>(range_(i) / resolution_(i));
      rollover_step_size_(i) = static_cast<int>(rollover_range_(i) / resolution_(i));
      origin_(i) = -range_(i) / 2;
    }

    rolling_grid_ = std::make_unique<rolling_grid_ns::RollingGrid>(grid_size_);
    occupancy_array_ = std::make_unique<grid_ns::Grid<CellState>>(grid_size_, UNKNOWN, origin_, resolution_);

    robot_position_ = Eigen::Vector3d(0, 0, 0);

    occupancy_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  }

  void RollingOccupancyGrid::InitializeOrigin(const Eigen::Vector3d &origin)
  {
    if (!initialized_)
    {
      initialized_ = true;
      origin_ = origin;
      occupancy_array_->SetOrigin(origin_);
    }
  }

  /**
   * @brief 在机器人位置变化时更新滚动占据网格的状态并执行相应的滚动操作。
   *
   * @param robot_position  传入的当前机器人位置
   * @return true           需要进行滚动更新
   * @return false          不需要进行滚动更新
   */
  bool RollingOccupancyGrid::UpdateRobotPosition(const Eigen::Vector3d &robot_position)
  {
    if (!initialized_)
    {
      return false;
    }

    // ********在上一次更新的big grid场景下计算前后两次更新之间机器人的移动情况*********
    robot_position_ = robot_position; // 更新机器人当前位置
    Eigen::Vector3i robot_grid_sub;
    Eigen::Vector3d diff = robot_position_ - origin_; // 将机器人当前位置与原点位置计算差值
    Eigen::Vector3i sub = Eigen::Vector3i::Zero();

    // 计算机器人当前位置在 big grid 中 cell 级的索引
    for (int i = 0; i < dimension_; i++)
    {
      robot_grid_sub(i) = diff(i) > 0 ? static_cast<int>(diff(i) / (rollover_step_size_(i) * resolution_(i))) : -1;
    }

    // 计算此次和上次机器人cell索引的差值，判断机器人运动情况
    Eigen::Vector3i sub_diff = Eigen::Vector3i::Zero();
    for (int i = 0; i < dimension_; i++)
    {
      sub_diff(i) = (grid_size_(i) / rollover_step_size_(i)) / 2 - robot_grid_sub(i);
    }

    // 如果机器人位置未发生变化，则直接返回false
    if (sub_diff.x() == 0 && sub_diff.y() == 0 && sub_diff.z() == 0)
    {
      return false;
    }

    // 计算需要滚动的步长(以cell的大小为一个单位，不够一个cell就不移动，以sub_diff的符号判断步长的正负)
    Eigen::Vector3i rollover_step(0, 0, 0);
    for (int i = 0; i < dimension_; i++)
    {
      rollover_step(i) =
          std::abs(sub_diff(i)) > 0 ? (i) * ((sub_diff(i) > 0) ? 1 : -1) * std::abs(sub_diff(i)) : 0;
    }

    // 获取被滚动出去的网格单元格的索引
    // std::cerr << "rollover_step:" << std::endl
    //           << rollover_step << std::endl;
    std::vector<int> rolled_out_grid_indices;
    rolling_grid_->GetRolledOutIndices(rollover_step, rolled_out_grid_indices);

    // Get rolled out occupancy cloud 根据滚动后的网格情况，更新滚出的占据点云数据 occupancy_cloud_
    occupancy_cloud_->clear();
    for (const auto &ind : rolled_out_grid_indices)
    {
      int array_ind = rolling_grid_->GetArrayInd(ind);
      if (occupancy_array_->GetCellValue(array_ind) == CellState::FREE)
      {
        Eigen::Vector3d position = occupancy_array_->Ind2Pos(ind);
        pcl::PointXYZI point;
        point.x = position.x();
        point.y = position.y();
        point.z = position.z();
        point.intensity = 0;
        occupancy_cloud_->points.push_back(point);
      }
      else if (occupancy_array_->GetCellValue(array_ind) == CellState::OCCUPIED)
      {
        Eigen::Vector3d position = occupancy_array_->Ind2Pos(ind);
        pcl::PointXYZI point;
        point.x = position.x();
        point.y = position.y();
        point.z = position.z();
        point.intensity = 1;
        occupancy_cloud_->points.push_back(point);
      }
    }

    // 执行网格滚动操作
    rolling_grid_->Roll(rollover_step);

    // Update origin 更新网格的原点
    for (int i = 0; i < dimension_; i++)
    {
      origin_(i) -= rollover_step(i) * resolution_(i);
    }
    occupancy_array_->SetOrigin(origin_);

    // 标记滚动后的网格单元格状态为未知
    std::vector<int> updated_grid_indices;
    rolling_grid_->GetUpdatedArrayIndices(updated_grid_indices);

    for (const auto &ind : updated_grid_indices)
    {
      occupancy_array_->SetCellValue(ind, CellState::UNKNOWN);
    }

    return true;
  }

  /**
   * @brief 更新滚动占据网格的占据状态，根据点云的intensity属性区分所属栅格状态
   *
   * @param cloud
   */
  void RollingOccupancyGrid::UpdateOccupancyStatus(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
  {
    if (!initialized_)
    {
      return;
    }
    for (const auto &point : cloud->points)
    {
      Eigen::Vector3i sub = occupancy_array_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
      if (!occupancy_array_->InRange(sub))
      {
        continue;
      }
      int array_ind = rolling_grid_->GetArrayInd(sub);
      if (point.intensity < 0.1)
      {
        occupancy_array_->SetCellValue(array_ind, CellState::FREE);
      }
      else if (point.intensity > 0.9)
      {
        occupancy_array_->SetCellValue(array_ind, CellState::OCCUPIED);
      }
    }
  }

  void RollingOccupancyGrid::RayTrace(const Eigen::Vector3d &origin, const Eigen::Vector3d &range)
  {
    // Eigen::Vector3i sub_max = occupancy_array_->GetSize() - Eigen::Vector3i::Ones();
    // Eigen::Vector3i sub_min = Eigen::Vector3i(0, 0, 0);
    Eigen::Vector3i origin_sub = occupancy_array_->Pos2Sub(origin);
    int ray_trace_count = 0;
    if (!occupancy_array_->InRange(origin_sub))
    {
      ROS_WARN("RollingOccupancyGrid::RayTrace(), robot not in range");
      return;
    }

    misc_utils_ns::UniquifyIntVector(updated_grid_indices_);

    for (const auto &ind : updated_grid_indices_)
    {
      if (occupancy_array_->InRange(ind))
      {
        Eigen::Vector3i cur_sub = occupancy_array_->Ind2Sub(ind);
        if (!occupancy_array_->InRange(cur_sub))
        {
          ROS_WARN_STREAM("RollingOccupancyGrid::RayTrace() " << cur_sub.transpose() << " sub out of range");
          continue;
        }
        int array_ind = rolling_grid_->GetArrayInd(ind);
        if (occupancy_array_->GetCellValue(array_ind) != OCCUPIED)
        {
          ROS_WARN_STREAM("RollingOccupancyGrid::RayTrace() " << cur_sub.transpose() << " sub not occupied");
          continue;
        }
        ray_trace_count++;
        std::vector<Eigen::Vector3i> ray_cast_cells;
        RayTraceHelper(origin_sub, cur_sub, ray_cast_cells);
        for (int i = 0; i < ray_cast_cells.size(); i++)
        {
          Eigen::Vector3i ray_sub = ray_cast_cells[i];
          int array_ind = rolling_grid_->GetArrayInd(ray_sub);
          if (occupancy_array_->GetCellValue(array_ind) == OCCUPIED)
          {
            break;
          }
          else
          {
            if (occupancy_array_->GetCellValue(array_ind) != OCCUPIED)
            {
              occupancy_array_->SetCellValue(array_ind, FREE);
            }
          }
        }
      }
    }
  }

  void RollingOccupancyGrid::RayTrace(const Eigen::Vector3d &origin)
  {
    if (!initialized_)
    {
      return;
    }
    RayTrace(origin, range_);
  }

  void RollingOccupancyGrid::RayTraceHelper(const Eigen::Vector3i &start_sub, const Eigen::Vector3i &end_sub,
                                            std::vector<Eigen::Vector3i> &cells)
  {
    cells.clear();
    MY_ASSERT(occupancy_array_->InRange(start_sub));
    MY_ASSERT(occupancy_array_->InRange(end_sub));

    if (start_sub == end_sub)
    {
      cells.push_back(start_sub);
      return;
    }
    Eigen::Vector3i diff_sub = end_sub - start_sub;
    double max_dist = diff_sub.squaredNorm();
    int step_x = misc_utils_ns::signum(diff_sub.x());
    int step_y = misc_utils_ns::signum(diff_sub.y());
    int step_z = misc_utils_ns::signum(diff_sub.z());
    double t_max_x = step_x == 0 ? DBL_MAX : misc_utils_ns::intbound(start_sub.x(), diff_sub.x());
    double t_max_y = step_y == 0 ? DBL_MAX : misc_utils_ns::intbound(start_sub.y(), diff_sub.y());
    double t_max_z = step_z == 0 ? DBL_MAX : misc_utils_ns::intbound(start_sub.z(), diff_sub.z());
    double t_delta_x = step_x == 0 ? DBL_MAX : (double)step_x / (double)diff_sub.x();
    double t_delta_y = step_y == 0 ? DBL_MAX : (double)step_y / (double)diff_sub.y();
    double t_delta_z = step_z == 0 ? DBL_MAX : (double)step_z / (double)diff_sub.z();
    double dist = 0;
    Eigen::Vector3i cur_sub = start_sub;

    while (occupancy_array_->InRange(cur_sub))
    {
      cells.push_back(cur_sub);
      dist = (cur_sub - start_sub).squaredNorm();
      int array_ind = rolling_grid_->GetArrayInd(cur_sub);
      if (cur_sub == end_sub || dist > max_dist || occupancy_array_->GetCellValue(array_ind) == OCCUPIED)
      {
        return;
      }
      if (t_max_x < t_max_y)
      {
        if (t_max_x < t_max_z)
        {
          cur_sub.x() += step_x;
          t_max_x += t_delta_x;
        }
        else
        {
          cur_sub.z() += step_z;
          t_max_z += t_delta_z;
        }
      }
      else
      {
        if (t_max_y < t_max_z)
        {
          cur_sub.y() += step_y;
          t_max_y += t_delta_y;
        }
        else
        {
          cur_sub.z() += step_z;
          t_max_z += t_delta_z;
        }
      }
    }
  }

  /**
   * @brief 从占据网格中获取Frontier点云的功能，通过检查unknown单元格以及其相邻的free单元格，识别Frontier区域，并将其位置添加到Frontier点云数据中。
   *        这里的Frontier只考虑xy方向上的是否满足条件，如果z方向上满足条件，那么当前的grid不会被设置为Frontier
   *
   * @param frontier_cloud  输出的Frontirer点云
   * @param origin          检测原点，一般为当前位置
   * @param range           检测范围(相对于origin)
   */
  void RollingOccupancyGrid::GetFrontier(pcl::PointCloud<pcl::PointXYZI>::Ptr &frontier_cloud,
                                         const Eigen::Vector3d &origin, const Eigen::Vector3d &range)
  {
    if (!initialized_)
    {
      return;
    }

    frontier_cloud->points.clear();
    // 计算索引范围(small grid levevl)
    Eigen::Vector3i sub_max = occupancy_array_->Pos2Sub(origin + range);
    Eigen::Vector3i sub_min = occupancy_array_->Pos2Sub(origin - range);
    Eigen::Vector3i origin_sub = occupancy_array_->Pos2Sub(origin);

    // 检查机器人是否在范围内
    if (!occupancy_array_->InRange(origin_sub))
    {
      ROS_WARN("RollingOccupancyGrid::GetFrontierInRange(), robot not in range");
      return;
    }
    int ray_trace_count = 0;

    // 计算small grid的个数
    int cell_num = occupancy_array_->GetCellNumber();
    for (int ind = 0; ind < cell_num; ind++)
    {
      Eigen::Vector3i cur_sub = occupancy_array_->Ind2Sub(ind); // 计算当前小方格在 small grid 上的 sub
      if (!occupancy_array_->InRange(cur_sub))
      {
        continue;
      }
      if (!InRange(cur_sub, sub_min, sub_max))
      {
        continue;
      }
      int array_ind = rolling_grid_->GetArrayInd(cur_sub);
      if (occupancy_array_->GetCellValue(array_ind) == UNKNOWN)
      {
        bool z_free = false;
        bool xy_free = false;

        // 检查cur_sub正下方的grid是否为FREE
        cur_sub(2)--;
        if (occupancy_array_->InRange(cur_sub))
        {
          array_ind = rolling_grid_->GetArrayInd(cur_sub);
          if (occupancy_array_->GetCellValue(array_ind) == FREE)
          {
            z_free = true;
            continue;
          }
        }
        // 检查cur_sub正上方的grid是否为FREE
        cur_sub(2) += 2;
        if (occupancy_array_->InRange(cur_sub))
        {
          array_ind = rolling_grid_->GetArrayInd(cur_sub);
          if (occupancy_array_->GetCellValue(array_ind) == FREE)
          {
            z_free = true;
            continue;
          }
        }
        cur_sub(2)--;

        // 检查cur_sub水平上四个方向上是否存在Free的grid
        for (int i = 0; i < 2; i++)
        {
          cur_sub(i)--;
          if (occupancy_array_->InRange(cur_sub))
          {
            array_ind = rolling_grid_->GetArrayInd(cur_sub);
            if (occupancy_array_->GetCellValue(array_ind) == FREE)
            {
              xy_free = true;
              cur_sub(i)++;
              break;
            }
          }
          cur_sub(i) += 2;
          if (occupancy_array_->InRange(cur_sub))
          {
            array_ind = rolling_grid_->GetArrayInd(cur_sub);
            if (occupancy_array_->GetCellValue(array_ind) == FREE)
            {
              xy_free = true;
              cur_sub(i)--;
              break;
            }
          }
          cur_sub(i)--;
        }

        if (xy_free && !z_free)
        {
          Eigen::Vector3d position = occupancy_array_->Sub2Pos(cur_sub);
          pcl::PointXYZI point;
          point.x = position.x();
          point.y = position.y();
          point.z = position.z();
          point.intensity = 0;
          frontier_cloud->points.push_back(point);
        }
      }
    }
  }

  void RollingOccupancyGrid::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &vis_cloud)
  {
    vis_cloud->clear();
    int cell_number = occupancy_array_->GetCellNumber();
    for (int i = 0; i < cell_number; i++)
    {
      int array_ind = rolling_grid_->GetArrayInd(i);
      if (occupancy_array_->GetCellValue(array_ind) == CellState::OCCUPIED ||
          occupancy_array_->GetCellValue(array_ind) == CellState::FREE)
      {
        Eigen::Vector3d position = occupancy_array_->Ind2Pos(i);
        pcl::PointXYZI point;
        point.x = position.x();
        point.y = position.y();
        point.z = position.z();
        if (occupancy_array_->GetCellValue(array_ind) == CellState::OCCUPIED)
        {
          point.intensity = 0.0;
        }
        else if (occupancy_array_->GetCellValue(array_ind) == CellState::FREE)
        {
          point.intensity = 1.0;
        }
        else
        {
          point.intensity = 2.0;
        }
        vis_cloud->points.push_back(point);
      }
    }
  }

  /**
   * @brief 检查sub是否在sub_min和sub_max之间，在就返回true，否则返回false
   *
   * @param sub 待检查元素
   * @param sub_min 范围下界
   * @param sub_max 范围上界
   * @return true
   * @return false
   */
  bool RollingOccupancyGrid::InRange(const Eigen::Vector3i &sub, const Eigen::Vector3i &sub_min,
                                     const Eigen::Vector3i &sub_max)
  {
    bool in_range = true;
    for (int i = 0; i < 3; i++)
    {
      in_range &= (sub(i) >= sub_min(i) && sub(i) <= sub_max(i));
    }
    return in_range;
  }

} // namespace rolling_occupancy_grid_ns