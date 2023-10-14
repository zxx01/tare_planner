//
// Created by caochao on 12/31/19.
//

#include "../../include/keypose_graph/keypose_graph.h"
#include <viewpoint_manager/viewpoint_manager.h>

namespace keypose_graph_ns
{
  KeyposeNode::KeyposeNode(double x, double y, double z, int node_ind, int keypose_id, bool is_keypose)
      : cell_ind_(0), node_ind_(node_ind), keypose_id_(keypose_id), is_keypose_(is_keypose), is_connected_(true)
  {
    position_.x = x;
    position_.y = y;
    position_.z = z;

    offset_to_keypose_.x = 0.0;
    offset_to_keypose_.y = 0.0;
    offset_to_keypose_.z = 0.0;
  }

  KeyposeNode::KeyposeNode(const geometry_msgs::Point &point, int node_ind, int keypose_id, bool is_keypose)
      : KeyposeNode(point.x, point.y, point.z, node_ind, keypose_id, is_keypose)
  {
  }

  KeyposeGraph::KeyposeGraph(ros::NodeHandle &nh)
      : allow_vertical_edge_(false), current_keypose_id_(0), kAddNodeMinDist(1.0), kAddEdgeCollisionCheckResolution(0.4), kAddEdgeCollisionCheckRadius(0.3), kAddEdgeConnectDistThr(3.0), kAddEdgeToLastKeyposeDistThr(3.0), kAddEdgeVerticalThreshold(1.0), kAddEdgeCollisionCheckPointNumThr(1)
  {
    ReadParameters(nh);
    kdtree_connected_nodes_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    connected_nodes_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    kdtree_nodes_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    nodes_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  }

  void KeyposeGraph::ReadParameters(ros::NodeHandle &nh)
  {
    kAddNodeMinDist = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddNodeMinDist", 0.5);
    kAddNonKeyposeNodeMinDist = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddNonKeyposeNodeMinDist", 0.5);
    kAddEdgeConnectDistThr = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeConnectDistThr", 0.5);
    kAddEdgeToLastKeyposeDistThr = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeToLastKeyposeDistThr", 0.5);
    kAddEdgeVerticalThreshold = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeVerticalThreshold", 0.5);
    kAddEdgeCollisionCheckResolution =
        misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeCollisionCheckResolution", 0.5);
    kAddEdgeCollisionCheckRadius = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeCollisionCheckRadius", 0.5);
    kAddEdgeCollisionCheckPointNumThr =
        misc_utils_ns::getParam<int>(nh, "keypose_graph/kAddEdgeCollisionCheckPointNumThr", 0.5);
  }

  /**
   * @brief 将一个新的节点添加到关键姿态图中，并初始化其邻居列表和距离列表。
   *
   * @param position
   * @param node_ind
   * @param keypose_id
   * @param is_keypose
   */
  void KeyposeGraph::AddNode(const geometry_msgs::Point &position, int node_ind, int keypose_id, bool is_keypose)
  {
    KeyposeNode new_node(position, node_ind, keypose_id, is_keypose);
    nodes_.push_back(new_node);
    std::vector<int> neighbors;
    graph_.push_back(neighbors);
    std::vector<double> neighbor_dist;
    dist_.push_back(neighbor_dist);
  }

  /**
   * @brief 添加节点和双向边
   *
   * @param position
   * @param node_ind
   * @param keypose_id
   * @param is_keypose
   * @param connected_node_ind
   * @param connected_node_dist
   */
  void KeyposeGraph::AddNodeAndEdge(const geometry_msgs::Point &position, int node_ind, int keypose_id, bool is_keypose,
                                    int connected_node_ind, double connected_node_dist)
  {
    AddNode(position, node_ind, keypose_id, is_keypose);
    AddEdge(connected_node_ind, node_ind, connected_node_dist);
  }

  /**
   * @brief 将两个节点连接起来，表示它们之间的边
   *
   * @param from_node_ind
   * @param to_node_ind
   * @param dist
   */
  void KeyposeGraph::AddEdge(int from_node_ind, int to_node_ind, double dist)
  {
    MY_ASSERT(from_node_ind >= 0 && from_node_ind < graph_.size() && from_node_ind < dist_.size());
    MY_ASSERT(to_node_ind >= 0 && to_node_ind < graph_.size() && to_node_ind < dist_.size());

    graph_[from_node_ind].push_back(to_node_ind);
    graph_[to_node_ind].push_back(from_node_ind);

    dist_[from_node_ind].push_back(dist);
    dist_[to_node_ind].push_back(dist);
  }

  bool KeyposeGraph::HasNode(const Eigen::Vector3d &position)
  {
    int closest_node_ind = -1;
    double min_dist = DBL_MAX;
    geometry_msgs::Point geo_position;
    geo_position.x = position.x();
    geo_position.y = position.y();
    geo_position.z = position.z();
    GetClosestNodeIndAndDistance(geo_position, closest_node_ind, min_dist);
    if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())
    {
      double xy_dist = misc_utils_ns::PointXYDist<geometry_msgs::Point>(geo_position, nodes_[closest_node_ind].position_);
      double z_dist = std::abs(geo_position.z - nodes_[closest_node_ind].position_.z);
      if (xy_dist < kAddNonKeyposeNodeMinDist && z_dist < 1.0)
      {
        return true;
      }
    }
    return false;
  }

  /**
   * @brief 检查两个给定节点之间是否存在连接的边。
   *
   * @param node_ind1 给定节点1
   * @param node_ind2 给定节点2
   * @return true
   * @return false
   */
  bool KeyposeGraph::HasEdgeBetween(int node_ind1, int node_ind2)
  {
    if (node_ind1 >= 0 && node_ind1 < nodes_.size() && node_ind2 >= 0 && node_ind2 < nodes_.size())
    {
      if (std::find(graph_[node_ind1].begin(), graph_[node_ind1].end(), node_ind2) != graph_[node_ind1].end() ||
          std::find(graph_[node_ind2].begin(), graph_[node_ind2].end(), node_ind1) != graph_[node_ind2].end())
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }

  bool KeyposeGraph::IsConnected(const Eigen::Vector3d &from_position, const Eigen::Vector3d &to_position)
  {
    geometry_msgs::Point from_node_position;
    from_node_position.x = from_position.x();
    from_node_position.y = from_position.y();
    from_node_position.z = from_position.z();
    int closest_from_node_ind = -1;
    double closest_from_node_dist = DBL_MAX;
    GetClosestNodeIndAndDistance(from_node_position, closest_from_node_ind, closest_from_node_dist);

    geometry_msgs::Point to_node_position;
    to_node_position.x = to_position.x();
    to_node_position.y = to_position.y();
    to_node_position.z = to_position.z();
    int closest_to_node_ind = -1;
    double closest_to_node_dist = DBL_MAX;
    GetClosestNodeIndAndDistance(to_node_position, closest_to_node_ind, closest_to_node_dist);

    if (closest_from_node_ind != -1 && closest_from_node_ind == closest_to_node_ind)
    {
      return true;
    }
    else if (HasEdgeBetween(closest_from_node_ind, closest_to_node_ind))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  /**
   * @brief 向关键姿态图中添加一个非关键姿态节点，并确保新节点与已有节点之间的距离满足特定条件
   *
   * @param new_node_position 待添加的node
   * @return int
   */
  int KeyposeGraph::AddNonKeyposeNode(const geometry_msgs::Point &new_node_position)
  {
    int new_node_index = -1;
    int closest_node_ind = -1;
    double closest_node_dist = DBL_MAX;

    // 查找最近节点
    GetClosestNodeIndAndDistance(new_node_position, closest_node_ind, closest_node_dist);
    // 如果存在最近节点，并且它与新节点的水平距离小于水平距离阈值，以及垂直距离小于1.0，那么直接返回最近节点的索引，表示不需要添加新节点。
    if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())
    {
      double xy_dist =
          misc_utils_ns::PointXYDist<geometry_msgs::Point>(new_node_position, nodes_[closest_node_ind].position_);
      double z_dist = std::abs(new_node_position.z - nodes_[closest_node_ind].position_.z);
      if (xy_dist < kAddNonKeyposeNodeMinDist && z_dist < 1.0)
      {
        return closest_node_ind;
      }
    }
    // 如果最近节点的距离不满足上述条件，则将新节点添加到关键姿态图中
    new_node_index = nodes_.size();
    KeyposeNode new_node(new_node_position, new_node_index, current_keypose_id_, false);
    new_node.SetCurrentKeyposePosition(current_keypose_position_);
    nodes_.push_back(new_node);
    std::vector<int> neighbors;
    graph_.push_back(neighbors);
    std::vector<double> neighbor_dist;
    dist_.push_back(neighbor_dist);

    return new_node_index;
  }

  /**
   * @brief 向关键姿态图（KeyposeGraph）中添加一条路径。该函数的目的是将一条路径的节点和边添加到关键姿态图中
   *
   * @param path
   */
  void KeyposeGraph::AddPath(const nav_msgs::Path &path)
  {
    if (path.poses.size() < 2)
    {
      return;
    }

    // 迭代添加给定path的节点和边
    int prev_node_index = -1;
    for (int i = 0; i < path.poses.size(); i++)
    {
      // 将当前点添加为一个非关键姿态节点，并获得当前节点的索引 cur_node_index
      int cur_node_index = AddNonKeyposeNode(path.poses[i].pose.position);
      if (i != 0)
      {
        // Add edge to previous node
        if (prev_node_index >= 0 && prev_node_index < nodes_.size())
        {
          // Check duplication
          // 检查是否已经存在连接前一个节点和当前节点的边，如果不存在则创建边，同时在两个节点间添加双向连接，以及计算并添加它们之间的距离。
          if (!HasEdgeBetween(prev_node_index, cur_node_index))
          {
            geometry_msgs::Point prev_node_position = nodes_[prev_node_index].position_;
            double dist_to_prev = misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(
                prev_node_position, path.poses[i].pose.position);
            graph_[prev_node_index].push_back(cur_node_index);
            graph_[cur_node_index].push_back(prev_node_index);

            dist_[prev_node_index].push_back(dist_to_prev);
            dist_[cur_node_index].push_back(dist_to_prev);
          }
        }
        else
        {
          ROS_ERROR_STREAM("KeyposeGraph::AddPath: prev_node_index " << prev_node_index << " out of bound [0, "
                                                                     << nodes_.size() - 1 << "]");
          return;
        }
      }
      prev_node_index = cur_node_index;
    }

    // 更新节点信息，构建新的kdtree
    UpdateNodes();
  }

  void KeyposeGraph::GetMarker(visualization_msgs::Marker &node_marker, visualization_msgs::Marker &edge_marker)
  {
    node_marker.points.clear();
    edge_marker.points.clear();

    for (const auto &node : nodes_)
    {
      node_marker.points.push_back(node.position_);
    }

    std::vector<std::pair<int, int>> added_edge;
    for (int i = 0; i < graph_.size(); i++)
    {
      int start_ind = i;
      for (int j = 0; j < graph_[i].size(); j++)
      {
        int end_ind = graph_[i][j];
        if (std::find(added_edge.begin(), added_edge.end(), std::make_pair(start_ind, end_ind)) == added_edge.end())
        {
          geometry_msgs::Point start_node_position = nodes_[start_ind].position_;
          geometry_msgs::Point end_node_position = nodes_[end_ind].position_;
          edge_marker.points.push_back(start_node_position);
          edge_marker.points.push_back(end_node_position);
          added_edge.emplace_back(start_ind, end_ind);
        }
      }
    }
  }

  int KeyposeGraph::GetConnectedNodeNum()
  {
    int connected_node_num = 0;
    for (int i = 0; i < nodes_.size(); i++)
    {
      if (nodes_[i].is_connected_)
      {
        connected_node_num++;
      }
    }
    return connected_node_num;
  }

  /**
   * @brief 将图中的节点信息转化为一个点云表示，其中连接的节点用颜色区分出来
   *
   * @param cloud
   */
  void KeyposeGraph::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    cloud->clear();
    for (const auto &node : nodes_)
    {
      pcl::PointXYZI point;
      point.x = node.position_.x;
      point.y = node.position_.y;
      point.z = node.position_.z;
      if (node.is_connected_)
      {
        point.intensity = 10;
      }
      else
      {
        point.intensity = -1;
      }
      cloud->points.push_back(point);
    }
  }

  /**
   * @brief 获取与给定节点query_ind相连的所有节点的索引connected_node_indices
   *
   * @param query_ind
   * @param connected_node_indices
   * @param constraints
   */
  void KeyposeGraph::GetConnectedNodeIndices(int query_ind, std::vector<int> &connected_node_indices,
                                             std::vector<bool> constraints)
  {
    if (nodes_.size() != constraints.size())
    {
      ROS_ERROR("KeyposeGraph::GetConnectedNodeIndices: constraints size not equal to node size");
      return;
    }
    if (query_ind < 0 || query_ind >= nodes_.size())
    {
      ROS_ERROR_STREAM("KeyposeGraph::GetConnectedNodeIndices: query_ind: " << query_ind << " out of range: [0, "
                                                                            << nodes_.size() << "]");
      return;
    }
    connected_node_indices.clear();
    std::vector<bool> visited(nodes_.size(), false);
    std::stack<int> dfs_stack;
    dfs_stack.push(query_ind);
    while (!dfs_stack.empty())
    {
      int current_ind = dfs_stack.top();
      connected_node_indices.push_back(current_ind);
      dfs_stack.pop();
      if (!visited[current_ind])
      {
        visited[current_ind] = true;
      }
      for (int i = 0; i < graph_[current_ind].size(); i++)
      {
        int neighbor_ind = graph_[current_ind][i];
        if (!visited[neighbor_ind] && constraints[neighbor_ind])
        {
          dfs_stack.push(neighbor_ind);
        }
      }
    }
  }

  /**
   * @brief 查关键姿态图中的节点和边是否与局部规划范围内的视点碰撞。
   *
   * @param robot_position
   * @param viewpoint_manager
   */
  void KeyposeGraph::CheckLocalCollision(const geometry_msgs::Point &robot_position,
                                         const std::shared_ptr<viewpoint_manager_ns::ViewPointManager> &viewpoint_manager)
  {
    // Get local planning horizon xy size
    int in_local_planning_horizon_count = 0;
    int collision_node_count = 0;
    int collision_edge_count = 0;
    int in_viewpoint_range_count = 0;
    Eigen::Vector3d viewpoint_resolution = viewpoint_manager->GetResolution();
    double max_z_diff = std::max(viewpoint_resolution.x(), viewpoint_resolution.y()) * 2;
    for (int i = 0; i < nodes_.size(); i++)
    {
      // 如果节点是关键姿态节点，则跳过不进行检查
      if (nodes_[i].is_keypose_)
      {
        continue;
      }

      // 获取节点的位置，并计算该节点在viewpoint_manager中的索引
      Eigen::Vector3d node_position =
          Eigen::Vector3d(nodes_[i].position_.x, nodes_[i].position_.y, nodes_[i].position_.z);
      int viewpoint_ind = viewpoint_manager->GetViewPointInd(node_position);
      bool node_in_collision = false;

      // 如果节点在可行范围内，并且在碰撞区域，那就删除这个节点相关的边
      if (viewpoint_manager->InRange(viewpoint_ind) &&
          std::abs(viewpoint_manager->GetViewPointHeight(viewpoint_ind) - node_position.z()) < max_z_diff)
      {
        in_local_planning_horizon_count++;
        in_viewpoint_range_count++;
        if (viewpoint_manager->ViewPointInCollision(viewpoint_ind))
        {
          node_in_collision = true;
          collision_node_count++;
          // Delete all the associated edges
          for (int j = 0; j < graph_[i].size(); j++)
          {
            int neighbor_ind = graph_[i][j];
            for (int k = 0; k < graph_[neighbor_ind].size(); k++)
            {
              if (graph_[neighbor_ind][k] == i)
              {
                graph_[neighbor_ind].erase(graph_[neighbor_ind].begin() + k);
                dist_[neighbor_ind].erase(dist_[neighbor_ind].begin() + k);
                k--;
              }
            }
          }
          graph_[i].clear();
          dist_[i].clear();
        }
        else // 如果节点在可行范围内，并且不在碰撞区域，进行edges的碰撞检查，发生碰撞的边都删掉
        {
          Eigen::Vector3d viewpoint_resolution = viewpoint_manager->GetResolution();
          double collision_check_resolution = std::min(viewpoint_resolution.x(), viewpoint_resolution.y()) / 2;
          // Check edge collision
          for (int j = 0; j < graph_[i].size(); j++)
          {
            int neighbor_ind = graph_[i][j];
            Eigen::Vector3d start_position = node_position;
            Eigen::Vector3d end_position = Eigen::Vector3d(
                nodes_[neighbor_ind].position_.x, nodes_[neighbor_ind].position_.y, nodes_[neighbor_ind].position_.z);
            std::vector<Eigen::Vector3d> interp_points;
            misc_utils_ns::LinInterpPoints(start_position, end_position, collision_check_resolution, interp_points);
            for (const auto &collision_check_position : interp_points)
            {
              int viewpoint_ind = viewpoint_manager->GetViewPointInd(collision_check_position);
              if (viewpoint_manager->InRange(viewpoint_ind))
              {
                if (viewpoint_manager->ViewPointInCollision(viewpoint_ind))
                {
                  geometry_msgs::Point viewpoint_position = viewpoint_manager->GetViewPointPosition(viewpoint_ind);
                  // Delete neighbors' edges
                  for (int k = 0; k < graph_[neighbor_ind].size(); k++)
                  {
                    if (graph_[neighbor_ind][k] == i)
                    {
                      collision_edge_count++;
                      graph_[neighbor_ind].erase(graph_[neighbor_ind].begin() + k);
                      dist_[neighbor_ind].erase(dist_[neighbor_ind].begin() + k);
                      k--;
                    }
                  }
                  // Delete the node's edge
                  graph_[i].erase(graph_[i].begin() + j);
                  dist_[i].erase(dist_[i].begin() + j);
                  j--;
                  break;
                }
              }
            }
          }
        }
      }
    }
  }

  /**
   * @brief 利用nodes_重新构造一棵kdtree
   *
   */
  void KeyposeGraph::UpdateNodes()
  {
    nodes_cloud_->clear();
    for (int i = 0; i < nodes_.size(); i++)
    {
      pcl::PointXYZI point;
      point.x = nodes_[i].position_.x;
      point.y = nodes_[i].position_.y;
      point.z = nodes_[i].position_.z;
      point.intensity = i;
      nodes_cloud_->points.push_back(point);
    }
    if (!nodes_cloud_->points.empty())
    {
      kdtree_nodes_->setInputCloud(nodes_cloud_);
    }
  }

  /**
   * @brief 通过连接性检查来确定节点之间的连接关系。
   *        从给定的机器人位置或者第一个关键姿势节点开始，递归地检查与其相连的所有节点，并标记它们为已连接。
   *
   * @param robot_position
   */
  void KeyposeGraph::CheckConnectivity(const geometry_msgs::Point &robot_position)
  {
    if (nodes_.empty())
    {
      return;
    }
    UpdateNodes(); // 构造kdtree

    // The first keypose node is always connected, set all the others to be disconnected
    int first_keypose_node_ind = -1;
    bool found_connected = false;

    for (int i = 0; i < nodes_.size(); i++)
    {
      if (nodes_[i].is_keypose_)
      {
        first_keypose_node_ind = i;
        break;
      }
    }

    // Check the connectivity starting from the robot
    for (int i = 0; i < nodes_.size(); i++)
    {
      nodes_[i].is_connected_ = false;
    }

    // 如果找到了第一个关键姿态节点，获取与该节点相连的所有节点的索引。
    if (first_keypose_node_ind >= 0 && first_keypose_node_ind < nodes_.size())
    {
      nodes_[first_keypose_node_ind].is_connected_ = true;
      connected_node_indices_.clear();
      std::vector<bool> constraint(nodes_.size(), true);
      GetConnectedNodeIndices(first_keypose_node_ind, connected_node_indices_, constraint);
    }
    // 如果没找到第一个关键姿态节点，获取与当前机器人位置节点相连的所有节点的索引。
    else
    {
      int robot_node_ind = -1;
      double robot_node_dist = DBL_MAX;
      GetClosestNodeIndAndDistance(robot_position, robot_node_ind, robot_node_dist);
      if (robot_node_ind >= 0 && robot_node_ind < nodes_.size())
      {
        nodes_[robot_node_ind].is_connected_ = true;
        connected_node_indices_.clear();
        std::vector<bool> constraint(nodes_.size(), true);
        GetConnectedNodeIndices(robot_node_ind, connected_node_indices_, constraint);
      }
      else
      {
        ROS_ERROR_STREAM("KeyposeGraph::CheckConnectivity: Cannot get closest robot node ind " << robot_node_ind);
      }
    }

    // 将连接的所有节点以点云的形式存储，并构造kdtree，同时也将所有的连接节点设置为is_connected_=true
    connected_nodes_cloud_->clear();
    for (int i = 0; i < connected_node_indices_.size(); i++)
    {
      int node_ind = connected_node_indices_[i];
      nodes_[node_ind].is_connected_ = true;
      pcl::PointXYZI point;
      point.x = nodes_[node_ind].position_.x;
      point.y = nodes_[node_ind].position_.y;
      point.z = nodes_[node_ind].position_.z;
      point.intensity = node_ind;
      connected_nodes_cloud_->points.push_back(point);
    }
    if (!connected_nodes_cloud_->points.empty())
    {
      kdtree_connected_nodes_->setInputCloud(connected_nodes_cloud_);
    }
  }

  /**
   * @brief 在关键姿态图中添加新的关键姿态节点，并与最近的关键姿态节点或其他节点连接形成边。
   *        如果与现有的keypose节点离得太近，就返回最近keypose节点
   *
   * @param keypose
   * @param planning_env
   * @return int
   */
  int KeyposeGraph::AddKeyposeNode(const nav_msgs::Odometry &keypose, const planning_env_ns::PlanningEnv &planning_env)
  {
    // current_keypose_id_每次传进来后都会自增，但是由于不是每次都会忘nodes中添加节点，因此new_node_ind不一定会增加
    current_keypose_position_ = keypose.pose.pose.position;
    current_keypose_id_ = static_cast<int>(keypose.pose.covariance[0]);
    int new_node_ind = nodes_.size();
    int keypose_node_count = 0;

    // 计算关键姿态节点数
    for (int i = 0; i < nodes_.size(); i++)
    {
      if (nodes_[i].is_keypose_)
      {
        keypose_node_count++;
      }
    }

    // 如果图为空或没有关键姿态节点，则将传入的keypose.position作为新节点添加，并将其标记为关键姿态节点。然后返回新加节点的索引。
    if (nodes_.empty() || keypose_node_count == 0)
    {
      AddNode(current_keypose_position_, new_node_ind, current_keypose_id_, true);
      return new_node_ind;
    }
    else
    {
      double min_dist = DBL_MAX;
      int min_dist_ind = -1;
      double last_keypose_dist = DBL_MAX;
      int last_keypose_ind = -1;
      int max_keypose_id = 0;
      std::vector<int> in_range_node_indices;
      std::vector<double> in_range_node_dist;

      // 遍历已有节点计算离传入的姿态点最近的关键姿态节点：
      for (int i = 0; i < nodes_.size(); i++)
      {
        // 如果不允许垂直边（allow_vertical_edge_ 为 false），
        // 并且当前的已有节点与传入的当前关键姿态节点的高度差超过阈值 kAddEdgeVerticalThreshold，则跳过当前已有节点。
        if (!allow_vertical_edge_)
        {
          if (std::abs(nodes_[i].position_.z - current_keypose_position_.z) > kAddEdgeVerticalThreshold)
          {
            continue;
          }
        }

        // 计算当前节点与当前关键姿态节点的距离。
        double dist = misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_,
                                                                                              current_keypose_position_);
        // 如果当前节点是关键姿态节点，且距离比之前的最小距离小，则更新最小距离和最小距离索引。
        if (dist < min_dist && nodes_[i].is_keypose_)
        {
          min_dist = dist;
          min_dist_ind = i;
        }

        // 找到上一个记录的keypose的id
        int keypose_id = nodes_[i].keypose_id_;
        if (keypose_id > max_keypose_id && nodes_[i].is_keypose_)
        {
          last_keypose_dist = dist;
          last_keypose_ind = i;
          max_keypose_id = keypose_id;
        }

        // 如果当前节点与传入的当前关键姿态节点的距离小于阈值 kAddEdgeConnectDistThr，
        // 则将当前节点的索引和距离添加到 in_range_node_indices 和 in_range_node_dist 列表中。
        if (dist < kAddEdgeConnectDistThr)
        {
          in_range_node_indices.push_back(i);
          in_range_node_dist.push_back(dist);
        }
      }

      // If the closest keypose node is  some distance away
      if (min_dist_ind >= 0 && min_dist_ind < nodes_.size())
      {
        // 如果最近的关键姿态节点与当前传入的关键姿态节点的距离大于 kAddNodeMinDist，
        // 则根据last_keypose_dist的大小，将当前传入关键姿态点与最近的关键姿态节点或前一个传入的关键姿态节点连接，形成边
        if (min_dist > kAddNodeMinDist)
        {
          // If the last keypose is within range
          if (last_keypose_dist < kAddEdgeToLastKeyposeDistThr &&
              last_keypose_ind >= 0 && last_keypose_ind < nodes_.size())
          {
            // Add edge to the last keypose node
            AddNodeAndEdge(current_keypose_position_, new_node_ind, current_keypose_id_, true, last_keypose_ind,
                           last_keypose_dist);
          }
          else
          {
            // Add edge to the nearest node
            AddNodeAndEdge(current_keypose_position_, new_node_ind, current_keypose_id_, true, min_dist_ind, min_dist);
          }

          // Check other nodes
          // 对于在范围内的其他节点，遍历这些节点，检查它们与当前节点连接的边是否发生碰撞，在不碰撞的情况下添加边。
          if (!in_range_node_indices.empty())
          {
            for (int idx = 0; idx < in_range_node_indices.size(); idx++)
            {
              int in_range_ind = in_range_node_indices[idx];
              if (in_range_ind >= 0 && in_range_ind < nodes_.size())
              {
                // Collision check
                KeyposeNode neighbor_node = nodes_[in_range_ind];
                if (std::find(graph_[new_node_ind].begin(), graph_[new_node_ind].end(), in_range_ind) !=
                    graph_[new_node_ind].end())
                  continue;
                double neighbor_node_dist = in_range_node_dist[idx];
                double diff_x = neighbor_node.position_.x - current_keypose_position_.x;
                double diff_y = neighbor_node.position_.y - current_keypose_position_.y;
                double diff_z = neighbor_node.position_.z - current_keypose_position_.z;
                int check_point_num = static_cast<int>(neighbor_node_dist / kAddEdgeCollisionCheckResolution);
                bool in_collision = false;
                for (int i = 0; i < check_point_num; i++)
                {
                  // std::cout << "checking the " << i << " collision point" << std::endl;
                  double check_point_x =
                      current_keypose_position_.x + kAddEdgeCollisionCheckResolution * i * diff_x / neighbor_node_dist;
                  double check_point_y =
                      current_keypose_position_.y + kAddEdgeCollisionCheckResolution * i * diff_y / neighbor_node_dist;
                  double check_point_z =
                      current_keypose_position_.z + kAddEdgeCollisionCheckResolution * i * diff_z / neighbor_node_dist;
                  if (planning_env.InCollision(check_point_x, check_point_y, check_point_z))
                  {
                    in_collision = true;
                    break;
                  }
                }
                if (!in_collision)
                {
                  AddEdge(new_node_ind, in_range_ind, neighbor_node_dist);
                }
              }
            }
          }
          return new_node_ind;
        }
        else
        {
          return min_dist_ind;
        }
      }
      else
      {
        ROS_ERROR_STREAM("KeyposeGraph::AddKeyposeNode: Nearest keypose ind out of range: " << min_dist_ind);
        return new_node_ind;
      }
    }
  }

  bool KeyposeGraph::IsPositionReachable(const geometry_msgs::Point &point, double dist_threshold)
  {
    int closest_node_ind = 0;
    double closest_node_dist = DBL_MAX;
    GetClosestConnectedNodeIndAndDistance(point, closest_node_ind, closest_node_dist);
    if (closest_node_ind >= 0 && closest_node_ind < nodes_.size() && closest_node_dist < dist_threshold)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool KeyposeGraph::IsPositionReachable(const geometry_msgs::Point &point)
  {
    int closest_node_ind = 0;
    double closest_node_dist = DBL_MAX;
    GetClosestConnectedNodeIndAndDistance(point, closest_node_ind, closest_node_dist);
    if (closest_node_ind >= 0 && closest_node_ind < nodes_.size() && closest_node_dist < kAddNonKeyposeNodeMinDist)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  /**
   * @brief 找到给定点的ind
   *
   * @param point 给定点
   * @return int
   */
  int KeyposeGraph::GetClosestNodeInd(const geometry_msgs::Point &point)
  {
    int node_ind = 0;
    double min_dist = DBL_MAX;
    GetClosestNodeIndAndDistance(point, node_ind, min_dist);
    return node_ind;
  }

  /**
   * @brief 从关键姿态图中获取指定点point的最近节点索引和距离。
   *        通过使用 Kd树进行最近邻搜索来加速查找，如果搜索失败或者结果不在范围内，则通过遍历所有节点来查找最近的节点。
   *
   * @param point     指定点
   * @param node_ind  找到的最近点id
   * @param dist      找到的最近点距离
   */
  void KeyposeGraph::GetClosestNodeIndAndDistance(const geometry_msgs::Point &point, int &node_ind, double &dist)
  {
    node_ind = -1;
    dist = DBL_MAX;
    if (nodes_cloud_->points.empty())
    {
      node_ind = -1;
      dist = DBL_MAX;
      return;
    }
    pcl::PointXYZI search_point;
    search_point.x = point.x;
    search_point.y = point.y;
    search_point.z = point.z;
    std::vector<int> nearest_neighbor_node_indices(1);
    std::vector<float> nearest_neighbor_squared_dist(1);
    kdtree_nodes_->nearestKSearch(search_point, 1, nearest_neighbor_node_indices, nearest_neighbor_squared_dist);
    if (!nearest_neighbor_node_indices.empty() && nearest_neighbor_node_indices.front() >= 0 &&
        nearest_neighbor_node_indices.front() < nodes_cloud_->points.size())
    {
      node_ind = static_cast<int>(nodes_cloud_->points[nearest_neighbor_node_indices.front()].intensity);
      dist = sqrt(nearest_neighbor_squared_dist.front());
    }
    else
    {
      ROS_WARN_STREAM("KeyposeGraph::GetClosestNodeIndAndDistance: search for nearest neighbor failed with "
                      << nodes_cloud_->points.size() << " nodes.");
      if (!nearest_neighbor_node_indices.empty())
      {
        ROS_WARN_STREAM("Nearest neighbor node Ind: " << nearest_neighbor_node_indices.front());
      }
      for (int i = 0; i < nodes_.size(); i++)
      {
        geometry_msgs::Point node_position = nodes_[i].position_;
        double dist_to_query =
            misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(point, node_position);
        if (dist_to_query < dist)
        {
          dist = dist_to_query;
          node_ind = i;
        }
      }
    }
  }

  /**
   * @brief 在关键姿态图中查找最接近给定点的节点，并返回该节点的索引以及与给定点之间的距离。
   *
   * @param point     给定点
   * @param node_ind  最近节点索引
   * @param dist      最近节点距离
   */
  void KeyposeGraph::GetClosestConnectedNodeIndAndDistance(const geometry_msgs::Point &point, int &node_ind, double &dist)
  {
    if (connected_nodes_cloud_->points.empty())
    {
      node_ind = -1;
      dist = DBL_MAX;
      return;
    }
    pcl::PointXYZI search_point;
    search_point.x = point.x;
    search_point.y = point.y;
    search_point.z = point.z;
    std::vector<int> nearest_neighbor_node_indices(1);
    std::vector<float> nearest_neighbor_squared_dist(1);
    kdtree_connected_nodes_->nearestKSearch(search_point, 1, nearest_neighbor_node_indices,
                                            nearest_neighbor_squared_dist);
    if (!nearest_neighbor_node_indices.empty() && nearest_neighbor_node_indices.front() >= 0 &&
        nearest_neighbor_node_indices.front() < connected_nodes_cloud_->points.size())
    {
      node_ind = static_cast<int>(connected_nodes_cloud_->points[nearest_neighbor_node_indices.front()].intensity);
      dist = sqrt(nearest_neighbor_squared_dist.front());
    }
    else
    {
      ROS_WARN_STREAM("KeyposeGraph::GetClosestNodeInd: search for nearest neighbor failed with "
                      << connected_nodes_cloud_->points.size() << " connected nodes.");
      node_ind = -1;
      dist = 0;
    }
  }

  int KeyposeGraph::GetClosestKeyposeID(const geometry_msgs::Point &point)
  {
    int closest_node_ind = GetClosestNodeInd(point);
    if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())
    {
      return nodes_[closest_node_ind].keypose_id_;
    }
    else
    {
      return -1;
    }
  }

  /**
   * @brief 从关键姿态图中获取与给定点的最近点的位置
   *
   * @param point 源点
   * @return geometry_msgs::Point 最近点的位置
   */
  geometry_msgs::Point KeyposeGraph::GetClosestNodePosition(const geometry_msgs::Point &point)
  {
    int closest_node_ind = GetClosestNodeInd(point);
    if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())
    {
      return nodes_[closest_node_ind].position_;
    }
    else
    {
      geometry_msgs::Point point;
      point.x = 0;
      point.y = 0;
      point.z = 0;
      return point;
    }
  }

  /**
   * @brief 在关键姿态图中找到两个给定点之间的最短路径，并且在路径长度不超过指定的最大路径长度时，可以返回路径的详细信息。
   *
   * @param start_point     用户指定的：起始点
   * @param target_point    用户指定的：终止点
   * @param max_path_length 用户指定的：最大路径长度限制
   * @param get_path        用户指定的：是否需要将找到的路径输出到path
   * @param path            输出的：找到的路径结果
   * @return true           找到
   * @return false          没找到
   */
  bool KeyposeGraph::GetShortestPathWithMaxLength(const geometry_msgs::Point &start_point,
                                                  const geometry_msgs::Point &target_point,
                                                  double max_path_length,
                                                  bool get_path, nav_msgs::Path &path)
  {
    // 1. 检查节点的数量是否小于2，是则直接将起始点和终点作为路径返回
    if (nodes_.size() < 2)
    {
      if (get_path)
      {
        geometry_msgs::PoseStamped start_pose;
        start_pose.pose.position = start_point;
        geometry_msgs::PoseStamped target_pose;
        target_pose.pose.position = target_point;
        path.poses.push_back(start_pose);
        path.poses.push_back(target_pose);
      }
      return misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(start_point, target_point);
    }

    // 2. 如果检查节点的数量是不小于2，首先查找两个离起始点和目标点最近的节点
    int from_idx = 0;
    int to_idx = 0;
    double min_dist_to_start = DBL_MAX;
    double min_dist_to_target = DBL_MAX;
    for (int i = 0; i < nodes_.size(); i++)
    {
      if (allow_vertical_edge_)
      {
        double dist_to_start =
            misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, start_point);
        double dist_to_target =
            misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, target_point);
        if (dist_to_start < min_dist_to_start)
        {
          min_dist_to_start = dist_to_start;
          from_idx = i;
        }
        if (dist_to_target < min_dist_to_target)
        {
          min_dist_to_target = dist_to_target;
          to_idx = i;
        }
      }
      else
      {
        double z_diff_to_start = std::abs(nodes_[i].position_.z - start_point.z);
        double z_diff_to_target = std::abs(nodes_[i].position_.z - target_point.z);
        // TODO: parameterize this
        if (z_diff_to_start < 1.5)
        {
          double xy_dist_to_start =
              misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, start_point);
          if (xy_dist_to_start < min_dist_to_start)
          {
            min_dist_to_start = xy_dist_to_start;
            from_idx = i;
          }
        }
        if (z_diff_to_target < 1.5)
        {
          double xy_dist_to_target =
              misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, target_point);
          if (xy_dist_to_target < min_dist_to_target)
          {
            min_dist_to_target = xy_dist_to_target;
            to_idx = i;
          }
        }
      }
    }

    // 执行 A* 搜索，寻找从起始节点到目标节点的最短路径，并且路径的长度不超过指定的最大路径长度。
    std::vector<geometry_msgs::Point> node_positions;
    for (int i = 0; i < nodes_.size(); i++)
    {
      node_positions.push_back(nodes_[i].position_);
    }
    std::vector<int> path_indices;
    double shortest_dist = DBL_MAX;
    bool found_path = misc_utils_ns::AStarSearchWithMaxPathLength(graph_, dist_, node_positions, from_idx, to_idx,
                                                                  get_path, path_indices, shortest_dist, max_path_length);
    if (found_path && get_path)
    {
      path.poses.clear();
      for (const auto &ind : path_indices)
      {
        geometry_msgs::PoseStamped pose;
        pose.pose.position = nodes_[ind].position_;
        pose.pose.orientation.w = nodes_[ind].keypose_id_;
        pose.pose.orientation.x = ind;
        path.poses.push_back(pose);
      }
    }

    return found_path;
  }

  /**
   * @brief 获取从给定起点到目标点的最短路径
   *
   * @param start_point         起始点
   * @param target_point        终点
   * @param get_path            是否返回找到的最短路径
   * @param path                返回的找到的路径
   * @param use_connected_nodes 指定是否只用连接范围内的node
   * @return double 返回的路径长度
   */
  double KeyposeGraph::GetShortestPath(const geometry_msgs::Point &start_point, const geometry_msgs::Point &target_point,
                                       bool get_path, nav_msgs::Path &path, bool use_connected_nodes)
  {
    // 如果关键姿态图中的节点数量小于2，说明没有足够的节点构成有效的路径，
    // 这时如果需要获取路径，则创建一个路径直接连接起点和目标点，将这两个点作为路径的两个节点，并返回两点之间的欧几里德距离。
    if (nodes_.size() < 2)
    {
      if (get_path)
      {
        geometry_msgs::PoseStamped start_pose;
        start_pose.pose.position = start_point;
        geometry_msgs::PoseStamped target_pose;
        target_pose.pose.position = target_point;
        path.poses.push_back(start_pose);
        path.poses.push_back(target_pose);
      }
      return misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(start_point, target_point);
    }

    int from_idx = 0;
    int to_idx = 0;
    double min_dist_to_start = DBL_MAX;
    double min_dist_to_target = DBL_MAX;
    for (int i = 0; i < nodes_.size(); i++)
    {
      /*
        1.如果只考虑连接范围内的节点，且当前的节点并不是连接范围内的点，就直接pass
      */
      if (use_connected_nodes && !nodes_[i].is_connected_)
      {
        continue;
      }

      /*
        2.找到距离给定的起始点和终点最近的keypose作为规划的起点和终点
      */
      // 如果允许垂直边（allow_vertical_edge_ 为真），则计算节点与起点和目标点之间的欧几里德距离，并选择距离最近的节点作为起点和目标点。
      if (allow_vertical_edge_)
      {
        double dist_to_start =
            misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, start_point);
        double dist_to_target =
            misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, target_point);
        if (dist_to_start < min_dist_to_start)
        {
          min_dist_to_start = dist_to_start;
          from_idx = i;
        }
        if (dist_to_target < min_dist_to_target)
        {
          min_dist_to_target = dist_to_target;
          to_idx = i;
        }
      }
      // 如果不允许垂直边，则在一定高度限制范围内找到起点和目标点
      else
      {
        double z_diff_to_start = std::abs(nodes_[i].position_.z - start_point.z);
        double z_diff_to_target = std::abs(nodes_[i].position_.z - target_point.z);
        // TODO: parameterize this
        if (z_diff_to_start < 1.5)
        {
          double xy_dist_to_start =
              misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, start_point);
          if (xy_dist_to_start < min_dist_to_start)
          {
            min_dist_to_start = xy_dist_to_start;
            from_idx = i;
          }
        }
        if (z_diff_to_target < 1.5)
        {
          double xy_dist_to_target =
              misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(nodes_[i].position_, target_point);
          if (xy_dist_to_target < min_dist_to_target)
          {
            min_dist_to_target = xy_dist_to_target;
            to_idx = i;
          }
        }
      }
    }

    /*
      3.获取所有节点的位置，并利用 A* 搜索算法计算从起点节点到目标节点的最短路径。
    */
    std::vector<geometry_msgs::Point> node_positions;
    for (int i = 0; i < nodes_.size(); i++)
    {
      node_positions.push_back(nodes_[i].position_);
    }
    std::vector<int> path_indices;
    double shortest_dist =
        misc_utils_ns::AStarSearch(graph_, dist_, node_positions, from_idx, to_idx, get_path, path_indices);

    /*
      4.如果需要返回路径，就将得到的路径返回
    */
    if (get_path)
    {
      path.poses.clear();
      for (const auto &ind : path_indices)
      {
        geometry_msgs::PoseStamped pose;
        pose.pose.position = nodes_[ind].position_;
        pose.pose.orientation.w = nodes_[ind].keypose_id_;
        pose.pose.orientation.x = ind;
        path.poses.push_back(pose);
      }
    }

    return shortest_dist;
  }

  /**
   * @brief 返回第一个记录的keypose
   *
   * @return geometry_msgs::Point
   */
  geometry_msgs::Point KeyposeGraph::GetFirstKeyposePosition()
  {
    geometry_msgs::Point point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    for (const auto &node : nodes_)
    {
      if (node.IsKeypose())
      {
        point = node.position_;
        break;
      }
    }
    return point;
  }

  geometry_msgs::Point KeyposeGraph::GetKeyposePosition(int keypose_id)
  {
    geometry_msgs::Point point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    for (const auto &node : nodes_)
    {
      if (node.keypose_id_ == keypose_id)
      {
        point = node.position_;
        break;
      }
    }
    return point;
  }

  void KeyposeGraph::GetKeyposePositions(std::vector<Eigen::Vector3d> &positions)
  {
    positions.clear();
    for (const auto &node : nodes_)
    {
      if (node.IsKeypose())
      {
        Eigen::Vector3d position(node.position_.x, node.position_.y, node.position_.z);
        positions.push_back(position);
      }
    }
  }

  geometry_msgs::Point KeyposeGraph::GetNodePosition(int node_ind)
  {
    geometry_msgs::Point node_position;
    node_position.x = 0;
    node_position.y = 0;
    node_position.z = 0;
    if (node_ind >= 0 && node_ind < nodes_.size())
    {
      node_position = nodes_[node_ind].position_;
    }
    else
    {
      ROS_WARN_STREAM("KeyposeGraph::GetNodePosition: node_ind " << node_ind << " out of bound [0, " << nodes_.size() - 1
                                                                 << "]");
    }
    return node_position;
  }

} // namespace keypose_graph_ns
