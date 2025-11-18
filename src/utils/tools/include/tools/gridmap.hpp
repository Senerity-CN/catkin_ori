#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <map>
#include <tools/config.hpp>
#include <decomp_basis/data_type.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace map_util
{
  using Tmap = std::vector<signed char>;
  template <int Dim>
  class MapUtil
  {
  public:
    /// Simple constructor
    MapUtil() {}
    /// Get map data
    Tmap getMap() { return map_; }

    //
    bool has_map_() { return has_map; }
    /// Get resolution
    decimal_t getRes() { return res_; }
    /// Get dimensions
    Veci<Dim> getDim() { return dim_; }
    /// Get origin
    Vecf<Dim> getOrigin() { return origin_d_; }

    void clearDynamicMap()
    {
      std::fill(dynamic_map_.begin(), dynamic_map_.end(), val_free);
    }
    void setDynamicObs(pcl::PointXYZ pt)
    {
      double coord_x = pt.x;
      double coord_y = pt.y;
      Vec2f pts = Vec2f(coord_x, coord_y);
      Veci<Dim> index2i;
      for (int i = 0; i < Dim; i++)
        // index2i(i) = std::round((pt(i) - origin_d_(i)) / res_);
        index2i(i) = std::round((pts(i) - origin_d_(i)) / res_ - 0.5);
      if (isOutside(index2i))
        return;
      for (int i = -expand_size; i <= expand_size; i++)
        for (int j = -expand_size; j <= expand_size; j++)
        {
          Veci<Dim> temp_index;
          temp_index(0) = index2i(0) + i;
          temp_index(1) = index2i(1) + j;
          if (isOutside(temp_index))
            continue;
          dynamic_map_[getIndex(temp_index)] = val_occ;
        }
    }

    void mergeMaps()
    {
      map_ = static_map_; // 先复制静态地图数据
      for (size_t i = 0; i < dynamic_map_.size(); ++i)
      {
        if (dynamic_map_[i] == val_occ)
        {
          map_[i] = val_occ; // 动态障碍物覆盖静态地图
        }
      }
    }

    void setParam(ConfigPtr config_, ros::NodeHandle &nh)
    {
      res_ = config_->mapRes;
      int buffer_size;
      expand_size = config_->expandSize;
      priv_config_ = config_;
      if (Dim == 3)
      {
        map_size(0) = config_->mapX;
        map_size(1) = config_->mapY;
        map_size(2) = config_->mapZ;
        origin_d_[0] = -map_size(0) / 2;
        origin_d_[1] = -map_size(1) / 2;
        origin_d_[2] = 0;
        dim_(0) = map_size(0) / res_;
        dim_(1) = map_size(1) / res_;
        dim_(2) = map_size(2) / res_;
        buffer_size = dim_(0) * dim_(1) * dim_(2);
      }
      else if (Dim == 2)
      {
        map_size(0) = config_->mapX;
        map_size(1) = config_->mapY;
        origin_d_[0] = -map_size(0) / 2;
        origin_d_[1] = -map_size(1) / 2;
        dim_(0) = map_size(0) / res_;
        dim_(1) = map_size(1) / res_;
        buffer_size = dim_(0) * dim_(1);
      }
      else
      {
        ROS_ERROR("grid map dimensions must be 2 or 3!");
      }
      map_.resize(buffer_size, val_free);
      point_cloud_sub_ = nh.subscribe("/global_map", 10, &MapUtil::MapBuild, this);
    }
    void MapBuild(const sensor_msgs::PointCloud2 &pointcloud_map)
    {
      // ROS_INFO("build map");
      if (has_map)
        return;
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(pointcloud_map, cloud);
      ROS_INFO("Received the point cloud");
      ROS_INFO("map_util is building the map! please wait~");
      if ((int)cloud.points.size() == 0)
        return;
      pcl::PointXYZ pt;
      ROS_INFO("cloud points size=%d\n", (int)cloud.points.size());
      for (int idx = 0; idx < (int)cloud.points.size(); idx++)
      {
        pt = cloud.points[idx];
        setObs(pt);
      }
      has_map = true;
      ROS_WARN("Finish gridmap built");
      static_map_ = map_;
    }
    // map update
    void mapUpdate(const sensor_msgs::PointCloud2 &pointcloud_map)
    {
      // std::fill(map_.begin(), map_.end(), val_free);
      dynamic_map_ = map_;
      clearDynamicMap();
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(pointcloud_map, cloud);
      if ((int)cloud.points.size() == 0)
        return;
      pcl::PointXYZ pt;
      for (int idx = 0; idx < (int)cloud.points.size(); idx++)
      {
        pt = cloud.points[idx];
        setDynamicObs(pt);
      }
      mergeMaps();
    }

    void setObs(pcl::PointXYZ pt)
    {
      if (Dim == 3)
      {
        double coord_x = pt.x;
        double coord_y = pt.y;
        double coord_z = pt.z;
        Vec3f pt = Vec3f(coord_x, coord_y, coord_z);
        Veci<Dim> index3i;
        for (int i = 0; i < Dim; i++)
          index3i(i) = std::round((pt(i) - origin_d_(i)) / res_ - 0.5);
        if (isOutside(index3i))
          return;
        for (int i = -expand_size; i <= expand_size; i++)
          for (int j = -expand_size; j <= expand_size; j++)
            for (int k = -expand_size; k <= expand_size; k++)
            {
              Veci<Dim> temp_index;
              temp_index(0) = index3i(0) + i;
              temp_index(1) = index3i(1) + j;
              temp_index(2) = index3i(2) + k;
              if (isOutside(temp_index))
                continue;
              map_[getIndex(temp_index)] = val_occ;
            }
      }
      else
      {
        double coord_x = pt.x;
        double coord_y = pt.y;
        Vec2f pt = Vec2f(coord_x, coord_y);
        Veci<Dim> index2i;
        for (int i = 0; i < Dim; i++)
          // index2i(i) = std::round((pt(i) - origin_d_(i)) / res_);
          index2i(i) = std::round((pt(i) - origin_d_(i)) / res_ - 0.5);
        if (isOutside(index2i))
          return;
        for (int i = -expand_size; i <= expand_size; i++)
          for (int j = -expand_size; j <= expand_size; j++)
          {
            Veci<Dim> temp_index;
            temp_index(0) = index2i(0) + i;
            temp_index(1) = index2i(1) + j;
            if (isOutside(temp_index))
              continue;
            map_[getIndex(temp_index)] = val_occ;
          }
      }
      return;
    }
    /// Get index of a cell
    int getIndex(const Veci<Dim> &pn)
    {
      return Dim == 2 ? pn(0) + dim_(0) * pn(1) : pn(0) + dim_(0) * pn(1) + dim_(0) * dim_(1) * pn(2);
    }
    /// Check if the given cell is outside of the map in i-the dimension
    bool isOutsideXYZ(const Veci<Dim> &n, int i) { return n(i) < 0 || n(i) >= dim_(i); }
    /// Check if the cell is free by index
    bool isFree(int idx) { return map_[idx] == val_free; }
    /// Check if the cell is unknown by index
    bool isUnknown(int idx) { return map_[idx] == val_unknown; }
    /// Check if the cell is occupied by index
    bool isOccupied(int idx) { return map_[idx] > val_free; }
    // free 0 occ 100 unknow -1
    /// Check if the cell is outside by coordinate
    bool isOutside(const Veci<Dim> &pn)
    {
      for (int i = 0; i < Dim; i++)
        if (pn(i) < 0 || pn(i) >= dim_(i))
          return true;
      return false;
    }
    /// Check if the given cell is free by coordinate
    bool isFree(const Veci<Dim> &pn)
    {
      if (isOutside(pn))
        return false;
      else
        return isFree(getIndex(pn));
    }
    /// Check if the given cell is occupied by coordinate
    bool isOccupied(const Veci<Dim> &pn)
    {
      if (isOutside(pn))
        return true;
      else
        return isOccupied(getIndex(pn));
    }
    /// Check if the given cell is unknown by coordinate
    bool isUnknown(const Veci<Dim> &pn)
    {
      if (isOutside(pn))
        return false;
      return map_[getIndex(pn)] == val_unknown;
    }

    /**
     * @brief Set map
     *
     * @param ori origin position
     * @param dim number of cells in each dimension
     * @param map array of cell values
     * @param res map resolution
     */
    void setMap(const Vecf<Dim> &ori, const Veci<Dim> &dim, const Tmap &map, decimal_t res)
    {
      map_ = map;
      dim_ = dim;
      origin_d_ = ori;
      res_ = res;
    }

    /// Print basic information about the util
    void info()
    {
      Vecf<Dim> range = dim_.template cast<decimal_t>() * res_;
      std::cout << "MapUtil Info ========================== " << std::endl;
      std::cout << "   res: [" << res_ << "]" << std::endl;
      std::cout << "   origin: [" << origin_d_.transpose() << "]" << std::endl;
      std::cout << "   range: [" << range.transpose() << "]" << std::endl;
      std::cout << "   dim: [" << dim_.transpose() << "]" << std::endl;
    };

    /// Float position to discrete cell coordinate
    Veci<Dim> floatToInt(const Vecf<Dim> &pt)
    {
      Veci<Dim> pn;
      for (int i = 0; i < Dim; i++)
        pn(i) = std::round((pt(i) - origin_d_(i)) / res_ - 0.5);
      return pn;
    }
    /// Discrete cell coordinate to float position
    Vecf<Dim> intToFloat(const Veci<Dim> &pn)
    {
      // return pn.template cast<decimal_t>() * res_ + origin_d_;
      return (pn.template cast<decimal_t>() + Vecf<Dim>::Constant(0.5)) * res_ + origin_d_;
    }

    /// Raytrace from float point pt1 to pt2
    vec_Veci<Dim> rayTrace(const Vecf<Dim> &pt1, const Vecf<Dim> &pt2)
    {
      Vecf<Dim> diff = pt2 - pt1;
      decimal_t k = 0.8;
      int max_diff = (diff / res_).template lpNorm<Eigen::Infinity>() / k;
      decimal_t s = 1.0 / max_diff;
      Vecf<Dim> step = diff * s;

      vec_Veci<Dim> pns;
      Veci<Dim> prev_pn = Veci<Dim>::Constant(-1);
      for (int n = 1; n < max_diff; n++)
      {
        Vecf<Dim> pt = pt1 + step * n;
        Veci<Dim> new_pn = floatToInt(pt);
        if (isOutside(new_pn))
          break;
        if (new_pn != prev_pn)
          pns.push_back(new_pn);
        prev_pn = new_pn;
      }
      return pns;
    }

    /// Check if the ray from p1 to p2 is occluded
    bool isBlocked(const Vecf<Dim> &p1, const Vecf<Dim> &p2, int8_t val = 100)
    {
      vec_Veci<Dim> pns = rayTrace(p1, p2);
      for (const auto &pn : pns)
      {
        if (map_[getIndex(pn)] >= val)
          return true;
      }
      return false;
    }

    /// Get occupied voxels
    vec_Vecf<Dim> getCloud()
    {
      vec_Vecf<Dim> cloud;
      Veci<Dim> n;
      if (Dim == 3)
      {
        for (n(0) = 0; n(0) < dim_(0); n(0)++)
        {
          for (n(1) = 0; n(1) < dim_(1); n(1)++)
          {
            for (n(2) = 0; n(2) < dim_(2); n(2)++)
            {
              if (isOccupied(getIndex(n)))
                cloud.push_back(intToFloat(n));
            }
          }
        }
      }
      else if (Dim == 2)
      {
        for (n(0) = 0; n(0) < dim_(0); n(0)++)
        {
          for (n(1) = 0; n(1) < dim_(1); n(1)++)
          {
            if (isOccupied(getIndex(n)))
              cloud.push_back(intToFloat(n));
          }
        }
      }

      return cloud;
    }

    /// Get free voxels
    vec_Vecf<Dim> getFreeCloud()
    {
      vec_Vecf<Dim> cloud;
      Veci<Dim> n;
      if (Dim == 3)
      {
        for (n(0) = 0; n(0) < dim_(0); n(0)++)
        {
          for (n(1) = 0; n(1) < dim_(1); n(1)++)
          {
            for (n(2) = 0; n(2) < dim_(2); n(2)++)
            {
              if (isFree(getIndex(n)))
                cloud.push_back(intToFloat(n));
            }
          }
        }
      }
      else if (Dim == 2)
      {
        for (n(0) = 0; n(0) < dim_(0); n(0)++)
        {
          for (n(1) = 0; n(1) < dim_(1); n(1)++)
          {
            if (isFree(getIndex(n)))
              cloud.push_back(intToFloat(n));
          }
        }
      }

      return cloud;
    }

    /// Get unknown voxels
    vec_Vecf<Dim> getUnknownCloud()
    {
      vec_Vecf<Dim> cloud;
      Veci<Dim> n;
      if (Dim == 3)
      {
        for (n(0) = 0; n(0) < dim_(0); n(0)++)
        {
          for (n(1) = 0; n(1) < dim_(1); n(1)++)
          {
            for (n(2) = 0; n(2) < dim_(2); n(2)++)
            {
              if (isUnknown(getIndex(n)))
                cloud.push_back(intToFloat(n));
            }
          }
        }
      }
      else if (Dim == 2)
      {
        for (n(0) = 0; n(0) < dim_(0); n(0)++)
        {
          for (n(1) = 0; n(1) < dim_(1); n(1)++)
          {
            if (isUnknown(getIndex(n)))
              cloud.push_back(intToFloat(n));
          }
        }
      }

      return cloud;
    }

    /// Dilate occupied cells
    void dilate(const vec_Veci<Dim> &dilate_neighbor)
    {
      Tmap map = map_;
      Veci<Dim> n = Veci<Dim>::Zero();
      if (Dim == 3)
      {
        for (n(0) = 0; n(0) < dim_(0); n(0)++)
        {
          for (n(1) = 0; n(1) < dim_(1); n(1)++)
          {
            for (n(2) = 0; n(2) < dim_(2); n(2)++)
            {
              if (isOccupied(getIndex(n)))
              {
                for (const auto &it : dilate_neighbor)
                {
                  if (!isOutside(n + it))
                    map[getIndex(n + it)] = val_occ;
                }
              }
            }
          }
        }
      }
      else if (Dim == 2)
      {
        for (n(0) = 0; n(0) < dim_(0); n(0)++)
        {
          for (n(1) = 0; n(1) < dim_(1); n(1)++)
          {
            if (isOccupied(getIndex(n)))
            {
              for (const auto &it : dilate_neighbor)
              {
                if (!isOutside(n + it))
                  map[getIndex(n + it)] = val_occ;
              }
            }
          }
        }
      }

      map_ = map;
    }

    /// Free unknown voxels
    void freeUnknown()
    {
      Veci<Dim> n;
      if (Dim == 3)
      {
        for (n(0) = 0; n(0) < dim_(0); n(0)++)
        {
          for (n(1) = 0; n(1) < dim_(1); n(1)++)
          {
            for (n(2) = 0; n(2) < dim_(2); n(2)++)
            {
              if (isUnknown(getIndex(n)))
                map_[getIndex(n)] = val_free;
            }
          }
        }
      }
      else if (Dim == 2)
      {
        for (n(0) = 0; n(0) < dim_(0); n(0)++)
        {
          for (n(1) = 0; n(1) < dim_(1); n(1)++)
          {
            if (isUnknown(getIndex(n)))
              map_[getIndex(n)] = val_free;
          }
        }
      }
    }
    void CheckIfCollisionUsingPosAndYaw(const Eigen::Vector3d &state, bool *res)
    {
      if (Dim == 3)
      {
        ROS_ERROR("CheckIfCollisionUsingPosAndYaw Only works with Dimension = 2!");
        return;
      }
      else
      {
        Eigen::Matrix2d rotM;
        double yaw = state[2];
        rotM << cos(yaw), -sin(yaw),
            sin(yaw), cos(yaw);
        Eigen::Vector2d p = state.head(2);
        for (int i = 0; i < priv_config_->conpts.size(); i++)
        {
          Eigen::Vector2d rep1 = priv_config_->conpts[i];
          Eigen::Vector2d rep2 = priv_config_->conpts[(i + 1) % priv_config_->conpts.size()];
          Eigen::Vector2d pt1 = p + rotM * rep1;
          Eigen::Vector2d pt2 = p + rotM * rep2;
          if (isBlocked(pt1, pt2))
          {
            *res = true;
            return;
          }
        }
      }
      *res = false;
      return;
    }

    void CheckIfCollisionUsingPos(const Eigen::Vector3d &state, bool *res)
    {
      Eigen::Vector2d pt = state.head(2);
      Veci<Dim> index2i;
      for (int i = 0; i < Dim; i++)
        index2i(i) = std::round((pt(i) - origin_d_(i)) / res_ - 0.5);
      if (isOccupied(getIndex(index2i)))
      {
        *res = true;
        return;
      }

      *res = false;
      return;
    }
    /// Map entity
    Tmap map_;         // final_map
    Tmap dynamic_map_; // dynamic map info
    Tmap static_map_;  // static map info
    std::string world_frame_id;

  protected:
    /// Resolution
    decimal_t res_;
    /// Origin, float type
    Vecf<Dim> origin_d_;
    /// Dimension, int type
    Veci<Dim> dim_;
    Vecf<Dim> map_size;
    /// Assume occupied cell has value 100
    int8_t val_occ = 100;
    /// Assume free cell has value 0
    int8_t val_free = 0;
    /// Assume unknown cell has value -1
    int8_t val_unknown = -1;
    /// inflate size
    int expand_size = 1;
    bool has_map = false;
    ros::Subscriber point_cloud_sub_;
    ConfigPtr priv_config_;
  };
  typedef MapUtil<2> OccMapUtil;
  typedef MapUtil<3> VoxelMapUtil;
}

#endif
