#pragma once

#define PCL_NO_PRECOMPILE

#include <cmath>
#include <cstdint>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/register_point_struct.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/impl/crop_box.hpp>

#include <pcl/filters/filter_indices.h>
#include <pcl/filters/impl/filter_indices.hpp>
struct EIGEN_ALIGN16 PointSick
{
    PCL_ADD_POINT4D;

    float intensity;
    float range;
    float azimuth;
    float elevation;
    std::uint32_t t;
    float curvature;
    std::uint32_t lidar_sec;
    std::uint32_t lidar_nsec;
    std::int8_t ring;
    std::int8_t layer;
    std::int8_t echo;
    std::int8_t reflector;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointSick,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, i)
    (float, range, range)
    (float, elevation, elevation)
    (std::uint32_t, t, t)
    (float, curvature, ts)
    (std::uint32_t, lidar_sec, lidar_sec)
    (std::uint32_t, lidar_nsec, lidar_nsec)
    (std::int8_t, ring, ring)
    (std::int8_t, layer, layer)
    (std::int8_t, echo, echo)
    (std::int8_t, reflector, reflector)
)

template <class T>
bool float_eq(const T a, const T b, const T eps = 10e-6)
{
  return std::fabs(a - b) < eps;
}

struct AWPointXYZIRCAEDT
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};   // adds x,y,z,padding + Eigen map helpers

  std::uint8_t intensity{0U};
  std::uint8_t return_type{0U};
  std::uint16_t channel{0U};

  float azimuth{0.0F};
  float elevation{0.0F};
  float distance{0.0F};

  std::uint32_t time_stamp{0U};

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  friend bool operator==(const AWPointXYZIRCAEDT & p1,
                         const PointSick & p2) noexcept
  {
    return float_eq<float>(p1.x, p2.x) &&
           float_eq<float>(p1.y, p2.y) &&
           float_eq<float>(p1.z, p2.z) &&
           p1.intensity == p2.intensity &&
           p1.return_type == p2.echo &&
           p1.channel == p2.ring &&
           float_eq<float>(p1.azimuth, p2.azimuth) &&
           float_eq<float>(p1.elevation, p2.elevation) &&
           float_eq<float>(p1.distance, p2.range) &&
           p1.time_stamp == p2.t;
  }
   friend bool operator==(const PointSick & p1,
                         const AWPointXYZIRCAEDT & p2) noexcept
  {
    return float_eq<float>(p1.x, p2.x) &&
           float_eq<float>(p1.y, p2.y) &&
           float_eq<float>(p1.z, p2.z) &&
           p1.intensity == p2.intensity &&
           p1.echo == p2.return_type &&
           p1.ring == p2.channel &&
           float_eq<float>(p1.azimuth, p2.azimuth) &&
           float_eq<float>(p1.elevation, p2.elevation) &&
           float_eq<float>(p1.range, p2.distance) &&
           p1.t == p2.time_stamp &&
           p1.curvature == static_cast<double>(p2.time_stamp) * 1e-9;
  }
   friend bool operator==(const AWPointXYZIRCAEDT & p1,
                         const AWPointXYZIRCAEDT & p2) noexcept
  {
    return float_eq<float>(p1.x, p2.x) &&
           float_eq<float>(p1.y, p2.y) &&
           float_eq<float>(p1.z, p2.z) &&
           p1.intensity == p2.intensity &&
           p1.return_type == p2.return_type &&
           p1.channel == p2.channel &&
           float_eq<float>(p1.azimuth, p2.azimuth) &&
           float_eq<float>(p1.elevation, p2.elevation) &&
           float_eq<float>(p1.distance, p2.distance) &&
           p1.time_stamp == p2.time_stamp;
  }
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
  AWPointXYZIRCAEDT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (std::uint8_t, intensity, intensity)
  (std::uint8_t, return_type, return_type)
  (std::uint16_t, channel, channel)
  (float, azimuth, azimuth)
  (float, elevation, elevation)
  (float, distance, distance)
  (std::uint32_t, time_stamp, time_stamp)
)
using AWPoint  = AWPointXYZIRCAEDT;
using PCLPoint = PointSick;

inline pcl::PointCloud<PCLPoint>::Ptr aw_to_pcl(
    const pcl::PointCloud<AWPoint>::ConstPtr& in)
{
    pcl::PointCloud<PCLPoint>::Ptr out(new pcl::PointCloud<PCLPoint>);

    out->resize(in->size());

    for (size_t i = 0; i < in->size(); ++i)
    {
        const auto& src = (*in)[i];
        auto& dst = (*out)[i];

        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;

        dst.intensity  = src.intensity;
        dst.echo = src.return_type;
        dst.ring    = src.channel;

        dst.azimuth    = src.azimuth;
        dst.elevation  = src.elevation;
        dst.range   = src.distance;

        dst.t = src.time_stamp;
        dst.curvature = static_cast<double>(src.time_stamp) * 1e-9;
    }

    out->width  = in->width;
    out->height = in->height;
    out->is_dense = in->is_dense;

    return out;
}
inline pcl::PointCloud<AWPoint>::Ptr pcl_to_aw(
    const pcl::PointCloud<PCLPoint>::ConstPtr& in)
{
    pcl::PointCloud<AWPoint>::Ptr out(new pcl::PointCloud<AWPoint>);

    out->resize(in->size());

    for (size_t i = 0; i < in->size(); ++i)
    {
        const auto& src = (*in)[i];
        auto& dst = (*out)[i];

        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;

        dst.intensity  = src.intensity;
        dst.return_type = src.echo;
        dst.channel    = src.ring;

        dst.azimuth    = src.azimuth;
        dst.elevation  = src.elevation;
        dst.distance   = src.range;

        dst.time_stamp = src.t;
    }

    out->width  = in->width;
    out->height = in->height;
    out->is_dense = in->is_dense;

    return out;
}
typedef PointSick PointType;
using PointCloudType = pcl::PointCloud<PointType>;