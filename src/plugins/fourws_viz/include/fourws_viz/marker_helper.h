#pragma once

#include <string>

#include <visualization_msgs/MarkerArray.h>

inline geometry_msgs::Point createMarkerPosition(double x, double y, double z)
{
  geometry_msgs::Point point;

  point.x = x;
  point.y = y;
  point.z = z;

  return point;
}

inline geometry_msgs::Quaternion createMarkerOrientation(double x, double y, double z, double w)
{
  geometry_msgs::Quaternion quaternion;

  quaternion.x = x;
  quaternion.y = y;
  quaternion.z = z;
  quaternion.w = w;

  return quaternion;
}

inline geometry_msgs::Vector3 createMarkerScale(double x, double y, double z)
{
  geometry_msgs::Vector3 scale;

  scale.x = x;
  scale.y = y;
  scale.z = z;

  return scale;
}

inline std_msgs::ColorRGBA createMarkerColor(float r, float g, float b, float a)
{
  std_msgs::ColorRGBA color;

  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;

  return color;
}

inline visualization_msgs::Marker createDefaultMarker(
  const std::string & frame_id, const std::string & ns, const int32_t id, const int32_t type,
  const geometry_msgs::Vector3 & scale, const std_msgs::ColorRGBA & color)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(0);

  marker.pose.position = createMarkerPosition(0.0, 0.0, 0.0);
  marker.pose.orientation = createMarkerOrientation(0.0, 0.0, 0.0, 1.0);
  marker.scale = scale;
  marker.color = color;
  marker.frame_locked = true;

  return marker;
}

inline void appendMarkerArray(
  const visualization_msgs::MarkerArray & additional_marker_array,
  visualization_msgs::MarkerArray * marker_array)
{
  for (const auto & marker : additional_marker_array.markers) {
    marker_array->markers.push_back(marker);
  }
}
