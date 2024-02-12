#include <transform.hpp>

Transform::Transform(ViewPoint &viewpoint)
{
    operator=(viewpoint);
}

void Transform::operator=(ViewPoint &viewpoint)
{
    view_point = viewpoint;
    Eigen::Translation3f translation(view_point.x, view_point.y, view_point.z);
    Eigen::Quaternionf rotation(view_point.qw, view_point.qx, view_point.qy, view_point.qz);
    transform_mat = translation * rotation;
}

template <>
void Transform::transform(pcl::PointXYZRGB &src, pcl::PointXYZRGB &dst) const
{
    dst = pcl::transformPoint(src, transform_mat);
}

template <>
void Transform::transform(pcl::PointXYZ &src, pcl::PointXYZ &dst) const
{
    dst = pcl::transformPoint(src, transform_mat);
}

Transform::~Transform()
{
}