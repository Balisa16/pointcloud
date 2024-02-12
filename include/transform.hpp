#pragma once

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <types.hpp>

class Transform
{
public:
    Transform(ViewPoint &viewpoint);
    void operator=(ViewPoint &viewpoint);

    template <typename T>
    void transform(T &src, T &dst) const;
    ~Transform();

private:
    ViewPoint view_point;
    Eigen::Affine3f transform_mat;
};