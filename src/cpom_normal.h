#ifndef CPOM_NORMAL_H
#define CPOM_NORMAL_H

#include <vector>
#include <cmath>
#include <numbers>
#include <tuple>

#include "bvh/v2/bvh.h"
#include "bvh/v2/vec.h"
#include "bvh/v2/tri.h"
#include "bvh/v2/bbox.h"

#include "cpom_types.h"
#include "dist_point_triangle.h"

BVH_ALWAYS_INLINE Vec3 get_normal(const Tri &tri);

template <typename T, int N>
BVH_ALWAYS_INLINE bvh::v2::Vec<T, N> vec_to_closest(const BBox& bbox, const bvh::v2::Vec<T, N>& p);

Bvh build_bvh(
    const std::vector<Tri>& tris,
    std::vector<BBox>& bboxes,
    std::vector<Vec3>& centers,
    std::vector<Vec3>& normals
);

Location get_closest(
    const Bvh& bvh,
    const std::vector<Tri>& tris,
    const std::vector<BBox>& bboxes,
    const std::vector<Vec3>& centers,
    const std::vector<Vec3>& normals,

    Vec3 qp,
    Vec3 norm,
    Scalar angle
);

#endif
