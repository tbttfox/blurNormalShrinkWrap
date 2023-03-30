#include <bvh/v2/bvh.h>
#include <bvh/v2/node.h>
#include <bvh/v2/default_builder.h>
#include <bvh/v2/thread_pool.h>
#include <bvh/v2/executor.h>
#include <bvh/v2/stack.h>
#include <bvh/v2/dist_point_triangle.h>
#include <bvh/v2/vec.h>
#include <bvh/v2/tri.h>

#include <vector>

using Scalar   = double;
using Index    = size_t;
using Vec3     = bvh::v2::Vec<Scalar, 3>;
using Tri      = bvh::v2::Tri<Scalar, 3>;
using BBox     = bvh::v2::BBox<Scalar, 3>;
using Node     = bvh::v2::Node<Scalar, 3>;
using Bvh      = bvh::v2::Bvh<Node>;

Bvh build_bvh(
    const std::vector<Tri>& tris,
    std::vector<BBox>& bboxes,
    std::vector<Vec3>& centers,
    std::vector<Vec3>& normals
);

Vec3 get_closest(
    const Bvh& bvh,
    const std::vector<Tri>& tris,
    const std::vector<BBox>& bboxes,
    const std::vector<Vec3>& centers,
    const std::vector<Vec3>& normals,

    Vec3 qp,
    Vec3 norm,
    Scalar angle
);

