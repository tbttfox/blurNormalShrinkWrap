#include "bvh/v2/bvh.h"
#include "bvh/v2/node.h"
#include "bvh/v2/default_builder.h"
#include "bvh/v2/thread_pool.h"
#include "bvh/v2/executor.h"
#include "bvh/v2/stack.h"
#include "bvh/v2/vec.h"
#include "bvh/v2/tri.h"

#include <vector>
#include <cmath>
#include <numbers>

#include "dist_point_triangle.h"


using Scalar   = double;
using Index    = size_t;
using Vec3     = bvh::v2::Vec<Scalar, 3>;
using Tri      = bvh::v2::Tri<Scalar, 3>;
using BBox     = bvh::v2::BBox<Scalar, 3>;
using Node     = bvh::v2::Node<Scalar, 3>;
using Bvh      = bvh::v2::Bvh<Node>;
using Location = std::tuple<Vec3, Index, Vec3>;

BVH_ALWAYS_INLINE Vec3 get_normal(const Tri &tri) {
    return normalize(cross(tri.p0 - tri.p1, tri.p2 - tri.p0));
}

template <typename T, int N>
BVH_ALWAYS_INLINE bvh::v2::Vec<T, N> vec_to_closest(const BBox& bbox, const bvh::v2::Vec<T, N>& p) {
    bvh::v2::Vec<T, N> ret;
    static_for<0, N>([&] (size_t i) {
        ret[i] = robust_max<T>(robust_max<T>(bbox.min[i] - p[i], p[i] - bbox.max[i]), 0);
    });
    return ret;
}

Bvh build_bvh(
    const std::vector<Tri>& tris,
    std::vector<BBox>& bboxes,
    std::vector<Vec3>& centers,
    std::vector<Vec3>& normals
) {
    bvh::v2::ThreadPool thread_pool;
    bvh::v2::ParallelExecutor executor(thread_pool);

    // Get triangle centers and bounding boxes (required for BVH builder)
    bboxes.resize(tris.size());
    centers.resize(tris.size());
    normals.resize(tris.size());
    executor.for_each(0, tris.size(), [&] (size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            bboxes[i]  = tris[i].get_bbox();
            centers[i] = tris[i].get_center();
            normals[i] = get_normal(tris[i]);
        }
    });

    typename bvh::v2::DefaultBuilder<Node>::Config config;
    config.quality = bvh::v2::DefaultBuilder<Node>::Quality::High;
    return bvh::v2::DefaultBuilder<Node>::build(thread_pool, bboxes, centers, config);
}

Location get_closest(
    const Bvh& bvh,
    const std::vector<Tri>& tris,
    const std::vector<BBox>& bboxes,
    const std::vector<Vec3>& centers,
    const std::vector<Vec3>& normals,

    Vec3 qp,
    Vec3 norm,
    Scalar angle
){
    static constexpr size_t invalid_id = std::numeric_limits<size_t>::max();
    static constexpr size_t stack_size = 64;

    Scalar cosTol = (angle >= std::numbers::pi) ? -2.0 : cos(angle);

    Scalar best_dist2 = std::numeric_limits<Scalar>::max();
    auto best_prim_idx = invalid_id;
    Vec3 best_point(0), best_bary(0);


    auto innerFunc = [&](const Node& leftNode, const Node& rightNode) {
        auto left_vec = vec_to_closest(leftNode.get_bbox(), qp);
        auto left_dist2 = dot(left_vec, left_vec);

        auto right_vec = vec_to_closest(rightNode.get_bbox(), qp);
        auto right_dist2 = dot(right_vec, right_vec);

        return std::make_tuple(
            left_dist2 < best_dist2,
            right_dist2 < best_dist2,
            left_dist2 < right_dist2
        );
    };

    auto leafFunc = [&](size_t begin, size_t end) {
        for (Index i = begin; i < end; ++i) {
            auto triIdx = bvh.prim_ids[i];

            // check if the normal angle is outside of tolerance
            if (bvh::v2::dot(norm, normals[triIdx]) < cosTol) continue;

            auto [prim_point, prim_bary] = bvh::v2::closest_point_tri(qp, tris[triIdx]);
            auto prim_vec = prim_point - qp;
            auto prim_dist2 = dot(prim_vec, prim_vec);
            if (prim_dist2 < best_dist2) {
                best_prim_idx = triIdx;
                best_point = prim_point;
                best_bary = prim_bary;
                best_dist2 = prim_dist2;
            }
        }
	return true;
    };

    bvh::v2::SmallStack<Bvh::Index, stack_size> nodeStack;
    
    auto root = bvh.get_root();

    bvh.traverse_top_down<false>(root, nodeStack, leafFunc, innerFunc);

    //std::cout << "Closest TriIdx " << best_prim_idx << std::endl;
    //std::cout << "Closest Bary" << best_bary[0] << ", " << best_bary[1] << ", " << best_bary[2] << std::endl;
    //std::cout << "Closest Point" << best_point[0] << ", " << best_point[1] << ", " << best_point[2] << std::endl;
    return std::make_tuple(best_point, best_prim_idx, best_bary);
}
