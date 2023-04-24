#include <cmath>
#include <numbers>

#include "cpom_normal.h"

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
            normals[i] = tris[i].get_normal();
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

    auto distFunc = [&](const Node& curNode) {
        return length_squared(curNode.get_bbox().vec_to_closest(qp));
    };

    auto leafFunc = [&](size_t begin, size_t end) {
        for (Index i = begin; i < end; ++i) {
            auto triIdx = bvh.prim_ids[i];

            // check if the normal angle is outside of tolerance
            if (bvh::v2::dot(norm, normals[triIdx]) < cosTol) continue;

            auto [prim_point, prim_bary] = bvh::v2::closest_point_tri(qp, tris[triIdx]);
            auto prim_dist2 = bvh::v2::length_squared<Scalar, 3>(prim_point - qp);
            if (prim_dist2 < best_dist2) {

                best_prim_idx = triIdx;
                best_point = prim_point;
                best_bary = prim_bary;
                best_dist2 = prim_dist2;
            }
        }
        return best_dist2;
    };

    bvh::v2::SmallStack<Bvh::Index, stack_size> nodeStack;
    bvh.closest_point(bvh.get_root(), nodeStack, leafFunc, distFunc, best_dist2);
    //std::cout << "Closest TriIdx " << best_prim_idx << std::endl;
    //std::cout << "Closest Bary" << best_bary[0] << ", " << best_bary[1] << ", " << best_bary[2] << std::endl;
    //std::cout << "Closest Point" << best_point[0] << ", " << best_point[1] << ", " << best_point[2] << std::endl;
    return std::make_tuple(best_point, best_prim_idx, best_bary);
}

