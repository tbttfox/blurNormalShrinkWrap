#ifndef BVH_V2_TRI_DIST_H
#define BVH_V2_TRI_DIST_H

#include "bvh/v2/utils.h"
#include "bvh/v2/tri.h"
#include "bvh/v2/vec.h"

#include <tuple>

// This was taken from https://github.com/embree/embree/blob/master/tutorials/common/math/closest_point.h
// licensed under apache 2.0

// I don't actually know if this works in anything other than 3d
// but I'm templating it that way becuase that's MY only usecase
// If anybody else needs something fancier ... sorry, that's on you :-D
template <typename T, size_t N>
::std::tuple<bvh::v2::Vec<T, N>, bvh::v2::Vec<T, 3>> closest_point_tri(bvh::v2::Vec<T, N> const& p, bvh::v2::Tri<T, N> const& tri)
{
    const T zero = static_cast<T>(0);
    const T one = static_cast<T>(1);

    bvh::v2::Vec<T, 3> bary(zero);

    const bvh::v2::Vec<T, N> ab = tri.p1 - tri.p0;
    const bvh::v2::Vec<T, N> ac = tri.p2 - tri.p0;
    const bvh::v2::Vec<T, N> ap = p - tri.p0;

    const T d1 = dot(ab, ap);
    const T d2 = dot(ac, ap);
    if (d1 <= zero && d2 <= zero) {
        bary[0] = one;
        return ::std::make_tuple(tri.p0, bary);
    }

    const bvh::v2::Vec<T, N> bp = p - tri.p1;
    const T d3 = dot(ab, bp);
    const T d4 = dot(ac, bp);
    if (d3 >= zero && d4 <= d3) {
        bary[1] = one;
        return ::std::make_tuple(tri.p1, bary);
    }

    const bvh::v2::Vec<T, N> cp = p - tri.p2;
    const T d5 = dot(ab, cp);
    const T d6 = dot(ac, cp);
    if (d6 >= zero && d5 <= d6) {
        bary[2] = one;
        return ::std::make_tuple(tri.p2, bary);
    }

    const T vc = d1 * d4 - d3 * d2;
    if (vc <= zero && d1 >= zero && d3 <= zero)
    {
        const T v = d1 / (d1 - d3);
        bary[1] = v;
        bary[0] = one - v;
        return ::std::make_tuple(tri.p0 + v * ab, bary);
    }

    const T vb = d5 * d2 - d1 * d6;
    if (vb <= zero && d2 >= zero && d6 <= zero)
    {
        const T v = d2 / (d2 - d6);
        bary[2] = v;
        bary[0] = one - v;
        return ::std::make_tuple(tri.p0 + v * ac, bary);
    }

    const T va = d3 * d6 - d5 * d4;
    if (va <= zero && (d4 - d3) >= zero && (d5 - d6) >= zero)
    {
        const T v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        bary[2] = v;
        bary[1] = one - v;
        return ::std::make_tuple(tri.p1 + v * (tri.p2 - tri.p1), bary);
    }

    const T denom = one / (va + vb + vc);
    // barycentric coords are u, v, w
    const T v = vb * denom;
    const T w = vc * denom;

    bary[0] = one - v - w;
    bary[1] = v;
    bary[2] = w;
    return ::std::make_tuple(tri.p0 + v * ab + w * ac, bary);
}

#endif
