#ifndef CPOM_TYPES_H
#define CPOM_TYPES_H

#include <tuple>

#include "bvh/v2/vec.h"
#include "bvh/v2/tri.h"
#include "bvh/v2/bbox.h"
#include "bvh/v2/node.h"
#include "bvh/v2/bvh.h"

using Scalar   = double;
using Index    = size_t;
using Vec3     = bvh::v2::Vec<Scalar, 3>;
using Tri      = bvh::v2::Tri<Scalar, 3>;
using BBox     = bvh::v2::BBox<Scalar, 3>;
using Node     = bvh::v2::Node<Scalar, 3>;
using Bvh      = bvh::v2::Bvh<Node>;
using Location = std::tuple<Vec3, Index, Vec3>;

#endif

