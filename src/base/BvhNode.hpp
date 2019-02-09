#pragma once

#include "rtutil.hpp"
#include "util.hpp"
#include "filesaves.hpp"
#include "RTTriangle.hpp"

#include <memory>


namespace FW {
class Bvh;

// bounding volume hierarchy tree node; stores the bounding box, interval of leafs covered and children.
// the only real functionality of BvhNode is the save/load feature.
struct BvhNode : noncopyable {
    AABB bb;
    size_t startPrim, endPrim; // [start, end)
    std::unique_ptr<BvhNode> left;
    std::unique_ptr<BvhNode> right;

    BvhNode() :
        bb(),
        startPrim(0), endPrim(0)
    {}

    BvhNode(size_t depth, size_t start, size_t end);

    BvhNode(Loader& is, size_t depth = 0);

    ~BvhNode() {}

    inline bool hasChildren() const {
        return !!left;
    }

    void save(Saver& os);
};

}
