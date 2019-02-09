#pragma once


#include "BvhNode.hpp"


#include <vector>
#include <iostream>
#include <memory>


namespace FW {


	class Bvh {
	public:

		Bvh();
		Bvh(std::istream& is);
		Bvh(std::vector<RTTriangle>* triangles, SplitMode splitMode);

		void constructTree(BvhNode * n, int start, int end, int depth);

		// move assignment for performance
		Bvh& operator=(Bvh&& other) {
			mode_ = other.mode_;
			std::swap(rootNode_, other.rootNode_);
			std::swap(indices_, other.indices_);
			return *this;
		}

		BvhNode&			root() { return *rootNode_; }
		const BvhNode&		root() const { return *rootNode_; }

		void				save(std::ostream& os);

		uint32_t			getIndex(uint32_t index) const { return indices_[index]; }

		//std::vector<uint32_t>* idx;
		std::pair<AABB, AABB> calculateBB(int start, int end);
		int sortVertices(int dim, int start, int end);

	private:


		SplitMode						mode_;
		std::unique_ptr<BvhNode>		rootNode_;
		std::vector<RTTriangle>*        triangles_;
		std::vector<uint32_t>			indices_; // triangle index list that will be sorted during BVH construction
	};


}