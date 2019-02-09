
#include "Bvh.hpp"
#include "filesaves.hpp"

#include <algorithm>
#include <iostream>

using namespace std;


namespace FW {

	const uint32_t MAX_TRIS = 10;

	class TrigComparator
	{
	public:
		int dim;
		std::vector<RTTriangle>* triangles_;
		TrigComparator(int dim, std::vector<RTTriangle>* triangles) : dim(dim), triangles_(triangles) {};
		bool operator ()(int a, int b)
		{
			RTTriangle t1 = (*triangles_)[a];
			RTTriangle t2 = (*triangles_)[b];
			return t1.centroid()[dim] < t2.centroid()[dim];
		}
	};

	Bvh::Bvh() { }

	Bvh::Bvh(std::vector<RTTriangle>* triangles, SplitMode splitMode) {
		triangles_ = triangles;
		indices_ = std::vector<uint32_t>(triangles->size());
		cout << "In total there are " << triangles->size() << " triangles" << endl;
		for (uint32_t i = 0; i < indices_.size(); i++)
			indices_[i] = i;
		mode_ = splitMode;
		BvhNode* n = new BvhNode(0, 0, indices_.size());
		rootNode_ = std::unique_ptr<BvhNode>(n);
		constructTree(n, 0, indices_.size(), 1);
		cout << "We are done constructing the whole BVH" << endl;
	}

	std::pair<AABB, AABB> Bvh::calculateBB(int start, int end) {
		Vec3f ma = Vec3f();
		Vec3f mi = Vec3f();
		Vec3f mic = Vec3f();
		Vec3f mac = Vec3f();
		for (int i = start; i < end; i++) {
			RTTriangle trig = (*triangles_)[getIndex(i)];
			Vec3f maxx = trig.max();
			Vec3f minn = trig.min();
			for (uint32_t i = 0; i < 3; i++) {
				ma[i] = std::max(maxx[i], ma[i]);
				mi[i] = std::min(minn[i], mi[i]);
				mac[i] = std::max(trig.centroid()[i], mac[i]);
				mic[i] = std::max(trig.centroid()[i], mic[i]);
			}
		}
		return std::make_pair(AABB(mi, ma), AABB(mic, mac));
	}



	int Bvh::sortVertices(int dim, int start, int end) {
		std::sort(indices_.begin() + start, indices_.begin() + end, TrigComparator(dim, triangles_));
		return (end - start) / 2 + start;
	}


	void Bvh::constructTree(BvhNode* n, int start, int end, int depth) {
		std::pair<AABB, AABB> bbs = calculateBB(start, end);
		n->bb = bbs.first;
		cout << "BB has " << (end - start) << " triangles and is " << n->bb << endl;
		n->left = 0;
		n->right = 0;

		// Find the max dim
		uint32_t maxdim;
		F32 maxval = -1.0;
		Vec3f dims = bbs.second.max - bbs.second.min;
		for (uint32_t i = 0; i < 3; i++) {
			if (dims[i] > maxval) {
				maxval = dims[i];
				maxdim = i;
			}
		}
		int middle = sortVertices(maxdim, start, end);

		if (end - start > MAX_TRIS) {
			BvhNode* left = new BvhNode(depth, start, middle);
			BvhNode* right = new BvhNode(depth, middle, end);
			n->left = std::unique_ptr<BvhNode>(left);
			n->right = std::unique_ptr<BvhNode>(right);
			constructTree(left, start, middle, depth + 1);
			constructTree(right, middle, end, depth + 1);
		}
	}



	// reconstruct from a file
	Bvh::Bvh(std::istream& is)
	{
		// Load file header.
		fileload(is, mode_);
		Statusbar nodeInfo("Loading nodes", 0);
		Loader loader(is, nodeInfo);

		// Load elements.
		{
			size_t size;
			fileload(is, size);

			for (size_t i = 0; i < size; ++i) {
				uint32_t idx;
				loader(idx);
				indices_.push_back(idx);
			}
		}

		// Load the rest.
		rootNode_.reset(new BvhNode(loader));
	}

	void Bvh::save(std::ostream& os) {
		// Save file header.
		filesave(os, mode_);
		Statusbar nodeInfo("Saving nodes", 0);
		Saver saver(os, nodeInfo);

		// Save elements.
		{
			filesave(os, (size_t)indices_.size());

			for (auto& i : indices_) {
				saver(i);
			}
		}

		// Save the rest.
		rootNode_->save(saver);
	}

} // namespace FW
