
#include "Bvh.hpp"
#include "filesaves.hpp"

#include <algorithm>
#include <iostream>

using namespace std;


namespace FW {

	const uint32_t TRI_LIMIT = 64;

	Bvh::Bvh() { }

	std::pair<AABB, AABB> Bvh::calculateBB(int start, int end) {
		Vec3f ma = Vec3f();
		Vec3f mi = Vec3f();
		Vec3f mic = Vec3f();
		Vec3f mac = Vec3f();
		for (int i = start; i < end; i++) {
			RTTriangle trig = (*triangles_)[getIndex(i)];
			Vec3f maxx = trig.max();
			Vec3f minn = trig.min();
			for (uint32_t j = 0; j < 3; j++) {
				ma[j] = std::max(maxx[j], ma[j]);
				mi[j] = std::min(minn[j], mi[j]);
				mac[j] = std::max(trig.centroid()[j], mac[j]);
				mic[j] = std::max(trig.centroid()[j], mic[j]);
			}
		}
		return std::make_pair(AABB(mi, ma), AABB(mic, mac));
	}

	Bvh::Bvh(std::vector<RTTriangle>* triangles, SplitMode splitMode) {
		triangles_ = triangles;
		indices_ = std::vector<uint32_t>(triangles->size());
		centroids_.reserve(triangles->size());
		for (RTTriangle t : (*triangles_)) {
			centroids_.push_back(t.centroid());
		}
		trig_comparator = TrigComparator(triangles, &indices_);
		cout << "In total there are " << triangles->size() << " triangles" << endl;
		for (uint32_t i = 0; i < indices_.size(); i++)
			indices_[i] = i;
		mode_ = splitMode;
		if (mode_ != SplitMode::SplitMode_ObjectMedian && mode_ != SplitMode::SplitMode_SpatialMedian) {
			cout << "Defaulting to object median" << endl;
			mode_ = SplitMode::SplitMode_ObjectMedian;
		}
		BvhNode* n = new BvhNode(0, 0, indices_.size());
		rootNode_ = std::unique_ptr<BvhNode>(n);
		BvhNode* nodes[1000];
		int start[1000];
		int end[1000];
		int depth[1000];
		int stackp = 1;
		nodes[0] = n;
		start[0] = 0;
		end[0] = indices_.size();
		depth[0] = 0;
		while (stackp) {
			stackp--;
			int st = start[stackp];
			int en = end[stackp];
			int dep = depth[stackp];
			BvhNode* node = nodes[stackp];
			std::pair<AABB, AABB> bbs = calculateBB(st, en);
			AABB maxbb = bbs.first;
			AABB cenbb = bbs.second;
			node->bb = maxbb;
			if (en - st > TRI_LIMIT) {
				// Find the max dim
				uint32_t maxdim;
				F32 maxval = -1.0;
				Vec3f dims = bbs.second.max - bbs.second.min;
				for (uint32_t i = 0; i < 3; i++) {
					if (dims[i] > maxval) {
						maxval = dims[i];
						maxdim = i;
						trig_comparator.setDim(i);
					}
				}
				int mid;
				if (mode_ == SplitMode::SplitMode_ObjectMedian) {
					// Split on median of objects
					std::sort(indices_.begin() + st, indices_.begin() + en, trig_comparator);
					mid = st + (en - st) / 2;
				}
				else if(mode_ == SplitMode::SplitMode_SpatialMedian) {
					// Split on the center of axis
					F32 splitp = 0.5f * (cenbb.min[maxdim] + cenbb.max[maxdim]);
					mid = st;
					for (int i = st; i < en; i++) {
						if (centroids_[getIndex(i)][maxdim] < splitp) {
							std::swap(indices_[i], indices_[mid]);
							mid++;
						}
					}
					if (mid == st || mid == en)
						mid = st + (en - st) / 2;
				}
				else {
					throw "Split mode not implemented!";
				}
				nodes[stackp] = new BvhNode(dep + 1, st, mid);
				node->left.reset(nodes[stackp]);
				depth[stackp] = dep + 1;
				start[stackp] = st;
				end[stackp++] = mid;
				nodes[stackp] = new BvhNode(dep + 1, mid, en);
				node->right.reset(nodes[stackp]);
				depth[stackp] = dep + 1;
				start[stackp] = mid;
				end[stackp++] = en;
			}
		}
		cout << "We are done constructing the whole BVH" << endl;
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
