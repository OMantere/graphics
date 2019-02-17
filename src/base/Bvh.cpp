
#include "Bvh.hpp"
#include "filesaves.hpp"

#include <algorithm>
#include <iostream>

using namespace std;


namespace FW {

	const uint32_t TRI_LIMIT = 3;
	const uint32_t SAH_LIMIT = 3;
	const uint32_t GROUPS = 10;

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

	AABB Bvh::maxBB(int start, int end) {
		Vec3f ma = Vec3f();
		Vec3f mi = Vec3f();
		for (int i = start; i < end; i++) {
			RTTriangle trig = (*triangles_)[getIndex(i)];
			Vec3f maxx = trig.max();
			Vec3f minn = trig.min();
			for (uint32_t j = 0; j < 3; j++) {
				ma[j] = std::max(maxx[j], ma[j]);
				mi[j] = std::min(minn[j], mi[j]);
			}
		}
		return AABB(mi, ma);
	}

	F32 Bvh::surfaceArea(AABB bb) {
		F32 x = bb.max[0] - bb.min[0];
		F32 y = bb.max[1] - bb.min[1];
		F32 z = bb.max[2] - bb.min[2];
		return 2 * (x * y + y * z + x * z);
	}

	Bvh::Bvh(std::vector<RTTriangle>* triangles, SplitMode splitMode) {
		triangles_ = triangles;
		indices_ = std::vector<uint32_t>(triangles->size());
		trig_comparator = TrigComparator(triangles, &indices_);
		cout << "In total there are " << triangles->size() << " triangles" << endl;
		for (uint32_t i = 0; i < indices_.size(); i++)
			indices_[i] = i;
		mode_ = splitMode;
		cout << "Split mode: " << mode_ << endl;
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
		std::sort(indices_.begin(), indices_.end(), trig_comparator);
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
			cout << "We have node with n = " << en - st << " triangles, bb is " << node->bb << endl;
			if (mode_ == SplitMode::SplitMode_Sah) {
				if (en - st > SAH_LIMIT) {
					// Try planes
					F32 best_score = 999999999.0f;
					int best_mid;
					int best_dim;
					Vec3f dims = cenbb.max - cenbb.min;
					int n = en - st;
					for (int j = 0; j < 3; j++) {
						trig_comparator.setDim(j);
						std::sort(indices_.begin() + st, indices_.begin() + en, trig_comparator);
						for (int split = 1; split < GROUPS - 1; split++) {
							int mid = split * n / GROUPS;
							AABB left_bb = maxBB(st, mid);
							AABB right_bb = maxBB(mid, en);
							F32 sah_score = surfaceArea(left_bb) * (mid - st) + surfaceArea(right_bb) * (en - mid);
							if (sah_score < best_score) {
								best_score = sah_score;
								best_mid = mid;
								best_dim = j;
							}
						}
					}
					int mid = best_mid;
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
			
			} else {
				if (en - st > TRI_LIMIT) {
					// Find the max dim
					F32 maxval = -1.0;
					Vec3f dims = bbs.second.max - bbs.second.min;
					for (uint32_t i = 0; i < 3; i++) {
						if (dims[i] > maxval) {
							maxval = dims[i];
							trig_comparator.setDim(i);
						}
					}
					// Split on the object median
					std::sort(indices_.begin() + st, indices_.begin() + en, trig_comparator);
					int mid = (en + st) / 2;
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
