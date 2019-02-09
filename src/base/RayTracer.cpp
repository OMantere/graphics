#define _CRT_SECURE_NO_WARNINGS

#include "base/Defs.hpp"
#include "base/Math.hpp"
#include "RayTracer.hpp"
#include <stdio.h>
#include "rtIntersect.inl"
#include <fstream>
#include <algorithm>
#include <stack>
#include <memory>
#include <iostream>

#include "rtlib.hpp"

using namespace std;


// Helper function for hashing scene data for caching BVHs
extern "C" void MD5Buffer(void* buffer, size_t bufLen, unsigned int* pDigest);


namespace FW
{


	Vec2f getTexelCoords(Vec2f uv, const Vec2i size)
	{

		// YOUR CODE HERE (R3):
		// Get texel indices of texel nearest to the uv vector. Used in texturing.
		// UV coordinates range from negative to positive infinity. First map them
		// to a range between 0 and 1 in order to support tiling textures, then
		// scale the coordinates by image resolution and find the nearest pixel.
		return Vec2f();
	}

	Mat3f formBasis(const Vec3f& n) {
		// YOUR CODE HERE (R4):
		return Mat3f();
	}


	String RayTracer::computeMD5(const std::vector<Vec3f>& vertices)
	{
		unsigned char digest[16];
		MD5Buffer((void*)&vertices[0], sizeof(Vec3f)*vertices.size(), (unsigned int*)digest);

		// turn into string
		char ad[33];
		for (int i = 0; i < 16; ++i)
			::sprintf(ad + i * 2, "%02x", digest[i]);
		ad[32] = 0;

		return FW::String(ad);
	}


	// --------------------------------------------------------------------------


	RayTracer::RayTracer()
	{
	}

	RayTracer::~RayTracer()
	{
	}


	void RayTracer::loadHierarchy(const char* filename, std::vector<RTTriangle>& triangles)
	{
		std::ifstream ifs(filename, std::ios::binary);
		bvh = std::move(Bvh(ifs));

		m_triangles = &triangles;
	}

	void RayTracer::saveHierarchy(const char* filename, const std::vector<RTTriangle>& triangles) {
		(void)triangles; // Not used.

		std::ofstream ofs(filename, std::ios::binary);
		bvh.save(ofs);
	}

	void RayTracer::constructHierarchy(std::vector<RTTriangle>& triangles, SplitMode splitMode) {
		// YOUR CODE HERE (R1):
		m_triangles = &triangles;
		bvh = Bvh(m_triangles, splitMode);
	}

	RaycastResult RayTracer::naiveRaycast(const Vec3f& orig, const Vec3f& dir, int start = 0, int end = -1) const {
		if (end < 0)
			end = m_triangles->size();
		float tmin = 1.0f, umin = 0.0f, vmin = 0.0f;
		int imin = -1;

		RaycastResult castresult;

		// Naive loop over all triangles.
		for (int j = start; j < end; j++) {
			int i = bvh.getIndex(j);
			float t, u, v;
			if ((*m_triangles)[i].intersect_woop(orig, dir, t, u, v))
			{
				if (t > 0.0f && t < tmin)
				{
					imin = i;
					tmin = t;
					umin = u;
					vmin = v;
				}
			}
		}

		if (imin != -1) {
			castresult = RaycastResult(&(*m_triangles)[imin], tmin, umin, vmin, orig + tmin * dir, orig, dir);
		}
		return castresult;
	}


	RaycastResult RayTracer::raycast(const Vec3f& orig, const Vec3f& dir) const {
		++m_rayCount;

		// YOUR CODE HERE (R1):
		// This is where you hierarchically traverse the tree you built!
		// You can use the existing code for the leaf nodes.

		std::stack<BvhNode*> s;
		// Get the root node
		const BvhNode& root = bvh.root();
		if (!!root.left)
			s.push(root.left.get());
		else
			return naiveRaycast(orig, dir, 0, m_triangles->size());
		if (!!root.right)
			s.push(root.right.get());
		RaycastResult best_result;
		Vec3f Rinv = Vec3f(1.0f / dir[0], 1.0f / dir[1], 1.0f / dir[2]);
		while (!s.empty()) {
			BvhNode* node = s.top(); s.pop();
			// Get the bb at this node, check intersections with the ray
			AABB bb = node->bb;
			float intersect_t;
			bool did_intersect = bb.rectIntersect(orig, dir, Rinv, intersect_t);
			// If there is intersection, recurse further
			if (did_intersect) {
				if (!!node->left) {
					s.push(node->left.get());
					if (!!node->right)
						s.push(node->right.get());
				}
				else { // We are at the leaf
					RaycastResult result = naiveRaycast(orig, dir, node->startPrim, node->endPrim);
					if (result && result.t < best_result.t) {
						best_result = result;
					}
				}
			}
		}

		return best_result;
	}

} // namespace FW