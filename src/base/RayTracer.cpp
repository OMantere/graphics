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
		//return naiveRaycast(orig, dir, 0, m_triangles->size());
			
		// YOUR CODE HERE (R1):
		// This is where you hierarchically traverse the tree you built!
		// You can use the existing code for the leaf nodes.

		BvhNode* stack[64];
		F32 node_t[64];				  // Minimum hit t on this node
		RaycastResult closest_intersection;  // Closest intersection found
		closest_intersection.t = 999999999.f;
		int stackp = 0;
		// Get the root node
		const BvhNode& root = bvh.root();
		if (!!root.left) {
			stack[stackp] = root.left.get();
			node_t[stackp++] = -999999999.f;
		} else
			return naiveRaycast(orig, dir, 0, m_triangles->size());
		if (!!root.right) {
			stack[stackp] = root.right.get();
			node_t[stackp++] = -999999999.f;
		}
		Vec3f Rinv = Vec3f(1.0f / dir[0], 1.0f / dir[1], 1.0f / dir[2]);
		while (stackp) {
			BvhNode* node = stack[--stackp];
			F32 near_t = node_t[stackp];
			if (near_t > closest_intersection.t) // If we have already intersected closer than this node, skip
				continue;
			if (!!node->left) {
				F32 left_intersect_t;
				F32 right_intersect_t = 999999999.f;
				bool left_result = node->left->bb.intersect2(orig, dir, Rinv, left_intersect_t);
				bool right_result = false;
				if (!!node->right) {
					bool right_result = node->right->bb.rectIntersect(orig, dir, Rinv, right_intersect_t);
				}
				if (right_result && left_result) {
					int left_i = stackp + 1;
					int right_i = stackp;
					stackp += 2;
					if (left_intersect_t > right_intersect_t) { // Push the farther node first
						std::swap(left_i, right_i);
					}
					stack[right_i] = node->right.get();
					node_t[right_i] = right_intersect_t;
					stack[left_i] = node->left.get();
					node_t[left_i] = left_intersect_t;
				}
				else if (left_result) {
					stack[stackp] = node->left.get();
					node_t[stackp++] = left_intersect_t;	
				}
				else if (right_result) {
					stack[stackp] = node->right.get();
					node_t[stackp++] = right_intersect_t;
				}
			} else { // We are at the leaf
				RaycastResult result = naiveRaycast(orig, dir, node->startPrim, node->endPrim);
				if (result && result.t < closest_intersection.t)
					closest_intersection = result;
			}
		}

		return closest_intersection;
	}

} // namespace FW