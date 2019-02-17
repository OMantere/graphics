#pragma once

#include <vector>
#include <iostream>
#include "rtutil.hpp"
#include "RTTriangle.hpp"

namespace FW {
	class TrigComparator
	{
	public:
		TrigComparator() {};
		TrigComparator(std::vector<RTTriangle>* triangles, std::vector<uint32_t>* indices) : triangles_(triangles), indices_(indices) {
			for (RTTriangle t : (*triangles_)) {
				centroids.push_back(t.centroid());
			}
		};
		void setDim(int dim_) { dim = dim_; }
		bool operator ()(uint32_t a, uint32_t b)
		{
			return centroids[a][dim] < centroids[b][dim];
		}
	private:
		int dim;
		std::vector<RTTriangle>* triangles_;
		std::vector<uint32_t>* indices_;
		std::vector<Vec3f> centroids;
	};
}
