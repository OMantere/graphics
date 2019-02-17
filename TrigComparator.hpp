#pragma once

#include <vector>
#include "rtutil.hpp"
#include "RTTriangle.hpp"

namespace FW {
	class TrigComparator
	{
	public:
		int dim;
		std::vector<RTTriangle>* triangles_;
		std::vector<uint32_t>& indices_;
		std::vector<Vec3f> centroids;
		TrigComparator(std::vector<RTTriangle>* triangles, std::vector<uint32_t>& indices) : triangles_(triangles), indices_(indices) {
			for (RTTriangle t : (*triangles_))
				centroids.push_back(t.centroid());
		};
		void setDim(int dim) { dim = dim; }
		bool operator ()(int a, int b)
		{
			return centroids[indices_[a]][dim] < centroids[indices_[b]][dim];
		}
	};
}
