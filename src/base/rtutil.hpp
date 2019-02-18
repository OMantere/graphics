#pragma once

/*
* utilities for internal use etc. in the raytracer class and the bvh construction
*/

#include "base/Math.hpp"

#include <iostream>
#include <limits>
#include <algorithm>

using namespace std;


namespace FW {


	enum SplitMode {
		SplitMode_SpatialMedian,
		SplitMode_ObjectMedian,
		SplitMode_Sah,
		SplitMode_None,
		SplitMode_Linear
	};

	struct Plane : public Vec4f {
		inline float dot(const Vec3f& p) const {
			return p.x * x + p.y * y + p.z * z + w;
		}
	};

	struct AABB {
		Vec3f min, max;
		inline AABB() : min(), max() {}
		inline AABB(const Vec3f& min, const Vec3f& max) : min(min), max(max) {}
		inline F32 area() const {
			Vec3f d(max - min);
			return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
		}
		inline bool rectIntersect(const Vec3f& orig, const Vec3f& dir, Vec3f& Rinv, F32& t) const {
			F32 tmax = 999999999.f;
			F32 tmin = -999999999.f;
			F32 t1;
			F32 t2;
			if (dir[0] == 0) { // Parallel ray
				if (orig[0] < min[0] || orig[0] > max[0]) // and outside of box
					return false;
			}
			else {
				if (dir[0] > 0) {
					tmin = (min[0] - orig[0]) * Rinv[0];
					tmax = (max[0] - orig[0]) * Rinv[0];
				}
				else {
					tmax = (min[0] - orig[0]) * Rinv[0];
					tmin = (max[0] - orig[0]) * Rinv[0];
				}
			}
			if (dir[1] == 0) { // Parallel ray
				if (orig[1] < min[1] || orig[1] > max[1]) // and outside of box
					return false;
			}
			else {
				if (dir[1] > 0) {
					t1 = (min[1] - orig[1]) * Rinv[1];
					t2 = (max[1] - orig[1]) * Rinv[1];
				}
				else {
					t2 = (min[1] - orig[1]) * Rinv[1];
					t1 = (max[1] - orig[1]) * Rinv[1];
				}
				if (tmin > t2 || t1 > tmax || tmax < 0)
					return false;
				tmin = std::max(tmin, t1);
				tmax = std::min(tmax, t2);
			}
			if (dir[2] == 0) { // Parallel ray
				if (orig[2] < min[2] || orig[2] > max[2]) // and outside of box
					return false;
			}
			else {
				if (dir[2] > 0) {
					t1 = (min[2] - orig[2]) * Rinv[2];
					t2 = (max[2] - orig[2]) * Rinv[2];
				}
				else {
					t2 = (min[2] - orig[2]) * Rinv[2];
					t1 = (max[2] - orig[2]) * Rinv[2];
				}
				if (tmin > t2 || t1 > tmax || tmax < 0)
					return false;
				tmin = std::max(tmin, t1);
				tmax = std::min(tmax, t2);
			}
			if (tmin > 0) { // tmin is closer to the eye
				t = tmin;
			}
			else { // We are inside of box
				t = tmax;
			}
			return true;
		}
	};


	inline std::ostream& operator<<(std::ostream& os, const FW::Vec3f& v) {
		return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
	}

	inline std::ostream& operator<<(std::ostream& os, const FW::Vec4f& v) {
		return os << "(" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << ")";
	}

	inline std::ostream& operator<<(std::ostream& os, const AABB& bb) {
		return os << "BB(" << bb.min << ", " << bb.max << ")";
	}


}
