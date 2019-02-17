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
		inline bool intersect2(const Vec3f& orig, const Vec3f& dir, Vec3f& Rinv, F32& t)
		{
			F32 tmin = (min.x - orig.x) / dir.x;
			F32 tmax = (max.x - orig.x) / dir.x;

			if (tmin > tmax) swap(tmin, tmax);

			F32 tymin = (min.y - orig.y) / dir.y;
			F32 tymax = (max.y - orig.y) / dir.y;

			if (tymin > tymax) swap(tymin, tymax);

			if ((tmin > tymax) || (tymin > tmax))
				return false;

			if (tymin > tmin)
				tmin = tymin;

			if (tymax < tmax)
				tmax = tymax;

			F32 tzmin = (min.z - orig.z) / dir.z;
			F32 tzmax = (max.z - orig.z) / dir.z;

			if (tzmin > tzmax) swap(tzmin, tzmax);

			if ((tmin > tzmax) || (tzmin > tmax))
				return false;

			if (tzmin > tmin)
				tmin = tzmin;

			if (tzmax < tmax)
				tmax = tzmax;

			t = tmin;

			return true;
		}
		inline bool rectIntersect(const Vec3f& orig, const Vec3f& dir, Vec3f& Rinv, F32& t) const {
			F32 tmax = 999999999.f;
			F32 tmin = -999999999.f;
			F32 t1;
			F32 t2;
			for (int i = 0; i < 3; i++) {
				if (dir[i] == 0) { // Parallel ray
					if (orig[i] < min[i] || orig[i] > max[i]) // and outside of box
						return false;
				}
				else {
					t1 = (min[i] - orig[i]) * Rinv[i];
					t2 = (max[i] - orig[i]) * Rinv[i];
					if (t1 > t2) {
						F32 tmp = t2;
						t2 = t1;
						t1 = tmp;
					}
					tmin = std::max(tmin, t1);
					tmax = std::min(tmax, t2);
				}
			}
			if (tmin > tmax) { // The box is missed
				return false;
			}
			else if (tmax < 0) { // The box is behind
				return false;
			}
			else {
				if (tmin > 0) { // tmin is closer to the eye
					t = tmin;
				}
				else { // We are inside of box
					t = tmax;
				}
				return true;
			}
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
