# Advanced Computer Graphics

## Completed tasks

### First round
BVH traversal and raycasting seems to work, but my BVHs are very slow. The traversal with reference BVH is very fast though.

I implemented spatial and object median modes, but they may not by implemented correctly.

Object median BVH with cutoff at max 64 trigs to split a node builds the BVH in about 14 seconds on conference. Performance is several times better than with naive raytrace but still pretty poor (< 0.1 MRays).

Both BVH construction and traversal use stacks.

The traversal code is in ```RayTracer.cpp```, method ```raycast()```. It calls a rectangle intersetion routine, ```rectIntersect```, defined on the ```AABB``` class, in ```rtutil.cpp```.

The BVH construction is defined on ```Bvh.cpp```.
