#ifndef SIMPLERT_RAY_H
#define SIMPLERT_RAY_H

#include "misc.h"

class ray
{
public:
    ray() {}
    ray(const HMM_Vec3& origin, const HMM_Vec3& direction) : orig(origin), dir(direction), tm(0) {}
    ray(const HMM_Vec3& origin, const HMM_Vec3& direction, float time) : orig(origin), dir(direction), tm(time) {}

    HMM_Vec3 origin() const  { return orig; }
    HMM_Vec3 direction() const { return dir; }
    float time() const { return tm; }

    HMM_Vec3 at(float t) const { return t*dir + orig; }

private:
    HMM_Vec3 orig;
    HMM_Vec3 dir;
    float tm;
};

#endif //SIMPLERT_RAY_H
