#ifndef SIMPLERT_HIT_RECORD_H
#define SIMPLERT_HIT_RECORD_H

#include "misc.h"
#include "ray.h"

class material;

struct hit_record
{
    HMM_Vec3 p;
    HMM_Vec3 normal;
    std::shared_ptr<material> mat;
    float t;
    float u;
    float v;
    bool front_face;

    void set_face_normal(const ray& r, const HMM_Vec3& outward_normal)
    {
        // Sets the hit record normal vector.
        // **NOTE**: the parameter `outward_normal` is assumed to have unit length.

        front_face = HMM_Dot(r.direction(), outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};

#endif //SIMPLERT_HIT_RECORD_H
