#ifndef SIMPLERT_HITTABLE_H
#define SIMPLERT_HITTABLE_H

#include "misc.h"
#include "ray.h"
#include "material.h"
#include "aabb.h"

#include <cmath>

class hittable
{
public:
    virtual ~hittable() = default;
    virtual bool hit(const ray& r, interval ray_t, hit_record& rec) const = 0;
    virtual aabb bounding_box() const = 0;
};

class sphere : public hittable
{
public:
    // Stationary Sphere
    sphere(HMM_Vec3 _center, float _radius, std::shared_ptr<material> _material) : center1(_center), radius(_radius), mat(_material), is_moving(false)
    {
        auto rvec = HMM_V3(radius, radius, radius);
        bbox = aabb(center1 - rvec, center1 + rvec);
    }

    // Moving Sphere
    sphere(HMM_Vec3 _center1, HMM_Vec3 _center2, float _radius, std::shared_ptr<material> _material) : center1(_center1), radius(_radius), mat(_material), is_moving(true)
    {
        center_vec = _center2 - _center1;

        auto rvec = HMM_V3(radius, radius, radius);
        aabb box1(_center1 - rvec, _center1 + rvec);
        aabb box2(_center2 - rvec, _center1 + rvec);
        bbox = aabb(box1, box2);
    }

    aabb bounding_box() const override { return bbox; }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override
    {
        HMM_Vec3 center = is_moving ? sphere_center(r.time()) : center1;
        HMM_Vec3 oc = r.origin() - center;
        auto a = HMM_LenSqr(r.direction());
        auto half_b = HMM_Dot(oc, r.direction());
        auto c = HMM_LenSqr(oc) - (radius * radius);

        auto discriminant = half_b*half_b - a*c;
        if (discriminant < 0) return false;
        auto sqrtd = std::sqrtf(discriminant);

        // Find the nearest root that lies in the acceptable range.
        auto root = (-half_b - sqrtd) / a;
        if (!ray_t.surrounds(root))
        {
            root = (-half_b + sqrtd) / a;
            if (!ray_t.surrounds(root))
                return false;
        }

        rec.t = root;
        rec.p = r.at(rec.t);
        HMM_Vec3 outward_normal = (rec.p - center1) / radius;
        rec.set_face_normal(r, outward_normal);
        get_sphere_uv(outward_normal, rec.u, rec.v);
        rec.mat = mat;

        return true;
    }

    static void get_sphere_uv(const HMM_Vec3 & p, float& u, float& v)
    {
        // p: a given point on the sphere of radius one, centered at the origin.
        // u: returned value [0,1] of angle around the Y axis from X=-1.
        // v: returned value [0,1] of angle from Y=-1 to Y=+1.
        //     <1 0 0> yields <0.50 0.50>       <-1  0  0> yields <0.00 0.50>
        //     <0 1 0> yields <0.50 1.00>       < 0 -1  0> yields <0.50 0.00>
        //     <0 0 1> yields <0.25 0.50>       < 0  0 -1> yields <0.75 0.50>

        auto theta = std::acosf(-p.Y);
        auto phi = std::atan2f(-p.Z, p.X) + pi;

        u = phi / (2*pi);
        v = theta / pi;
    }

private:
    HMM_Vec3 center1;
    float radius;
    std::shared_ptr<material> mat;
    bool is_moving;
    HMM_Vec3 center_vec;
    aabb bbox;

    HMM_Vec3 sphere_center(float time) const
    {
        // Linearly interpolate from center1 to center2 according to time, where t=0 yields
        // center1, and t=1 yields center2.
        return center1 + time*center_vec;
    }
};

class quad : public hittable
{
public:
    quad(const HMM_Vec3& _Q, const HMM_Vec3& _u, const HMM_Vec3& _v, std::shared_ptr<material> m) : Q(_Q), u(_u), v(_v), mat(m)
    {
        auto n = HMM_Cross(u, v);
        normal = HMM_Norm(n);
        D = HMM_Dot(normal, Q);
        w = n / HMM_Dot(n, n);

        set_bounding_box();
    }

    virtual void set_bounding_box()
    {
        bbox = aabb(Q, Q+u+v).pad();
    }

    aabb bounding_box() const override { return bbox; }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override
    {
        auto denom = HMM_Dot(normal, r.direction());

        // No hit if the ray is parallel to the plane.
        if (std::fabsf(denom) < 1e-8)
            return false;

        // Return false if the hit point parameter t is outside the ray interval.
        auto t = (D - HMM_Dot(normal, r.origin())) / denom;
        if (!ray_t.contains(t))
            return false;

        // Determine the hit point lies within the planar shape using its plane coordinates.
        auto intersection = r.at(t);
        auto planar_hitpt_vector = intersection - Q;
        auto alpha = HMM_Dot(w, HMM_Cross(planar_hitpt_vector, v));
        auto beta = HMM_Dot(w, HMM_Cross(u, planar_hitpt_vector));

        if (!is_interior(alpha, beta, rec))
            return false;

        rec.t = t;
        rec.p = intersection;
        rec.mat = mat;
        rec.set_face_normal(r, normal);

        return true;
    }

    virtual bool is_interior(float a, float b, hit_record& rec) const
    {
        // Given the hit point in plane coordinates, return false if it is outside the
        // primitive, otherwise set the hit record UV coordinates and return true.

        if ((a < 0) || (1 < a) || (b < 0) || (1 < b))
            return false;

        rec.u = a;
        rec.v = b;
        return true;
    }

private:
    HMM_Vec3 Q;
    HMM_Vec3 u, v;
    std::shared_ptr<material> mat;
    aabb bbox;
    HMM_Vec3 normal;
    float D;
    HMM_Vec3 w;
};

class hittable_list : public hittable
{
public:
    std::vector<std::shared_ptr<hittable>> objects;

    hittable_list() {}
    hittable_list(std::shared_ptr<hittable> object) { add(object); }

    void clear() { objects.clear(); }
    void add(std::shared_ptr<hittable> object)
    {
        objects.push_back(object);
        bbox = aabb(bbox, object->bounding_box());
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override
    {
        hit_record temp_rcc;
        bool hit_anything = false;
        float closest_so_far = ray_t.max;

        for (const auto& object : objects)
        {
            if (object->hit(r, interval(ray_t.min, closest_so_far), temp_rcc))
            {
                hit_anything = true;
                closest_so_far = temp_rcc.t;
                rec = temp_rcc;
            }
        }

        return hit_anything;
    }

    aabb bounding_box() const override { return bbox; }

private:
    aabb bbox;
};

class bvh_node : public hittable
{
public:
    bvh_node(const hittable_list& list) : bvh_node(list.objects, 0, list.objects.size()) {}

    bvh_node(const std::vector<std::shared_ptr<hittable>>& src_objects, size_t start, size_t end)
    {
        auto objects = src_objects; // Create a modifiable array of the source scene objects

        int axis = random_int(0, 2);
        auto comparator = (axis == 0) ? box_x_compare
                                      : (axis == 1) ? box_y_compare
                                                    : box_z_compare;

        size_t object_span = end - start;

        if (object_span == 1) {
            left = right = objects[start];
        } else if (object_span == 2) {
            if (comparator(objects[start], objects[start+1])) {
                left = objects[start];
                right = objects[start+1];
            } else {
                left = objects[start+1];
                right = objects[start];
            }
        } else {
            std::sort(objects.begin() + start, objects.begin() + end, comparator);

            auto mid = start + object_span/2;
            left = std::make_shared<bvh_node>(objects, start, mid);
            right = std::make_shared<bvh_node>(objects, mid, end);
        }

        bbox = aabb(left->bounding_box(), right->bounding_box());
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override
    {
        if (!bbox.hit(r, ray_t))
            return false;

        bool hit_left = left->hit(r, ray_t, rec);
        bool hit_right = right->hit(r, interval(ray_t.min, hit_left ? rec.t : ray_t.max), rec);

        return hit_left || hit_right;
    }

    aabb bounding_box() const override { return bbox; }

private:
    std::shared_ptr<hittable> left;
    std::shared_ptr<hittable> right;
    aabb bbox;

    static bool box_compare(const std::shared_ptr<hittable> a, const std::shared_ptr<hittable> b, int axis_index) {
        return a->bounding_box().axis(axis_index).min < b->bounding_box().axis(axis_index).min;
    }

    static bool box_x_compare (const std::shared_ptr<hittable> a, const std::shared_ptr<hittable> b) {
        return box_compare(a, b, 0);
    }

    static bool box_y_compare (const std::shared_ptr<hittable> a, const std::shared_ptr<hittable> b) {
        return box_compare(a, b, 1);
    }

    static bool box_z_compare (const std::shared_ptr<hittable> a, const std::shared_ptr<hittable> b) {
        return box_compare(a, b, 2);
    }
};

inline std::shared_ptr<hittable_list> box(const HMM_Vec3& a, const HMM_Vec3& b, std::shared_ptr<material> mat)
{
    // Returns the 3D box (six sides) that contains the two opposite vertices a & b.

    auto sides = std::make_shared<hittable_list>();

    // Construct the two opposite vertices with the minimum and maximum coordinates.
    auto min = HMM_V3(std::fminf(a.X, b.X), std::fminf(a.Y, b.Y), std::fminf(a.Z, b.Z));
    auto max = HMM_V3(std::fmaxf(a.X, b.X), std::fmaxf(a.Y, b.Y), std::fmaxf(a.Z, b.Z));

    auto dx = HMM_V3(max.X - min.X, 0, 0);
    auto dy = HMM_V3(0, max.Y - min.Y, 0);
    auto dz = HMM_V3(0, 0, max.Z - min.Z);

    sides->add(make_shared<quad>(HMM_V3(min.X, min.Y, max.Z),  dx,  dy, mat)); // front
    sides->add(make_shared<quad>(HMM_V3(max.X, min.Y, max.Z), -dz,  dy, mat)); // right
    sides->add(make_shared<quad>(HMM_V3(max.X, min.Y, min.Z), -dx,  dy, mat)); // back
    sides->add(make_shared<quad>(HMM_V3(min.X, min.Y, min.Z),  dz,  dy, mat)); // left
    sides->add(make_shared<quad>(HMM_V3(min.X, max.Y, max.Z),  dx, -dz, mat)); // top
    sides->add(make_shared<quad>(HMM_V3(min.X, min.Y, min.Z),  dx,  dz, mat)); // bottom

    return sides;
}

class translate : public hittable
{
public:
    translate(std::shared_ptr<hittable> p, const HMM_Vec3& displacement) : object(p), offset(displacement)
    {
        bbox = object->bounding_box() + offset;
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override
    {
        // Move the ray backwards by the offset
        ray offset_r(r.origin() - offset, r.direction(), r.time());

        // Determine where (if any) an intersection occurs along the offset ray
        if (!object->hit(offset_r, ray_t, rec))
            return false;

        // Move the intersection point forwards by the offset
        rec.p += offset;

        return true;
    }

    aabb bounding_box() const override { return bbox; }

private:
    std::shared_ptr<hittable> object;
    HMM_Vec3 offset;
    aabb bbox;
};

class rotate_y : public hittable {
public:
    rotate_y(std::shared_ptr<hittable> p, float angle) : object(p)
    {
        auto radians = HMM_DegToRad * angle;
        sin_theta = std::sinf(radians);
        cos_theta = std::cosf(radians);
        bbox = object->bounding_box();

        HMM_Vec3 min = HMM_V3( infinity,  infinity,  infinity);
        HMM_Vec3 max = HMM_V3(-infinity, -infinity, -infinity);

        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                for (int k = 0; k < 2; k++) {
                    auto x = i*bbox.x.max + (1-i)*bbox.x.min;
                    auto y = j*bbox.y.max + (1-j)*bbox.y.min;
                    auto z = k*bbox.z.max + (1-k)*bbox.z.min;

                    auto newx =  cos_theta*x + sin_theta*z;
                    auto newz = -sin_theta*x + cos_theta*z;

                    HMM_Vec3 tester = HMM_V3(newx, y, newz);

                    for (int c = 0; c < 3; c++) {
                        min[c] = fmin(min[c], tester[c]);
                        max[c] = fmax(max[c], tester[c]);
                    }
                }
            }
        }

        bbox = aabb(min, max);
    }

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override {
        // Change the ray from world space to object space
        auto origin = r.origin();
        auto direction = r.direction();

        origin[0] = cos_theta * r.origin()[0] - sin_theta * r.origin()[2];
        origin[2] = sin_theta * r.origin()[0] + cos_theta * r.origin()[2];

        direction[0] = cos_theta * r.direction()[0] - sin_theta * r.direction()[2];
        direction[2] = sin_theta * r.direction()[0] + cos_theta * r.direction()[2];

        ray rotated_r(origin, direction, r.time());

        // Determine where (if any) an intersection occurs in object space
        if (!object->hit(rotated_r, ray_t, rec))
            return false;

        // Change the intersection point from object space to world space
        auto p = rec.p;
        p[0] = cos_theta * rec.p[0] + sin_theta * rec.p[2];
        p[2] = -sin_theta * rec.p[0] + cos_theta * rec.p[2];

        // Change the normal from object space to world space
        auto normal = rec.normal;
        normal[0] = cos_theta * rec.normal[0] + sin_theta * rec.normal[2];
        normal[2] = -sin_theta * rec.normal[0] + cos_theta * rec.normal[2];

        rec.p = p;
        rec.normal = normal;
    };

    aabb bounding_box() const override { return bbox; }

private:
    std::shared_ptr<hittable> object;
    float sin_theta;
    float cos_theta;
    aabb bbox;
};

#endif //SIMPLERT_HITTABLE_H
