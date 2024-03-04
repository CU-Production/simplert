#ifndef SIMPLERT_MATERIAL_H
#define SIMPLERT_MATERIAL_H

#include "misc.h"
#include "ray.h"
#include "hit_record.h"
#include "texture.h"

class material
{
public:
    virtual ~material() = default;

    virtual bool scatter(const ray& r_in, const hit_record& rec, HMM_Vec3& attenuation, ray& scattered) const = 0;
};

class lambertian : public material
{
public:
    lambertian(const HMM_Vec3& a) : albedo(std::make_shared<solid_color>(a)) {}
    lambertian(std::shared_ptr<texture> a) : albedo(a) {}

    bool scatter(const ray& r_in, const hit_record& rec, HMM_Vec3& attenuation, ray& scattered) const override
    {
        auto scatter_direction = rec.normal + Vec3::random_unit_vector();

        // Catch degenerate scatter direction
        if (Vec3::vec3_near_zero(scatter_direction))
            scatter_direction = rec.normal;

        scattered = ray(rec.p, scatter_direction, r_in.time());
        attenuation = albedo->value(rec.u, rec.v, rec.p);
        return true;
    }

private:
    std::shared_ptr<texture> albedo;
};

class metal : public material
{
public:
    metal(const HMM_Vec3& a, float f) : albedo(a), fuzz(f < 1 ? f : 1.0f) {}

    bool scatter(const ray& r_in, const hit_record& rec, HMM_Vec3& attenuation, ray& scattered) const override
    {
        HMM_Vec3 reflected = Vec3::reflect(HMM_Norm(r_in.direction()), rec.normal);
        scattered = ray(rec.p, reflected + fuzz*Vec3::random_unit_vector(), r_in.time());
        attenuation = albedo;
        return (HMM_Dot(scattered.direction(), rec.normal) > 0);;
    }

private:
    HMM_Vec3 albedo;
    float fuzz;
};

class dielectric : public material
{
public:
    dielectric(float index_of_refraction) : ir(index_of_refraction) {}

    bool scatter(const ray& r_in, const hit_record& rec, HMM_Vec3& attenuation, ray& scattered) const override
    {
        attenuation = {1.0, 1.0, 1.0};
        float refraction_ratio = rec.front_face ? (1.0f/ir) : ir;

        HMM_Vec3 unit_direction = HMM_Norm(r_in.direction());
        float cos_theta = std::fminf(HMM_Dot(-unit_direction, rec.normal), 1.0);
        float sin_theta = std::sqrtf(1.0f - cos_theta*cos_theta);

        bool cannot_refract = refraction_ratio * sin_theta > 1.0;
        HMM_Vec3 direction;

        if (cannot_refract || reflectance(cos_theta, refraction_ratio) > random_float())
            direction = Vec3::reflect(unit_direction, rec.normal);
        else
            direction = Vec3::refract(unit_direction, rec.normal, refraction_ratio);

        scattered = ray(rec.p, direction, r_in.time());
        return true;
    }

private:
    float ir; // Index of Refraction

    static float reflectance(float cosine, float ref_idx)
    {
        // Use Schlick's approximation for reflectance.
        auto r0 = (1-ref_idx) / (1+ref_idx);
        r0 = r0*r0;
        return r0 + (1-r0)*std::powf((1 - cosine), 5);
    }
};

#endif //SIMPLERT_MATERIAL_H
