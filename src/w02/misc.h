#ifndef SIMPLERT_MISC_H
#define SIMPLERT_MISC_H

#include "HandmadeMath.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <limits>
#include <numbers>
#include <vector>
#include <memory>
#include <random>

const float infinity = std::numeric_limits<float>::infinity();
const float pi = std::numbers::pi_v<float>;

class interval
{
public:
    float min, max;

    interval() : min(+infinity), max(-infinity) {} // Default interval is empty
    interval(float _min, float _max) : min(_min), max(_max) {}
    interval(const interval& a, const interval& b) : min(fmin(a.min, b.min)), max(fmax(a.max, b.max)) {}

    bool contains(float x) const {
        return min <= x && x <= max;
    }

    bool surrounds(float x) const {
        return min < x && x < max;
    }

    float clamp(float x) const {
        if (x < min) return min;
        if (x > max) return max;
        return x;
    }

    interval expand(float delta) const {
        auto padding = delta/2.0f;
        return interval(min - padding, max + padding);
    }

    float size() const {
        return max - min;
    }

    static const interval empty, universe;
};
const static interval empty   (+infinity, -infinity);
const static interval universe(-infinity, +infinity);

interval operator+(const interval& ival, float displacement) {
    return interval(ival.min + displacement, ival.max + displacement);
}

interval operator+(float displacement, const interval& ival) {
    return ival + displacement;
}

void push_color(std::vector<uint8_t>& image_data, HMM_Vec3 pixel_color)
{
    image_data.push_back(static_cast<int>(255.999 * pixel_color.X));
    image_data.push_back(static_cast<int>(255.999 * pixel_color.Y));
    image_data.push_back(static_cast<int>(255.999 * pixel_color.Z));
}

void save_jpg(int image_width, int image_height, const std::vector<HMM_Vec3>& image_color_data, const char* filename)
{
    std::vector<uint8_t> image_data;
    image_data.reserve(image_width * image_height * 3);

    for (int i = 0; i < image_height * image_width; i++)
    {
        push_color(image_data, image_color_data[i]);
    }

    stbi_write_jpg(filename, image_width, image_height, 3, image_data.data(), 100);
}

inline float random_float()
{
    // Returns a random real in [0,1).
    static std::uniform_real_distribution<float> distribution(0.0, 1.0);
    static std::mt19937 generator;
    return distribution(generator);
}

inline float random_float(float min, float max)
{
    // Returns a random real in [min,max).
    return min + (max - min) * random_float();
}

inline int random_int(int min, int max)
{
    // Returns a random integer in [min,max].
    return static_cast<int>(random_float(min, max+1));
}

namespace Vec3
{
    inline HMM_Vec3 random_vec3()
    {
        return HMM_Vec3{random_float(), random_float(), random_float()};
    }

    inline HMM_Vec3 random_vec3(float min, float max)
    {
        return HMM_Vec3{random_float(min, max), random_float(min, max), random_float(min, max)};
    }

    inline HMM_Vec3 random_in_unit_sphere()
    {
        while (true)
        {
            auto p = random_vec3(-1, 1);
            if (HMM_LenSqr(p) < 1)
                return p;
        }
    }

    inline HMM_Vec3 random_unit_vector()
    {
        return HMM_Norm(random_in_unit_sphere());
    }

    inline HMM_Vec3 random_on_hemisphere(const HMM_Vec3& normal)
    {
        auto on_unit_sphere = random_unit_vector();
        if (HMM_Dot(on_unit_sphere, normal) > 0.0f) // In the same hemisphere as the normal
            return on_unit_sphere;
        else
            return -on_unit_sphere;
    }

    bool vec3_near_zero(const HMM_Vec3& v)
    {
        // Return true if the vector is close to zero in all dimensions.
        auto s = 1e-8f;
        return (std::fabsf(v.X) < s) && (std::fabsf(v.Y) < s) && (std::fabsf(v.Z) < s);
    }

    inline HMM_Vec3 reflect(const HMM_Vec3& v, const HMM_Vec3& n)
    {
        return v - 2*HMM_Dot(v,n)*n;
    }

    inline HMM_Vec3 refract(const HMM_Vec3& uv, const HMM_Vec3& n, float etai_over_etat)
    {
        auto cos_theta = std::fminf(HMM_Dot(-uv, n), 1.0f);
        HMM_Vec3 r_out_perp = etai_over_etat * (uv + cos_theta*n);
        HMM_Vec3 r_out_parallel = -std::sqrtf(std::fabsf(1.0f - HMM_LenSqr(r_out_perp))) * n;
        return r_out_perp + r_out_parallel;
    }

    inline HMM_Vec3 random_in_unit_disk()
    {
        while (true)
        {
            HMM_Vec3 p = {random_float(-1, 1), random_float(-1, 1), 0};
            if (HMM_LenSqr(p) < 1)
                return p;
        }
    }
};

inline float linear_to_gamma(float linear_component)
{
    return std::sqrtf(linear_component);
}

#endif //SIMPLERT_MISC_H
