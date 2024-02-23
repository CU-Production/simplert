#include "HandmadeMath.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <iostream>
#include <vector>
#include <memory>
#include <limits>
#include <numbers>
#include <random>
#include <chrono>
#include <cmath>
#include <cstdlib>

const float infinity = std::numeric_limits<float>::infinity();
const float pi = std::numbers::pi_v<float>;

class interval
{
public:
    float min, max;

    interval() : min(+infinity), max(-infinity) {} // Default interval is empty
    interval(float _min, float _max) : min(_min), max(_max) {}

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

    static const interval empty, universe;
};
const static interval empty   (+infinity, -infinity);
const static interval universe(-infinity, +infinity);

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
};

inline float linear_to_gamma(float linear_component)
{
    return std::sqrtf(linear_component);
}

class ray
{
public:
    ray() {}
    ray(const HMM_Vec3& origin, const HMM_Vec3& direction) : orig(origin), dir(direction) {}

    HMM_Vec3 origin() const  { return orig; }
    HMM_Vec3 direction() const { return dir; }

    HMM_Vec3 at(float t) const { return t*dir + orig; }

private:
    HMM_Vec3 orig;
    HMM_Vec3 dir;
};

class material;
struct hit_record
{
    HMM_Vec3 p;
    HMM_Vec3 normal;
    std::shared_ptr<material> mat;
    float t;
    bool front_face;

    void set_face_normal(const ray& r, const HMM_Vec3& outward_normal)
    {
        // Sets the hit record normal vector.
        // **NOTE**: the parameter `outward_normal` is assumed to have unit length.

        front_face = HMM_Dot(r.direction(), outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};

class material
{
public:
    virtual ~material() = default;

    virtual bool scatter(const ray& r_in, const hit_record& rec, HMM_Vec3& attenuation, ray& scattered) const = 0;
};

class lambertian : public material
{
public:
    lambertian(const HMM_Vec3& a) : albedo(a) {}

    bool scatter(const ray& r_in, const hit_record& rec, HMM_Vec3& attenuation, ray& scattered) const override
    {
        auto scatter_direction = rec.normal + Vec3::random_unit_vector();

        // Catch degenerate scatter direction
        if (Vec3::vec3_near_zero(scatter_direction))
            scatter_direction = rec.normal;

        scattered = ray(rec.p, scatter_direction);
        attenuation = albedo;
        return true;
    }

private:
    HMM_Vec3 albedo;
};

class metal : public material
{
public:
    metal(const HMM_Vec3 & a, float f) : albedo(a), fuzz(f < 1 ? f : 1.0f) {}

    bool scatter(const ray& r_in, const hit_record& rec, HMM_Vec3& attenuation, ray& scattered) const override
    {
        HMM_Vec3 reflected = Vec3::reflect(HMM_Norm(r_in.direction()), rec.normal);
        scattered = ray(rec.p, reflected + fuzz*Vec3::random_unit_vector());
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

        if (cannot_refract)
            direction = Vec3::reflect(unit_direction, rec.normal);
        else
            direction = Vec3::refract(unit_direction, rec.normal, refraction_ratio);

        scattered = ray(rec.p, direction);
        return true;
    }

private:
    float ir; // Index of Refraction
};

class hittable
{
public:
    virtual ~hittable() = default;
    virtual bool hit(const ray& r, interval ray_t, hit_record& rec) const = 0;
};

class sphere : public hittable
{
public:
    sphere(HMM_Vec3 _center, float _radius, std::shared_ptr<material> _material) : center(_center), radius(_radius), mat(_material) {}

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override
    {
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
        HMM_Vec3 outward_normal = (rec.p - center) / radius;
        rec.set_face_normal(r, outward_normal);
        rec.mat = mat;

        return true;
    }

private:
    HMM_Vec3 center;
    float radius;
    std::shared_ptr<material> mat;
};

class hittable_list : public hittable
{
public:
    std::vector<std::shared_ptr<hittable>> objects;

    hittable_list() {}
    hittable_list(std::shared_ptr<hittable> object) { add(object); }

    void clear() { objects.clear(); }
    void add(std::shared_ptr<hittable> object) { objects.push_back(object); }

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
};

class camera
{
public:
    float aspect_ratio      = 1.0;  // Ratio of image width over height
    int   image_width       = 100;  // Rendered image width in pixel count
    int   image_height      = 100;  // Rendered image height
    int   samples_per_pixel = 10;   // Count of random samples for each pixel
    int   max_depth         = 10;   // Maximum number of ray bounces into scene

    std::vector<HMM_Vec3> render(const hittable& world)
    {
        initialize();

        std::vector<HMM_Vec3> image_color_data;
        image_color_data.resize(image_width * image_height, {0, 0, 0});

        for (int j = 0; j < image_height; ++j)
        {
            for (int i = 0; i < image_width; ++i)
            {
                auto pixel_center = pixel00_loc + (i * pixel_delta_u) + (j * pixel_delta_v);
                auto ray_direction = pixel_center - center;

                HMM_Vec3 pixel_color = {0, 0, 0};

                if (samples_per_pixel <= 1)
                {
                    ray r(center, ray_direction);
                    pixel_color = ray_color(r, max_depth, world);
                }
                else
                {
                    for (int sample = 0; sample < samples_per_pixel; ++sample)
                    {
                        ray r = get_ray(i, j);
                        pixel_color += ray_color(r, max_depth, world);
                    }
                    pixel_color = pixel_color * (1.0f / (float)samples_per_pixel);
                }

                // Apply the linear to gamma transform.
                pixel_color.X = linear_to_gamma(pixel_color.X);
                pixel_color.Y = linear_to_gamma(pixel_color.Y);
                pixel_color.Z = linear_to_gamma(pixel_color.Z);

                image_color_data[j * image_width + i] = pixel_color;
            }
        }

        return image_color_data;
    }

private:
    HMM_Vec3 center;         // Camera center
    HMM_Vec3 pixel00_loc;    // Location of pixel 0, 0
    HMM_Vec3 pixel_delta_u;  // Offset to pixel to the right
    HMM_Vec3 pixel_delta_v;  // Offset to pixel below

    void initialize()
    {
        aspect_ratio = (float)image_width / (float)image_height;

        center = HMM_Vec3{0, 0, 0};

        // Determine viewport dimensions.
        float focal_length = 1.0;
        float viewport_height = 2.0;
        float viewport_width = viewport_height * ((float)image_width/(float)image_height);

        // Calculate the vectors across the horizontal and down the vertical viewport edges.
        auto viewport_u = HMM_Vec3{viewport_width, 0, 0};
        auto viewport_v = HMM_Vec3{0, -viewport_height, 0};

        // Calculate the horizontal and vertical delta vectors from pixel to pixel.
        pixel_delta_u = viewport_u / (float)image_width;
        pixel_delta_v = viewport_v / (float)image_height;

        // Calculate the location of the upper left pixel.
        HMM_Vec3 viewport_upper_left = center - HMM_Vec3{0, 0, focal_length} - viewport_u/2 - viewport_v/2;
        pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);
    }

    ray get_ray(int i, int j) const
    {
        // Get a randomly sampled camera ray for the pixel at location i,j.
        auto pixel_center = pixel00_loc + (i * pixel_delta_u) + (j * pixel_delta_v);
        auto pixel_sample = pixel_center + pixel_sample_square();

        auto ray_origin = center;
        auto ray_direction = pixel_sample - ray_origin;

        return ray(ray_origin, ray_direction);
    }

    HMM_Vec3 pixel_sample_square() const
    {
        // Returns a random point in the square surrounding a pixel at the origin.
        auto px = -0.5f + random_float();
        auto py = -0.5f + random_float();
        return (px * pixel_delta_u) + (py * pixel_delta_v);
    }

    HMM_Vec3 ray_color(const ray& r, int depth, const hittable& world) const
    {
        // If we've exceeded the ray bounce limit, no more light is gathered.
        if (depth <= 0)
            return HMM_Vec3{0,0,0};

        hit_record rec;
        if (world.hit(r, interval(0.001, infinity), rec))
        {
            ray scattered;
            HMM_Vec3 attenuation;
            if (rec.mat->scatter(r, rec, attenuation, scattered))
                return attenuation * ray_color(scattered, depth-1, world);
            return HMM_Vec3{0, 0, 0};
        }

        HMM_Vec3 unit_direction = HMM_Norm(r.direction());
        float a = 0.5f * (unit_direction.Y + 1.0f);
        return HMM_Lerp(HMM_Vec3{1.0f, 1.0f, 1.0f}, a, HMM_Vec3{0.5f, 0.7f, 1.0f});
    }
};

int main()
{
    auto timeStart = std::chrono::high_resolution_clock::now();

    hittable_list world;

    auto material_ground = std::make_shared<lambertian>(HMM_Vec3{0.8, 0.8, 0.0});
    auto material_center = std::make_shared<lambertian>(HMM_Vec3{0.1, 0.2, 0.5});
    auto material_left   = std::make_shared<dielectric>(1.5);
    auto material_right  = std::make_shared<metal>(HMM_Vec3{0.8, 0.6, 0.2}, 0.0);

    world.add(std::make_shared<sphere>(HMM_Vec3{ 0.0, -100.5, -1.0}, 100.0, material_ground));
    world.add(std::make_shared<sphere>(HMM_Vec3{ 0.0,    0.0, -1.0},   0.5, material_center));
    world.add(std::make_shared<sphere>(HMM_Vec3{-1.0,    0.0, -1.0},   0.5, material_left));
    world.add(std::make_shared<sphere>(HMM_Vec3{ 1.0,    0.0, -1.0},   0.5, material_right));

    camera cam;

    cam.image_width  = 640;
    cam.image_height = 360;
    cam.aspect_ratio = 16.0f / 9.0f;
    cam.max_depth    = 50;

    std::vector<HMM_Vec3> image_color_data = cam.render(world);

    save_jpg(cam.image_width, cam.image_height, image_color_data, "output.jpg");

    auto timeEnd = std::chrono::high_resolution_clock::now();

    uint64_t milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(timeEnd - timeStart).count();
    std::cout << "Time taken: " << milliseconds << "ms" << std::endl;
    return 0;
}
