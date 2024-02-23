#include "HandmadeMath.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <limits>
#include <numbers>

const float infinity = std::numeric_limits<float>::infinity();
const float pi = std::numbers::pi_v<float>;

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

void push_color(std::vector<uint8_t>& image_data, HMM_Vec3 pixel_color)
{
    image_data.push_back(static_cast<int>(255.999 * pixel_color.X));
    image_data.push_back(static_cast<int>(255.999 * pixel_color.Y));
    image_data.push_back(static_cast<int>(255.999 * pixel_color.Z));
}

struct hit_record
{
    HMM_Vec3 p;
    HMM_Vec3 normal;
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

class hittable
{
public:
    virtual ~hittable() = default;
    virtual bool hit(const ray& r, float ray_tmin, float ray_tmax, hit_record& rec) const = 0;
};

class sphere : public hittable
{
public:
    sphere(HMM_Vec3 _center, float _radius) : center(_center), radius(_radius) {}

    bool hit(const ray& r, float ray_tmin, float ray_tmax, hit_record& rec) const override
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
        if (root <= ray_tmin || ray_tmax <= root)
        {
            root = (-half_b + sqrtd) / a;
            if (root <= ray_tmin || ray_tmax <= root)
                return false;
        }

        rec.t = root;
        rec.p = r.at(rec.t);
        HMM_Vec3 outward_normal = (rec.p - center) / radius;
        rec.set_face_normal(r, outward_normal);

        return true;
    }

private:
    HMM_Vec3 center;
    float radius;
};

class hittable_list : public hittable
{
public:
    std::vector<std::shared_ptr<hittable>> objects;

    hittable_list() {}
    hittable_list(std::shared_ptr<hittable> object) { add(object); }

    void clear() { objects.clear(); }
    void add(std::shared_ptr<hittable> object) { objects.push_back(object); }

    bool hit(const ray& r, float ray_tmin, float ray_tmax, hit_record& rec) const override
    {
        hit_record temp_rcc;
        bool hit_anything = false;
        float closest_so_far = ray_tmax;

        for (const auto& object : objects)
        {
            if (object->hit(r, ray_tmin, closest_so_far, temp_rcc))
            {
                hit_anything = true;
                closest_so_far = temp_rcc.t;
                rec = temp_rcc;
            }
        }

        return hit_anything;
    }
};

HMM_Vec3 ray_color(const ray& r, const hittable& world)
{
    hit_record rec;
    if (world.hit(r, 0, infinity, rec))
    {
        return 0.5 * (rec.normal + HMM_Vec3{1.0f, 1.0f, 1.0f});
    }

    HMM_Vec3 unit_direction = HMM_Norm(r.direction());
    float a = 0.5f * (unit_direction.Y + 1.0f);
    return HMM_Lerp(HMM_Vec3{1.0f, 1.0f, 1.0f}, a, HMM_Vec3{0.5f, 0.7f, 1.0f});
}

int main()
{
    // Image
    const int image_width = 640;
    const int image_height = 360;
    const float aspect_ratio = 16.0f / 9.0f;

    std::vector<uint8_t> image_data;
    image_data.reserve(image_width * image_height * 3);

    std::vector<HMM_Vec3> image_color_data;
    image_color_data.resize(image_width * image_height, {0, 0, 0});

    // World
    hittable_list world;
    world.add(std::make_shared<sphere>(HMM_Vec3{0, 0, -1}, 0.5));
    world.add(std::make_shared<sphere>(HMM_Vec3{0, -100.5, -1}, 100));

    // Camera
    float focal_length = 1.0f;
    float viewport_height = 2.0f;
    float viewport_width = viewport_height * (static_cast<float>(image_width)/image_height);
    HMM_Vec3 camera_center = {0, 0, 0};

    // Calculate the vectors across the horizontal and down the vertical viewport edges.
    HMM_Vec3 viewport_u = {viewport_width, 0, 0};
    HMM_Vec3 viewport_v = {0, -viewport_height, 0};

    // Calculate the horizontal and vertical delta vectors from pixel to pixel.
    HMM_Vec3 pixel_delta_u = viewport_u / image_width;
    HMM_Vec3 pixel_delta_v = viewport_v / image_height;

    // Calculate the location of the upper left pixel.
    HMM_Vec3 viewport_upper_left = camera_center - HMM_Vec3{0, 0, focal_length} - viewport_u/2 - viewport_v/2;
    HMM_Vec3 pixel00_loc = viewport_upper_left + 0.5f * (pixel_delta_u + pixel_delta_v);

    for (int j = 0; j < image_height; ++j)
    {
        for (int i = 0; i < image_width; ++i)
        {
            auto pixel_center = pixel00_loc + (i * pixel_delta_u) + (j * pixel_delta_v);
            auto ray_direction = pixel_center - camera_center;
            ray r(camera_center, ray_direction);

            HMM_Vec3 pixel_color = ray_color(r, world);

            image_color_data[j * image_width + i] = pixel_color;
        }
    }

    for (int i = 0; i < image_height * image_width; i++)
    {
        push_color(image_data, image_color_data[i]);
    }

    stbi_write_jpg("output.jpg", image_width, image_height, 3, image_data.data(), 100);

    return 0;
}
