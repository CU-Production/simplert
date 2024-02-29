#ifndef SIMPLERT_CAMERA_H
#define SIMPLERT_CAMERA_H

#include "parallel-util.hpp"

#include "misc.h"
#include "hittable.h"
#include "material.h"
#include "ray.h"

#ifndef USE_PARALLEL_FOR
#define USE_PARALLEL_FOR 1
#endif

class camera
{
public:
    float aspect_ratio      = 1.0f;  // Ratio of image width over height
    int   image_width       = 100;   // Rendered image width in pixel count
    int   image_height      = 100;   // Rendered image height
    int   samples_per_pixel = 10;    // Count of random samples for each pixel
    int   max_depth         = 10;    // Maximum number of ray bounces into scene

    float vfov        = 90.0f;            // Vertical view angle (field of view)
    HMM_Vec3 lookfrom = {0,0,-1};  // Point camera is looking from
    HMM_Vec3 lookat   = {0,0,0};   // Point camera is looking at
    HMM_Vec3 vup      = {0,1,0};   // Camera-relative "up" direction

    float defocus_angle = 0;  // Variation angle of rays through each pixel
    float focus_dist    = 10; // Distance from camera lookfrom point to plane of perfect focus

    std::vector<HMM_Vec3> render(const hittable& world)
    {
        initialize();

        std::vector<HMM_Vec3> image_color_data;
        image_color_data.resize(image_width * image_height, {0, 0, 0});

        auto render_func = [&](int i, int j){
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
        };

#if USE_PARALLEL_FOR
        parallelutil::parallel_for_2d(image_width, image_height, [&](int i, int j){
            render_func(i, j);
        });
#else
        for (int j = 0; j < image_height; ++j)
        {
            for (int i = 0; i < image_width; ++i)
            {
                render_func(i, j);
            }
        }
#endif

        return image_color_data;
    }

private:
    HMM_Vec3 center;          // Camera center
    HMM_Vec3 pixel00_loc;     // Location of pixel 0, 0
    HMM_Vec3 pixel_delta_u;   // Offset to pixel to the right
    HMM_Vec3 pixel_delta_v;   // Offset to pixel below
    HMM_Vec3 u, v, w;         // Camera frame basis vectors
    HMM_Vec3 defocus_disk_u;  // Defocus disk horizontal radius
    HMM_Vec3 defocus_disk_v;  // Defocus disk vertical radius

    void initialize()
    {
        aspect_ratio = (float)image_width / (float)image_height;

        center = lookfrom;

        // Determine viewport dimensions.
        float theta = vfov * HMM_DegToRad;
        float h = std::tanf(theta/2.0f);
        float viewport_height = 2.0f * h * focus_dist;
        float viewport_width = viewport_height * ((float)image_width/(float)image_height);

        // Calculate the u,v,w unit basis vectors for the camera coordinate frame.
        w = HMM_Norm(lookfrom - lookat);
        u = HMM_Norm(HMM_Cross(vup, w));
        v = HMM_Cross(w, u);

        // Calculate the vectors across the horizontal and down the vertical viewport edges.
        HMM_Vec3 viewport_u = viewport_width * u;    // Vector across viewport horizontal edge
        HMM_Vec3 viewport_v = viewport_height * -v;  // Vector down viewport vertical edge

        // Calculate the horizontal and vertical delta vectors from pixel to pixel.
        pixel_delta_u = viewport_u / (float)image_width;
        pixel_delta_v = viewport_v / (float)image_height;

        // Calculate the location of the upper left pixel.
        HMM_Vec3 viewport_upper_left = center - (focus_dist * w) - viewport_u/2 - viewport_v/2;
        pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

        // Calculate the camera defocus disk basis vectors.
        auto defocus_radius = focus_dist * tan((defocus_angle / 2) * HMM_DegToRad);
        defocus_disk_u = u * defocus_radius;
        defocus_disk_v = v * defocus_radius;
    }

    ray get_ray(int i, int j) const
    {
        // Get a randomly-sampled camera ray for the pixel at location i,j, originating from
        // the camera defocus disk.
        auto pixel_center = pixel00_loc + (i * pixel_delta_u) + (j * pixel_delta_v);
        auto pixel_sample = pixel_center + pixel_sample_square();

        auto ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
        auto ray_direction = pixel_sample - ray_origin;
        auto ray_time = random_float();

        return ray(ray_origin, ray_direction, ray_time);
    }

    HMM_Vec3 pixel_sample_square() const
    {
        // Returns a random point in the square surrounding a pixel at the origin.
        auto px = -0.5f + random_float();
        auto py = -0.5f + random_float();
        return (px * pixel_delta_u) + (py * pixel_delta_v);
    }

    HMM_Vec3 defocus_disk_sample() const
    {
        // Returns a random point in the camera defocus disk.
        auto p = Vec3::random_in_unit_disk();
        return center + (p.X * defocus_disk_u) + (p.Y * defocus_disk_v);
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

#endif //SIMPLERT_CAMERA_H
