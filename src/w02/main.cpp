// #define USE_PARALLEL_FOR 0
#define USE_PARALLEL_FOR 1

#include "misc.h"
#include "ray.h"
#include "hit_record.h"
#include "hittable.h"
#include "material.h"
#include "camera.h"
#include "aabb.h"

#include <iostream>
#include <chrono>
#include <memory>
#include <cmath>
#include <cstdlib>

int main()
{
    auto timeStart = std::chrono::high_resolution_clock::now();

    hittable_list world;

    auto ground_material = std::make_shared<lambertian>(HMM_Vec3{0.5, 0.5, 0.5});
    world.add(std::make_shared<sphere>(HMM_Vec3{0,-1000,0}, 1000, ground_material));

    for (int a = -11; a < 11; a++) {
        for (int b = -11; b < 11; b++) {
            auto choose_mat = random_float();
            HMM_Vec3 center = {a + 0.9f*random_float(), 0.2, b + 0.9f*random_float()};

            if (HMM_Len(center - HMM_Vec3{4, 0.2, 0}) > 0.9) {
                std::shared_ptr<material> sphere_material;

                if (choose_mat < 0.8) {
                    // diffuse
                    auto albedo = Vec3::random_vec3() * Vec3::random_vec3();
                    sphere_material = std::make_shared<lambertian>(albedo);
                    // auto center2 = center + HMM_V3(0, random_float(0,.5), 0);
                    // world.add(std::make_shared<sphere>(center, center2, 0.2, sphere_material));
                    world.add(std::make_shared<sphere>(center, 0.2, sphere_material));
                } else if (choose_mat < 0.95) {
                    // metal
                    auto albedo = Vec3::random_vec3(0.5, 1);
                    auto fuzz = random_float(0, 0.5);
                    sphere_material = std::make_shared<metal>(albedo, fuzz);
                    world.add(std::make_shared<sphere>(center, 0.2, sphere_material));
                } else {
                    // glass
                    sphere_material = std::make_shared<dielectric>(1.5);
                    world.add(std::make_shared<sphere>(center, 0.2, sphere_material));
                }
            }
        }
    }

    auto material1 = std::make_shared<dielectric>(1.5);
    world.add(std::make_shared<sphere>(HMM_Vec3{0, 1, 0}, 1.0, material1));

    auto material2 = std::make_shared<lambertian>(HMM_Vec3{0.4, 0.2, 0.1});
    world.add(std::make_shared<sphere>(HMM_Vec3{-4, 1, 0}, 1.0, material2));

    auto material3 = std::make_shared<metal>(HMM_Vec3{0.7, 0.6, 0.5}, 0.0);
    world.add(std::make_shared<sphere>(HMM_Vec3{4, 1, 0}, 1.0, material3));

    world = hittable_list(std::make_shared<bvh_node>(world));

    camera cam;

    cam.image_width  = 640;
    cam.image_height = 360;
    cam.aspect_ratio = 16.0f / 9.0f;
    cam.max_depth    = 50;
    cam.samples_per_pixel = 500;

    cam.vfov     = 20.0f;
    cam.lookfrom = HMM_Vec3{13,2,3};
    cam.lookat   = HMM_Vec3{ 0,0,0};
    cam.vup      = HMM_Vec3{ 0,1, 0};

    cam.defocus_angle = 0.6f;
    cam.focus_dist    = 10.0f;

    std::vector<HMM_Vec3> image_color_data = cam.render(world);

    save_jpg(cam.image_width, cam.image_height, image_color_data, "output.jpg");

    auto timeEnd = std::chrono::high_resolution_clock::now();

    uint64_t milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(timeEnd - timeStart).count();
    std::cout << "Time taken: " << milliseconds << "ms" << std::endl;
    return 0;
}
