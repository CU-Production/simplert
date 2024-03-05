// #define USE_PARALLEL_FOR 0
#define USE_PARALLEL_FOR 1

#include "misc.h"
#include "ray.h"
#include "hit_record.h"
#include "hittable.h"
#include "material.h"
#include "camera.h"
#include "aabb.h"
#include "texture.h"

#include <iostream>
#include <chrono>
#include <memory>
#include <cmath>
#include <cstdlib>

void random_spheres()
{
    hittable_list world;

//    auto ground_material = std::make_shared<lambertian>(HMM_Vec3{0.5, 0.5, 0.5});
//    world.add(std::make_shared<sphere>(HMM_Vec3{0,-1000,0}, 1000, ground_material));

    auto checker = std::make_shared<checker_texture>(0.32, HMM_V3(.2, .3, .1), HMM_V3(.9, .9, .9));
    world.add(make_shared<sphere>(HMM_V3(0,-1000,0), 1000, std::make_shared<lambertian>(checker)));

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

    save_jpg(cam.image_width, cam.image_height, image_color_data, "random_spheres.jpg");
}

void two_spheres()
{
    hittable_list world;

    auto checker = std::make_shared<checker_texture>(0.8, HMM_V3(.2, .3, .1), HMM_V3(.9, .9, .9));

    world.add(std::make_shared<sphere>(HMM_V3(0,-10, 0), 10, std::make_shared<lambertian>(checker)));
    world.add(std::make_shared<sphere>(HMM_V3(0, 10, 0), 10, std::make_shared<lambertian>(checker)));

    camera cam;

    cam.image_width  = 640;
    cam.image_height = 360;
    cam.aspect_ratio = 16.0f / 9.0f;
    cam.max_depth    = 50;

    cam.vfov     = 20;
    cam.lookfrom = HMM_V3(13,2,3);
    cam.lookat   = HMM_V3(0,0,0);
    cam.vup      = HMM_V3(0,1,0);

    cam.defocus_angle = 0;

    std::vector<HMM_Vec3> image_color_data = cam.render(world);

    save_jpg(cam.image_width, cam.image_height, image_color_data, "two_spheres.jpg");
}

void earth()
{
    auto earth_texture = std::make_shared<image_texture>("earthmap.jpg");
    auto earth_surface = std::make_shared<lambertian>(earth_texture);
    auto globe = make_shared<sphere>(HMM_V3(0,0,0), 2, earth_surface);

    camera cam;

    cam.image_width  = 640;
    cam.image_height = 360;
    cam.aspect_ratio = 16.0f / 9.0f;
    cam.max_depth    = 50;

    cam.vfov     = 20;
    cam.lookfrom = HMM_V3(0,0,12);
    cam.lookat   = HMM_V3(0,0,0);
    cam.vup      = HMM_V3(0,1,0);

    cam.defocus_angle = 0;

    std::vector<HMM_Vec3> image_color_data = cam.render(hittable_list(globe));
    save_jpg(cam.image_width, cam.image_height, image_color_data, "earth.jpg");
}

void two_perlin_spheres()
{
    hittable_list world;

    auto pertext = std::make_shared<noise_texture>(4);
    world.add(std::make_shared<sphere>(HMM_V3(0,-1000,0), 1000, std::make_shared<lambertian>(pertext)));
    world.add(std::make_shared<sphere>(HMM_V3(0,2,0), 2, std::make_shared<lambertian>(pertext)));

    camera cam;

    cam.image_width  = 640;
    cam.image_height = 360;
    cam.aspect_ratio = 16.0f / 9.0f;
    cam.max_depth    = 50;

    cam.vfov     = 20;
    cam.lookfrom = HMM_V3(13,2,3);
    cam.lookat   = HMM_V3(0,0,0);
    cam.vup      = HMM_V3(0,1,0);

    cam.defocus_angle = 0;

    std::vector<HMM_Vec3> image_color_data = cam.render(world);
    save_jpg(cam.image_width, cam.image_height, image_color_data, "two_perlin_spheres.jpg");
}

void quads() {
    hittable_list world;

    // Materials
    auto left_red     = std::make_shared<lambertian>(HMM_V3(1.0, 0.2, 0.2));
    auto back_green   = std::make_shared<lambertian>(HMM_V3(0.2, 1.0, 0.2));
    auto right_blue   = std::make_shared<lambertian>(HMM_V3(0.2, 0.2, 1.0));
    auto upper_orange = std::make_shared<lambertian>(HMM_V3(1.0, 0.5, 0.0));
    auto lower_teal   = std::make_shared<lambertian>(HMM_V3(0.2, 0.8, 0.8));

    // Quads
    world.add(std::make_shared<quad>(HMM_V3(-3,-2, 5), HMM_V3(0, 0,-4), HMM_V3(0, 4, 0), left_red));
    world.add(std::make_shared<quad>(HMM_V3(-2,-2, 0), HMM_V3(4, 0, 0), HMM_V3(0, 4, 0), back_green));
    world.add(std::make_shared<quad>(HMM_V3( 3,-2, 1), HMM_V3(0, 0, 4), HMM_V3(0, 4, 0), right_blue));
    world.add(std::make_shared<quad>(HMM_V3(-2, 3, 1), HMM_V3(4, 0, 0), HMM_V3(0, 0, 4), upper_orange));
    world.add(std::make_shared<quad>(HMM_V3(-2,-3, 5), HMM_V3(4, 0, 0), HMM_V3(0, 0,-4), lower_teal));

    camera cam;

    cam.aspect_ratio      = 1.0;
    cam.image_width       = 400;
    cam.image_height      = 400;
    cam.samples_per_pixel = 100;
    cam.max_depth         = 50;

    cam.vfov     = 80;
    cam.lookfrom = HMM_V3(0,0,9);
    cam.lookat   = HMM_V3(0,0,0);
    cam.vup      = HMM_V3(0,1,0);

    cam.defocus_angle = 0;

    std::vector<HMM_Vec3> image_color_data = cam.render(world);
    save_jpg(cam.image_width, cam.image_height, image_color_data, "quads.jpg");
}

void simple_light() {
    hittable_list world;

    auto pertext = std::make_shared<noise_texture>(4);
    world.add(std::make_shared<sphere>(HMM_V3(0,-1000,0), 1000, std::make_shared<lambertian>(pertext)));
    world.add(std::make_shared<sphere>(HMM_V3(0,2,0), 2, std::make_shared<lambertian>(pertext)));

    auto difflight = std::make_shared<diffuse_light>(HMM_V3(4,4,4));
    world.add(std::make_shared<sphere>(HMM_V3(0,7,0), 2, difflight));
    world.add(std::make_shared<quad>(HMM_V3(3,1,-2), HMM_V3(2,0,0), HMM_V3(0,2,0), difflight));

    camera cam;

    cam.image_width       = 640;
    cam.image_height      = 360;
    cam.aspect_ratio      = 16.0f / 9.0f;
    cam.max_depth         = 50;
    cam.samples_per_pixel = 500;
    cam.background        = HMM_V3(0,0,0);

    cam.vfov     = 20;
    cam.lookfrom = HMM_V3(26,3,6);
    cam.lookat   = HMM_V3(0,2,0);
    cam.vup      = HMM_V3(0,1,0);

    cam.defocus_angle = 0;

    std::vector<HMM_Vec3> image_color_data = cam.render(world);
    save_jpg(cam.image_width, cam.image_height, image_color_data, "simple_light.jpg");
}

void cornell_box() {
    hittable_list world;

    auto red   = std::make_shared<lambertian>(HMM_V3(.65, .05, .05));
    auto white = std::make_shared<lambertian>(HMM_V3(.73, .73, .73));
    auto green = std::make_shared<lambertian>(HMM_V3(.12, .45, .15));
    auto light = std::make_shared<diffuse_light>(HMM_V3(15, 15, 15));

    world.add(make_shared<quad>(HMM_V3(555,0,0), HMM_V3(0,555,0), HMM_V3(0,0,555), green));
    world.add(make_shared<quad>(HMM_V3(0,0,0), HMM_V3(0,555,0), HMM_V3(0,0,555), red));
    world.add(make_shared<quad>(HMM_V3(343, 554, 332), HMM_V3(-130,0,0), HMM_V3(0,0,-105), light));
    world.add(make_shared<quad>(HMM_V3(0,0,0), HMM_V3(555,0,0), HMM_V3(0,0,555), white));
    world.add(make_shared<quad>(HMM_V3(555,555,555), HMM_V3(-555,0,0), HMM_V3(0,0,-555), white));
    world.add(make_shared<quad>(HMM_V3(0,0,555), HMM_V3(555,0,0), HMM_V3(0,555,0), white));

    camera cam;

    cam.image_width       = 256;
    cam.image_height      = 256;
    cam.aspect_ratio      = 1.0f;
    cam.samples_per_pixel = 200;
    cam.max_depth         = 50;
    cam.background        = HMM_V3(0,0,0);

    cam.vfov     = 40;
    cam.lookfrom = HMM_V3(278, 278, -800);
    cam.lookat   = HMM_V3(278, 278, 0);
    cam.vup      = HMM_V3(0,1,0);

    cam.defocus_angle = 0;

    std::vector<HMM_Vec3> image_color_data = cam.render(world);
    save_jpg(cam.image_width, cam.image_height, image_color_data, "cornell_box.jpg");
}

int main()
{
    auto timeStart = std::chrono::high_resolution_clock::now();

    switch (7) {
        case 1: random_spheres();      break;
        case 2: two_spheres();         break;
        case 3: earth();               break;
        case 4: two_perlin_spheres();  break;
        case 5: quads();               break;
        case 6: simple_light();        break;
        case 7: cornell_box();         break;
    }

    auto timeEnd = std::chrono::high_resolution_clock::now();

    uint64_t milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(timeEnd - timeStart).count();
    std::cout << "Time taken: " << milliseconds << "ms" << std::endl;
    return 0;
}
