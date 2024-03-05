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

    world.add(std::make_shared<quad>(HMM_V3(555,0,0), HMM_V3(0,555,0), HMM_V3(0,0,555), green));
    world.add(std::make_shared<quad>(HMM_V3(0,0,0), HMM_V3(0,555,0), HMM_V3(0,0,555), red));
    world.add(std::make_shared<quad>(HMM_V3(343, 554, 332), HMM_V3(-130,0,0), HMM_V3(0,0,-105), light));
    world.add(std::make_shared<quad>(HMM_V3(0,0,0), HMM_V3(555,0,0), HMM_V3(0,0,555), white));
    world.add(std::make_shared<quad>(HMM_V3(555,555,555), HMM_V3(-555,0,0), HMM_V3(0,0,-555), white));
    world.add(std::make_shared<quad>(HMM_V3(0,0,555), HMM_V3(555,0,0), HMM_V3(0,555,0), white));

    std::shared_ptr<hittable> box1 = box(HMM_V3(0,0,0), HMM_V3(165,330,165), white);
    box1 = std::make_shared<rotate_y>(box1, 15);
    box1 = std::make_shared<translate>(box1, HMM_V3(265,0,295));
    world.add(box1);

    std::shared_ptr<hittable> box2 = box(HMM_V3(0,0,0), HMM_V3(165,165,165), white);
    box2 = std::make_shared<rotate_y>(box2, -18);
    box2 = std::make_shared<translate>(box2, HMM_V3(130,0,65));
    world.add(box2);

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

void cornell_smoke() {
    hittable_list world;

    auto red   = std::make_shared<lambertian>(HMM_V3(.65, .05, .05));
    auto white = std::make_shared<lambertian>(HMM_V3(.73, .73, .73));
    auto green = std::make_shared<lambertian>(HMM_V3(.12, .45, .15));
    auto light = std::make_shared<diffuse_light>(HMM_V3(7, 7, 7));

    world.add(std::make_shared<quad>(HMM_V3(555,0,0), HMM_V3(0,555,0), HMM_V3(0,0,555), green));
    world.add(std::make_shared<quad>(HMM_V3(0,0,0), HMM_V3(0,555,0), HMM_V3(0,0,555), red));
    world.add(std::make_shared<quad>(HMM_V3(113,554,127), HMM_V3(330,0,0), HMM_V3(0,0,305), light));
    world.add(std::make_shared<quad>(HMM_V3(0,555,0), HMM_V3(555,0,0), HMM_V3(0,0,555), white));
    world.add(std::make_shared<quad>(HMM_V3(0,0,0), HMM_V3(555,0,0), HMM_V3(0,0,555), white));
    world.add(std::make_shared<quad>(HMM_V3(0,0,555), HMM_V3(555,0,0), HMM_V3(0,555,0), white));

    std::shared_ptr<hittable> box1 = box(HMM_V3(0,0,0), HMM_V3(165,330,165), white);
    box1 = std::make_shared<rotate_y>(box1, 15);
    box1 = std::make_shared<translate>(box1, HMM_V3(265,0,295));

    std::shared_ptr<hittable> box2 = box(HMM_V3(0,0,0), HMM_V3(165,165,165), white);
    box2 = std::make_shared<rotate_y>(box2, -18);
    box2 = std::make_shared<translate>(box2, HMM_V3(130,0,65));

    world.add(std::make_shared<constant_medium>(box1, 0.01, HMM_V3(0,0,0)));
    world.add(std::make_shared<constant_medium>(box2, 0.01, HMM_V3(1,1,1)));

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
    save_jpg(cam.image_width, cam.image_height, image_color_data, "cornell_smoke.jpg");
}

void final_scene(int image_width, int samples_per_pixel, int max_depth) {
    hittable_list boxes1;
    auto ground = std::make_shared<lambertian>(HMM_V3(0.48, 0.83, 0.53));

    int boxes_per_side = 20;
    for (int i = 0; i < boxes_per_side; i++) {
        for (int j = 0; j < boxes_per_side; j++) {
            auto w = 100.0;
            auto x0 = -1000.0 + i*w;
            auto z0 = -1000.0 + j*w;
            auto y0 = 0.0;
            auto x1 = x0 + w;
            auto y1 = random_float(1,101);
            auto z1 = z0 + w;

            boxes1.add(box(HMM_V3(x0,y0,z0), HMM_V3(x1,y1,z1), ground));
        }
    }

    hittable_list world;

    world.add(std::make_shared<bvh_node>(boxes1));

    auto light = std::make_shared<diffuse_light>(HMM_V3(7, 7, 7));
    world.add(std::make_shared<quad>(HMM_V3(123,554,147), HMM_V3(300,0,0), HMM_V3(0,0,265), light));

    auto center1 = HMM_V3(400, 400, 200);
    auto center2 = center1 + HMM_V3(30,0,0);
    auto sphere_material = std::make_shared<lambertian>(HMM_V3(0.7, 0.3, 0.1));
    world.add(std::make_shared<sphere>(center1, center2, 50, sphere_material));

    world.add(std::make_shared<sphere>(HMM_V3(260, 150, 45), 50, std::make_shared<dielectric>(1.5)));
    world.add(std::make_shared<sphere>(
        HMM_V3(0, 150, 145), 50, std::make_shared<metal>(HMM_V3(0.8, 0.8, 0.9), 1.0)
    ));

    auto boundary = std::make_shared<sphere>(HMM_V3(360,150,145), 70, std::make_shared<dielectric>(1.5));
    world.add(boundary);
    world.add(std::make_shared<constant_medium>(boundary, 0.2, HMM_V3(0.2, 0.4, 0.9)));
    boundary = std::make_shared<sphere>(HMM_V3(0,0,0), 5000, std::make_shared<dielectric>(1.5));
    world.add(std::make_shared<constant_medium>(boundary, .0001, HMM_V3(1,1,1)));

    auto emat = std::make_shared<lambertian>(std::make_shared<image_texture>("earthmap.jpg"));
    world.add(std::make_shared<sphere>(HMM_V3(400,200,400), 100, emat));
    auto pertext = std::make_shared<noise_texture>(0.1);
    world.add(std::make_shared<sphere>(HMM_V3(220,280,300), 80, std::make_shared<lambertian>(pertext)));

    hittable_list boxes2;
    auto white = std::make_shared<lambertian>(HMM_V3(.73, .73, .73));
    int ns = 1000;
    for (int j = 0; j < ns; j++) {
        boxes2.add(std::make_shared<sphere>(Vec3::random_vec3(0,165), 10, white));
    }

    world.add(std::make_shared<translate>(
        std::make_shared<rotate_y>(
            std::make_shared<bvh_node>(boxes2), 15),
            HMM_V3(-100,270,395)
        )
    );

    camera cam;

    cam.aspect_ratio      = 1.0;
    cam.image_width       = image_width;
    cam.image_height      = image_width;
    cam.samples_per_pixel = samples_per_pixel;
    cam.max_depth         = max_depth;
    cam.background        = HMM_V3(0,0,0);

    cam.vfov     = 40;
    cam.lookfrom = HMM_V3(478, 278, -600);
    cam.lookat   = HMM_V3(278, 278, 0);
    cam.vup      = HMM_V3(0,1,0);

    cam.defocus_angle = 0;

    std::vector<HMM_Vec3> image_color_data = cam.render(world);
    std::string final_scene_name = std::format("final_scene_spp{}.jpg", samples_per_pixel);
    save_jpg(cam.image_width, cam.image_height, image_color_data, final_scene_name.c_str());
}

int main()
{
    auto timeStart = std::chrono::high_resolution_clock::now();

    switch (10) {
        case 1: random_spheres();      break;
        case 2: two_spheres();         break;
        case 3: earth();               break;
        case 4: two_perlin_spheres();  break;
        case 5: quads();               break;
        case 6: simple_light();        break;
        case 7: cornell_box();         break;
        case 8: cornell_smoke();       break;
        case 9:  final_scene(256, 10000, 40); break;
        case 10:  final_scene(256, 1000, 40); break;
        default: final_scene(256,   250,  4); break;
    }

    auto timeEnd = std::chrono::high_resolution_clock::now();

    uint64_t milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(timeEnd - timeStart).count();
    std::cout << "Time taken: " << milliseconds << "ms" << std::endl;
    return 0;
}
