#include "HandmadeMath.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <iostream>
#include <vector>

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

HMM_Vec3 ray_color(const ray& r) {
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
    HMM_Vec3 pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

    for (int j = 0; j < image_height; ++j) {
        for (int i = 0; i < image_width; ++i) {
            auto pixel_center = pixel00_loc + (i * pixel_delta_u) + (j * pixel_delta_v);
            auto ray_direction = pixel_center - camera_center;
            ray r(camera_center, ray_direction);

            HMM_Vec3 pixel_color = ray_color(r);

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
