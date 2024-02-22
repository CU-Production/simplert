#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <iostream>
#include <vector>

int main()
{
    // Image

    int image_width = 256;
    int image_height = 256;

    // Render
    std::vector<uint8_t> image_data;
    image_data.reserve(image_width * image_height * 3);
//    image_data.resize(image_width * image_height * 3, 0);

    for (int j = 0; j < image_height; ++j) {
        for (int i = 0; i < image_width; ++i) {
            auto r = double(i) / (image_width-1);
            auto g = double(j) / (image_height-1);
            auto b = 0;

            int ir = static_cast<int>(255.999 * r);
            int ig = static_cast<int>(255.999 * g);
            int ib = static_cast<int>(255.999 * b);

            image_data.push_back(ir);
            image_data.push_back(ig);
            image_data.push_back(ib);
        }
    }

    stbi_write_jpg("output.jpg", image_width, image_height, 3, image_data.data(), 100);

    return 0;
}
