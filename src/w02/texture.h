#ifndef SIMPLERT_TEXTURE_H
#define SIMPLERT_TEXTURE_H

#include "misc.h"
#include "rtw_image.h"

class texture
{
public:
    virtual ~texture() = default;

    virtual HMM_Vec3 value(float u, float v, const HMM_Vec3& p) const = 0;
};

class solid_color : public texture
{
public:
    solid_color(HMM_Vec3 c) : color_value(c) {}
    solid_color(float red, float green, float blue) : solid_color(HMM_V3(red,green,blue)) {}

    HMM_Vec3 value(float u, float v, const HMM_Vec3& p) const override { return color_value; }

private:
    HMM_Vec3 color_value;
};

class checker_texture : public texture
{
public:
    checker_texture(double _scale, std::shared_ptr<texture> _even, std::shared_ptr<texture> _odd) : inv_scale(1.0 / _scale), even(_even), odd(_odd) {}

    checker_texture(double _scale, const HMM_Vec3& c1, const HMM_Vec3& c2) :
        inv_scale(1.0 / _scale),
        even(std::make_shared<solid_color>(c1)),
        odd(std::make_shared<solid_color>(c2))
    {}

    HMM_Vec3 value(float u, float v, const HMM_Vec3& p) const override
    {
        auto xInteger = static_cast<int>(std::floor(inv_scale * p.X));
        auto yInteger = static_cast<int>(std::floor(inv_scale * p.Y));
        auto zInteger = static_cast<int>(std::floor(inv_scale * p.Z));

        bool isEven = (xInteger + yInteger + zInteger) % 2 == 0;

        return isEven ? even->value(u, v, p) : odd->value(u, v, p);
    }

private:
    float inv_scale;
    std::shared_ptr<texture> even;
    std::shared_ptr<texture> odd;
};

class image_texture : public texture
{
public:
    image_texture(const char* filename) : image(filename) {}

    HMM_Vec3 value(float u, float v, const HMM_Vec3& p) const override
    {
        // If we have no texture data, then return solid cyan as a debugging aid.
        if (image.height() <= 0) return HMM_V3(0,1,1);

        // Clamp input texture coordinates to [0,1] x [1,0]
        u = interval(0, 1).clamp(u);
        v = 1.0 - interval(0, 1).clamp(v); // Flip V to image coordinates

        auto i = static_cast<int>(u * image.width());
        auto j = static_cast<int>(v * image.height());
        auto pixel = image.pixel_data(i, j);

        auto color_scale = 1.0f / 255.0f;
        return HMM_V3(color_scale*pixel[0], color_scale*pixel[1], color_scale*pixel[2]);
    }

private:
    rtw_image image;
};

#endif //SIMPLERT_TEXTURE_H
