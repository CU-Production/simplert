#ifndef SIMPLERT_PERLIN_H
#define SIMPLERT_PERLIN_H

#include "misc.h"

class perlin
{
public:
    perlin() {
        ranfloat = new float[point_count];
        for (int i = 0; i < point_count; i++) {
            ranfloat[i] = random_float();
        }

        perm_x = perlin_generate_perm();
        perm_y = perlin_generate_perm();
        perm_z = perlin_generate_perm();
    }

    ~perlin() {
        delete[] ranfloat;
        delete[] perm_x;
        delete[] perm_y;
        delete[] perm_z;
    }

    float noise(const HMM_Vec3& p) const {
        auto u = p.X - std::floorf(p.X);
        auto v = p.Y - std::floorf(p.Y);
        auto w = p.Z - std::floorf(p.Z);
        u = u*u*(3-2*u);
        v = v*v*(3-2*v);
        w = w*w*(3-2*w);

        auto i = static_cast<int>(std::floorf(p.X));
        auto j = static_cast<int>(std::floorf(p.Y));
        auto k = static_cast<int>(std::floorf(p.Z));
        float c[2][2][2];

        for (int di=0; di < 2; di++)
            for (int dj=0; dj < 2; dj++)
                for (int dk=0; dk < 2; dk++)
                    c[di][dj][dk] = ranfloat[
                            perm_x[(i+di) & 255] ^
                            perm_y[(j+dj) & 255] ^
                            perm_z[(k+dk) & 255]
                    ];

        return trilinear_interp(c, u, v, w);
    }

private:
    static const int point_count = 256;
    float* ranfloat;
    int* perm_x;
    int* perm_y;
    int* perm_z;

    static int* perlin_generate_perm() {
        auto p = new int[point_count];

        for (int i = 0; i < perlin::point_count; i++)
            p[i] = i;

        permute(p, point_count);

        return p;
    }

    static void permute(int* p, int n) {
        for (int i = n-1; i > 0; i--) {
            int target = random_int(0, i);
            int tmp = p[i];
            p[i] = p[target];
            p[target] = tmp;
        }
    }

    static float trilinear_interp(float c[2][2][2], float u, float v, float w) {
        float accum = 0.0f;
        for (int i=0; i < 2; i++)
            for (int j=0; j < 2; j++)
                for (int k=0; k < 2; k++)
                    accum += (i*u + (1-i)*(1-u))*
                             (j*v + (1-j)*(1-v))*
                             (k*w + (1-k)*(1-w))*c[i][j][k];

        return accum;
    }
};

#endif //SIMPLERT_PERLIN_H
