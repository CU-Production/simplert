add_executable(w02
        aabb.h
        camera.h
        hit_record.h
        hittable.h
        material.h
        misc.h
        ray.h
        rtw_image.h
        perlin.h
        main.cpp)
target_link_libraries(w02 PRIVATE stb hmm parallel-util)
