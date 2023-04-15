#include "utilities_package.h"

//#include "aabb.h"
//#include "aarect.h"
#include "aabb_minmax_box.h"
#include "colour.h"
#include "collision.h"
#include "collision_list.h"
#include "eye.h"
#include "material.h"
#include "plane_rects.h"
#include "sphere.h"
#include "texture.h"
#include "vec3.h"

#include <iostream>
#include <cstdlib>
#include <thread>
#include <vector>
#include <functional>
#include <chrono>
#include <omp.h>
#include <time.h>
#include <cmath>
#include <string>
#include <fstream>
#include <cmath>

using namespace std::chrono;

int image_width;
int samples_per_pixel;
int max_depth;
int image_height;

int global_x, global_k, global_z;
int light_x0, light_x1, light_y0, light_y1, light_k;

//collision_list world;
//colour background;
//colour pixel_colour(0, 0, 0);
//eye camera;


colour ray_colour(const ray& r, const colour& background, const collision& world, int depth) {
    
    collision_record record;

    // If reached max depth, won't branch out and get more ray light
    if (depth <= 0)
        return colour(0, 0, 0);

    // If ray hits nothing, return background colour;
    if (!world.hit(r, 0.001, infinity, record)) 
        return background;

    ray scattered;
    colour attenuation;
    colour emitted = record.mat_ptr->emitted(record.u, record.v, record.p);

    if (!record.mat_ptr->scatter(r, record, attenuation, scattered))
        return emitted;

    return emitted + attenuation * ray_colour(scattered, background, world, depth - 1);

}


collision_list preset_scene() {

    collision_list objects;

    auto red = make_shared<lambertian>(colour(.65, .05, .05));
    auto white = make_shared<lambertian>(colour(.73, .73, .73));
    auto green = make_shared<lambertian>(colour(.12, .45, .15));
    auto blue = make_shared<lambertian>(colour(0.196, 0.196, 0.658));
    auto light = make_shared<diffuse_light>(colour(25, 25, 25));
    auto metal_material = make_shared<metal>(colour(0.7, 0.6, 0.5), 0.0);

    light_x0 = -1;
    light_x1 =  1;
    light_y0 = -1;
    light_y1 =  1;
    global_k = light_k = 5;

    // make the light source
    objects.add(make_shared<xz_rect>(light_x0, light_x1, light_y0, light_y1, light_k, light));

    // make the ground plane
    objects.add(make_shared<xz_rect>(-555, 555, -555, 555, 0, green));

    // add three spheres
    objects.add(make_shared<sphere>(coordinate(0, 1, -1), 1.0, metal_material));

    objects.add(make_shared<sphere>(coordinate(2, 0.75, -2), 0.75, red));
    objects.add(make_shared<sphere>(coordinate(3, 0.75, 2), 0.75, blue));

    return objects;

}


void base_ray_trace(int image_width, int samples_per_pixel, int max_depth, std::string filename) {

    // start by creating new file with filename
    std::ofstream output_file(filename);

    //int global_x, global_k, global_z;
    //int light_x0, light_x1, light_y0, light_y1, light_k;

    // image
    auto aspect_ratio = 16.0 / 9.0;
    int image_height = static_cast<int>(image_width / aspect_ratio);

    // World
    collision_list world = preset_scene();
    coordinate look_from = coordinate(13, 2, 3);
    coordinate look_at = coordinate(0, 0, 0);
    auto v_fov = 40.0;

    // choose either blue or black
    //background = colour(0, 0, 0);
    colour background = colour(0.0823, 0.6745, 0.9294);

    // Eye
    const vec3 v_up(0, 1, 0);
    const auto distance_to_focus = 10.0;
    auto aperture = 0.0;

    eye camera(look_from, look_at, v_up, v_fov, aspect_ratio, aperture, distance_to_focus, 0.0, 1.0);

    // ppm file header
    output_file << "P3\n" << image_width << ' ' << image_height << "\n255\n";

    for (int j = image_height - 1; j >= 0; --j) {

        //std::cerr << "\rLines remaining: " << j << ' ' << std::flush;

        for (int i = 0; i < image_width; ++i) {

            colour pixel_colour(0, 0, 0);

            for (int s = 0; s < samples_per_pixel; ++s) {

                auto u = (i + random_double()) / (image_width - 1);
                auto v = (j + random_double()) / (image_height - 1);
                ray r = camera.get_ray(u, v);
                pixel_colour += ray_colour(r, background, world, max_depth);

            }

            write_colour(output_file, pixel_colour, samples_per_pixel);

        }

    }

    output_file.close();

}

void old_omp_solution(int image_width, int samples_per_pixel, int max_depth, std::string filename) {

    // start by creating new file with filename
    std::ofstream output_file(filename);
    //output_file.open(filename, ios::out);

    //int global_x, global_k, global_z;
    //int light_x0, light_x1, light_y0, light_y1, light_k;

    // image
    auto aspect_ratio = 16.0 / 9.0;
    int image_height = static_cast<int>(image_width / aspect_ratio);

    // World
    collision_list world = preset_scene();
    coordinate look_from = coordinate(13, 2, 3);
    coordinate look_at = coordinate(0, 0, 0);
    auto v_fov = 40.0;

    // choose either blue or black
    //background = colour(0, 0, 0);
    colour background = colour(0.0823, 0.6745, 0.9294);

    // Eye
    const vec3 v_up(0, 1, 0);
    const auto distance_to_focus = 10.0;
    auto aperture = 0.0;

    eye camera(look_from, look_at, v_up, v_fov, aspect_ratio, aperture, distance_to_focus, 0.0, 1.0);

    // ppm file header
    output_file << "P3\n" << image_width << ' ' << image_height << "\n255\n";

    for (int j = image_height - 1; j >= 0; --j) {

        //std::cerr << "\rLines remaining: " << j << ' ' << std::flush;

        for (int i = 0; i < image_width; ++i) {

            colour pixel_colour(0, 0, 0);

            #pragma omp parallel for schedule(dynamic, 4)
            for (int s = 0; s < samples_per_pixel; ++s) {

                auto u = (i + random_double()) / (image_width - 1);
                auto v = (j + random_double()) / (image_height - 1);
                ray r = camera.get_ray(u, v);
                pixel_colour += ray_colour(r, background, world, max_depth);

            }

            write_colour(output_file, pixel_colour, samples_per_pixel);

        }

    }

    output_file.close();

}

void first_new_solution(int image_width, int samples_per_pixel, int max_depth, std::string filename) {

    // start by creating new file with filename
    std::ofstream output_file(filename);

    //int global_x, global_k, global_z;
    //int light_x0, light_x1, light_y0, light_y1, light_k;

    // image
    auto aspect_ratio = 16.0 / 9.0;
    int image_height = static_cast<int>(image_width / aspect_ratio);

    // World
    collision_list world = preset_scene();
    coordinate look_from = coordinate(13, 2, 3);
    coordinate look_at = coordinate(0, 0, 0);
    auto v_fov = 40.0;

    // choose either blue or black
    //background = colour(0, 0, 0);
    colour background = colour(0.0823, 0.6745, 0.9294);

    // Eye
    const vec3 v_up(0, 1, 0);
    const auto distance_to_focus = 10.0;
    auto aperture = 0.0;

    eye camera(look_from, look_at, v_up, v_fov, aspect_ratio, aperture, distance_to_focus, 0.0, 1.0);

    // ppm file header
    output_file << "P3\n" << image_width << ' ' << image_height << "\n255\n";

    for (int j = image_height - 1; j >= 0; --j) {

        //std::cerr << "\rLines remaining: " << j << ' ' << std::flush;

        for (int i = 0; i < image_width; ++i) {

            colour pixel_colour(0, 0, 0);

            #pragma omp parallel for ordered schedule(dynamic, 12) // new_solution_1
            for (int s = 0; s < samples_per_pixel; ++s) {

                auto u = (i + random_double()) / (image_width - 1);
                auto v = (j + random_double()) / (image_height - 1);
                ray r = camera.get_ray(u, v);
                pixel_colour += ray_colour(r, background, world, max_depth);

            }

            write_colour(output_file, pixel_colour, samples_per_pixel);

        }

    }

    output_file.close();

}

void second_new_solution(int image_width, int samples_per_pixel, int max_depth, std::string filename) {

    // start by creating new file with filename
    std::ofstream output_file(filename);

    //int global_x, global_k, global_z;
    //int light_x0, light_x1, light_y0, light_y1, light_k;

    // image
    auto aspect_ratio = 16.0 / 9.0;
    int image_height = static_cast<int>(image_width / aspect_ratio);

    // World
    collision_list world = preset_scene();
    coordinate look_from = coordinate(13, 2, 3);
    coordinate look_at = coordinate(0, 0, 0);
    auto v_fov = 40.0;

    // choose either blue or black
    //background = colour(0, 0, 0);
    colour background = colour(0.0823, 0.6745, 0.9294);

    // Eye
    const vec3 v_up(0, 1, 0);
    const auto distance_to_focus = 10.0;
    auto aperture = 0.0;

    eye camera(look_from, look_at, v_up, v_fov, aspect_ratio, aperture, distance_to_focus, 0.0, 1.0);

    // ppm file header
    output_file << "P3\n" << image_width << ' ' << image_height << "\n255\n";

    for (int j = image_height - 1; j >= 0; --j) {

        //std::cerr << "\rLines remaining: " << j << ' ' << std::flush;

        for (int i = 0; i < image_width; ++i) {

            colour pixel_colour(0, 0, 0);

            #pragma omp parallel for schedule(dynamic, 12) // new_solution_2
            for (int s = 0; s < samples_per_pixel; ++s) {

                auto u = (i + random_double()) / (image_width - 1);
                auto v = (j + random_double()) / (image_height - 1);
                ray r = camera.get_ray(u, v);
                pixel_colour += ray_colour(r, background, world, max_depth);

            }

            write_colour(output_file, pixel_colour, samples_per_pixel);

        }

    }

    output_file.close();

}

void third_new_solution(int image_width, int samples_per_pixel, int max_depth, std::string filename) {

    // start by creating new file with filename
    std::ofstream output_file(filename);

    //int global_x, global_k, global_z;
    //int light_x0, light_x1, light_y0, light_y1, light_k;

    // image
    auto aspect_ratio = 16.0 / 9.0;
    int image_height = static_cast<int>(image_width / aspect_ratio);

    // World
    collision_list world = preset_scene();
    coordinate look_from = coordinate(13, 2, 3);
    coordinate look_at = coordinate(0, 0, 0);
    auto v_fov = 40.0;

    // choose either blue or black
    //background = colour(0, 0, 0);
    colour background = colour(0.0823, 0.6745, 0.9294);

    // Eye
    const vec3 v_up(0, 1, 0);
    const auto distance_to_focus = 10.0;
    auto aperture = 0.0;

    eye camera(look_from, look_at, v_up, v_fov, aspect_ratio, aperture, distance_to_focus, 0.0, 1.0);

    // ppm file header
    output_file << "P3\n" << image_width << ' ' << image_height << "\n255\n";

    for (int j = image_height - 1; j >= 0; --j) {

       //std::cerr << "\rLines remaining: " << j << ' ' << std::flush;

        for (int i = 0; i < image_width; ++i) {

            colour pixel_colour(0, 0, 0);

            #pragma omp parallel for ordered schedule(guided) // new_solution_3
            for (int s = 0; s < samples_per_pixel; ++s) {

                auto u = (i + random_double()) / (image_width - 1);
                auto v = (j + random_double()) / (image_height - 1);
                ray r = camera.get_ray(u, v);
                pixel_colour += ray_colour(r, background, world, max_depth);

            }

            write_colour(output_file, pixel_colour, samples_per_pixel);

        }

    }

    output_file.close();


}

void fourth_new_solution(int image_width, int samples_per_pixel, int max_depth, std::string filename) {

    // start by creating new file with filename
    std::ofstream output_file(filename);

    //int global_x, global_k, global_z;
    //int light_x0, light_x1, light_y0, light_y1, light_k;

    // image
    auto aspect_ratio = 16.0 / 9.0;
    int image_height = static_cast<int>(image_width / aspect_ratio);

    // World
    collision_list world = preset_scene();
    coordinate look_from = coordinate(13, 2, 3);
    coordinate look_at = coordinate(0, 0, 0);
    auto v_fov = 40.0;

    // choose either blue or black
    //background = colour(0, 0, 0);
    colour background = colour(0.0823, 0.6745, 0.9294);

    // Eye
    const vec3 v_up(0, 1, 0);
    const auto distance_to_focus = 10.0;
    auto aperture = 0.0;

    eye camera(look_from, look_at, v_up, v_fov, aspect_ratio, aperture, distance_to_focus, 0.0, 1.0);

    // ppm file header
    output_file << "P3\n" << image_width << ' ' << image_height << "\n255\n";

    for (int j = image_height - 1; j >= 0; --j) {

        //std::cerr << "\rLines remaining: " << j << ' ' << std::flush;

        for (int i = 0; i < image_width; ++i) {

            colour pixel_colour(0, 0, 0);

            #pragma omp parallel for schedule(guided) // new_solution_4
            for (int s = 0; s < samples_per_pixel; ++s) {

                auto u = (i + random_double()) / (image_width - 1);
                auto v = (j + random_double()) / (image_height - 1);
                ray r = camera.get_ray(u, v);
                pixel_colour += ray_colour(r, background, world, max_depth);

            }

            write_colour(output_file, pixel_colour, samples_per_pixel);

        }

    }

    output_file.close();

}


void fifth_new_solution(int image_width, int samples_per_pixel, int max_depth, std::string filename) {

    // start by creating new file with filename
    std::ofstream output_file(filename);

    //int global_x, global_k, global_z;
    //int light_x0, light_x1, light_y0, light_y1, light_k;

    // image
    auto aspect_ratio = 16.0 / 9.0;
    int image_height = static_cast<int>(image_width / aspect_ratio);

    // World
    register collision_list world = preset_scene();
    coordinate look_from = coordinate(13, 2, 3);
    coordinate look_at = coordinate(0, 0, 0);
    auto v_fov = 40.0;

    // choose either blue or black
    //background = colour(0, 0, 0);
    register colour background = colour(0.0823, 0.6745, 0.9294);

    // Eye
    const vec3 v_up(0, 1, 0);
    const auto distance_to_focus = 10.0;
    auto aperture = 0.0;

    register eye camera(look_from, look_at, v_up, v_fov, aspect_ratio, aperture, distance_to_focus, 0.0, 1.0);

    // ppm file header
    output_file << "P3\n" << image_width << ' ' << image_height << "\n255\n";

    for (int j = image_height - 1; j >= 0; --j) {

        //std::cerr << "\rLines remaining: " << j << ' ' << std::flush;

        for (int i = 0; i < image_width; ++i) {

            //colour pixel_colour(0, 0, 0);
            double a = 0, b = 0, c = 0;
            auto rng = random_double();

            #pragma omp parallel for reduction(+: a, b, c) schedule(dynamic, 4) // new_solution_4
            for (int s = 0; s < samples_per_pixel; ++s) {

                auto u = (i + rng) / (image_width - 1);
                auto v = (j + rng) / (image_height - 1);
                ray r = camera.get_ray(u, v);

                colour temp_colour(0, 0, 0);
                temp_colour = ray_colour(r, background, world, max_depth);

                a += temp_colour.x();
                b += temp_colour.y();
                c += temp_colour.z();

            }

            colour pixel_colour(a, b, c);
            write_colour(output_file, pixel_colour, samples_per_pixel);

        }

    }

}


void opencl_solution(int image_width, int samples_per_pixel, int max_depth, std::string filename) {

    // start by creating new file with filename
    std::ofstream output_file(filename);

    //int global_x, global_k, global_z;
    //int light_x0, light_x1, light_y0, light_y1, light_k;

    // image
    auto aspect_ratio = 16.0 / 9.0;
    int image_height = static_cast<int>(image_width / aspect_ratio);

    // World
    collision_list world = preset_scene();
    coordinate look_from = coordinate(13, 2, 3);
    coordinate look_at = coordinate(0, 0, 0);
    auto v_fov = 40.0;

    // choose either blue or black
    //background = colour(0, 0, 0);
    colour background = colour(0.0823, 0.6745, 0.9294);

    // Eye
    const vec3 v_up(0, 1, 0);
    const auto distance_to_focus = 10.0;
    auto aperture = 0.0;

    eye camera(look_from, look_at, v_up, v_fov, aspect_ratio, aperture, distance_to_focus, 0.0, 1.0);

    // ppm file header
    std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

}

void opencl_omp_solution(int image_width, int samples_per_pixel, int max_depth, std::string filename) {

    // start by creating new file with filename
    std::ofstream output_file(filename);

    //int global_x, global_k, global_z;
    //int light_x0, light_x1, light_y0, light_y1, light_k;

    // image
    auto aspect_ratio = 16.0 / 9.0;
    int image_height = static_cast<int>(image_width / aspect_ratio);

    // World
    collision_list world = preset_scene();
    coordinate look_from = coordinate(13, 2, 3);
    coordinate look_at = coordinate(0, 0, 0);
    auto v_fov = 40.0;

    // choose either blue or black
    //background = colour(0, 0, 0);
    colour background = colour(0.0823, 0.6745, 0.9294);

    // Eye
    const vec3 v_up(0, 1, 0);
    const auto distance_to_focus = 10.0;
    auto aperture = 0.0;

    eye camera(look_from, look_at, v_up, v_fov, aspect_ratio, aperture, distance_to_focus, 0.0, 1.0);

    // ppm file header
    std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

}



int main(int argc, char** argv) {

    // record all of the filenames
    std::vector<std::vector<std::string>> file_names_1 (8, std::vector<std::string>(10, " "));
    std::vector<std::vector<std::string>> file_names_2(8, std::vector<std::string>(10, " "));
    std::vector<std::vector<std::string>> file_names_3(8, std::vector<std::string>(10, " "));
    std::vector<std::vector<std::string>> file_names_4(8, std::vector<std::string>(10, " "));
    std::vector<std::vector<std::string>> file_names_5(8, std::vector<std::string>(10, " "));
    std::vector<std::vector<std::string>> file_names_6(8, std::vector<std::string>(10, " "));
    std::vector<std::vector<std::string>> file_names_7(8, std::vector<std::string>(10, " "));
    std::vector<std::vector<std::string>> file_names_8(8, std::vector<std::string>(10, " "));

    // get a default runtime to fill the empty vector
    auto def_start = high_resolution_clock::now();
    auto def_end = high_resolution_clock::now();
    auto def_runtime = duration_cast<microseconds>(def_end - def_start);

    // record all of the runtimes
    //std::vector<std::vector<std::vector<std::chrono::microseconds>>> runtimes (8, std::vector<std::vector<std::chrono::microseconds>(8, def_runtime));
    std::vector<std::vector<std::chrono::microseconds>> base_ray_trace_runtimes(8, std::vector<std::chrono::microseconds>(10, def_runtime));
    std::vector<std::vector<std::chrono::microseconds>> original_omp_ray_trace_runtimes(8, std::vector<std::chrono::microseconds>(10, def_runtime));
    std::vector<std::vector<std::chrono::microseconds>> solution_1_ray_trace_runtimes(8, std::vector<std::chrono::microseconds>(10, def_runtime));
    std::vector<std::vector<std::chrono::microseconds>> solution_2_ray_trace_runtimes(8, std::vector<std::chrono::microseconds>(10, def_runtime));
    std::vector<std::vector<std::chrono::microseconds>> solution_3_ray_trace_runtimes(8, std::vector<std::chrono::microseconds>(10, def_runtime));
    std::vector<std::vector<std::chrono::microseconds>> solution_4_ray_trace_runtimes(8, std::vector<std::chrono::microseconds>(10, def_runtime));
    std::vector<std::vector<std::chrono::microseconds>> opencl_ray_trace_runtimes(8, std::vector<std::chrono::microseconds>(10, def_runtime));
    std::vector<std::vector<std::chrono::microseconds>> opencl_omp_ray_trace_runtimes(8, std::vector<std::chrono::microseconds>(10, def_runtime));

    // record the mean runtimes
    std::vector<std::vector<std::chrono::microseconds>> mean_runtimes(8, std::vector<std::chrono::microseconds>(8, def_runtime));

    // stores all of the different samples per pixels to be ran
    std::vector<int> spps;
    spps.push_back(128);
    spps.push_back(256);
    spps.push_back(512);
    spps.push_back(1024);
    spps.push_back(2048);
    spps.push_back(4096);
    spps.push_back(8192);
    spps.push_back(16384);


    // obtain image_width and max_depth values from cmd line inputs
    image_width = atoi(argv[1]);
    max_depth = atoi(argv[2]);
    auto aspect_ratio = 16.0 / 9.0;
    image_height = static_cast<int>(image_width / aspect_ratio);

    std::string render_type;
    
    
    render_type = "base_ray_trace";
    for (int x = 0; x < 8; x++) {

        for (int y = 0; y < 10; y++) {

            std::string filename = render_type + " " + std::to_string(image_width) + "x"
                + std::to_string(image_height) + " " + std::to_string(spps[x]) + " "
                + std::to_string(max_depth) + " " + std::to_string(y + 1) + ".ppm";

            file_names_1[x][y] = filename;

        }

    }
    
    /*
    render_type = "original_omp_ray_trace";
    for (int x = 0; x < 8; x++) {

        for (int y = 0; y < 10; y++) {

            std::string filename = render_type + " " + std::to_string(image_width) + "x"
                + std::to_string(image_height) + " " + std::to_string(spps[x]) + " "
                + std::to_string(max_depth) + " " + std::to_string(y + 1) + ".ppm";

            file_names_2[x][y] = filename;

        }

    }

    render_type = "solution1_ray_trace";
    for (int x = 0; x < 8; x++) {

        for (int y = 0; y < 10; y++) {

            std::string filename = render_type + " " + std::to_string(image_width) + "x"
                + std::to_string(image_height) + " " + std::to_string(spps[x]) + " "
                + std::to_string(max_depth) + " " + std::to_string(y + 1) + ".ppm";

            file_names_3[x][y] = filename;

        }

    }

    render_type = "solution_2_ray_trace";
    for (int x = 0; x < 8; x++) {

        for (int y = 0; y < 10; y++) {

            std::string filename = render_type + " " + std::to_string(image_width) + "x"
                + std::to_string(image_height) + " " + std::to_string(spps[x]) + " "
                + std::to_string(max_depth) + " " + std::to_string(y + 1) + ".ppm";

            file_names_4[x][y] = filename;

        }

    }

    render_type = "solution_3_ray_trace";
    for (int x = 0; x < 8; x++) {

        for (int y = 0; y < 10; y++) {

            std::string filename = render_type + " " + std::to_string(image_width) + "x"
                + std::to_string(image_height) + " " + std::to_string(spps[x]) + " "
                + std::to_string(max_depth) + " " + std::to_string(y + 1) + ".ppm";

            file_names_5[x][y] = filename;

        }

    }
    */
    render_type = "solution_4_ray_trace";
    for (int x = 0; x < 8; x++) {

        for (int y = 0; y < 10; y++) {

            std::string filename = render_type + " " + std::to_string(image_width) + "x"
                + std::to_string(image_height) + " " + std::to_string(spps[x]) + " "
                + std::to_string(max_depth) + " " + std::to_string(y + 1) + ".ppm";

            file_names_6[x][y] = filename;

        }

    }
    

    /*
    render_type = "opencl_ray_trace";
    for (int x = 0; x < 8; x++) {

        for (int y = 0; y < 10; y++) {

            std::string filename = render_type + " " + std::to_string(image_width) + "x"
                + std::to_string(image_height) + " " + std::to_string(spps[x]) + " "
                + std::to_string(max_depth) + " " + std::to_string(y + 1) + ".ppm";

            file_names_7[x][y] = filename;

        }

    }

    render_type = "opencl_omp_ray_trace";
    for (int x = 0; x < 8; x++) {

        for (int y = 0; y < 10; y++) {

            std::string filename = render_type + " " + std::to_string(image_width) + "x"
                + std::to_string(image_height) + " " + std::to_string(spps[x]) + " "
                + std::to_string(max_depth) + " " + std::to_string(y + 1) + ".ppm";

            file_names_8[x][y] = filename;

        }

    }
    */


    /*
    // Testing purposes - make sure all file names come out right
    for (int x = 0; x < 8; x++) {

        for (std::string filename : file_names_1[x]) {

            std::cout << filename << std::endl;

        }

    }
    */
    
    /*
    // ray tracer without parallelization
    for (int y = 0; y < 7; y++) {

        for (int z = 0; z < 10; z++) {

            // recording starting point
            auto start = high_resolution_clock::now();

            base_ray_trace(image_width, spps[y], max_depth, file_names_1[y][z]);
            auto stop = high_resolution_clock::now();

            auto runtime = duration_cast<microseconds>(stop - start);
            base_ray_trace_runtimes[y][z] = runtime;

            std::cerr << runtime.count() << " microseconds" << std::endl;

        }

    }
    */
    


    /*
    // first start with original OpenMP Implementation
    for (int y = 0; y < 8; y++) {

        for (int z = 0; z < 10; z++) {

            // recording starting point
            auto start = high_resolution_clock::now();

            old_omp_solution(image_width, spps[y], max_depth, file_names_2[y][z]);
            auto stop = high_resolution_clock::now();

            auto runtime = duration_cast<microseconds>(stop - start);
            original_omp_ray_trace_runtimes[y][z] = runtime;

            std::cerr << runtime.count() << " microseconds" << std::endl;

        }

        std::cerr << "\nDone with original OpenMP implementation part " << y << "\n";

    }
    */

    /*
    // first new OpenMP Implementation
    for (int y = 0; y < 8; y++) {

        for (int z = 0; z < 10; z++) {

            // recording starting point
            auto start = high_resolution_clock::now();
            first_new_solution(image_width, spps[y], max_depth, file_names_3[y][z]);
            auto stop = high_resolution_clock::now();

            auto runtime = duration_cast<microseconds>(stop - start);
            solution_2_ray_trace_runtimes[y][z] = runtime;

            std::cerr << runtime.count() << " microseconds" << std::endl;

        }

        std::cerr << "\nDone with first new OpenMP implementation part " << y << "\n";

    }
    */

    /*
    // second new OpenMP Implementation
    for (int y = 0; y < 8; y++) {

        for (int z = 0; z < 10; z++) {

            // recording starting point
            auto start = high_resolution_clock::now();
            second_new_solution(image_width, spps[y], max_depth, file_names_4[y][z]);
            auto stop = high_resolution_clock::now();

            auto runtime = duration_cast<microseconds>(stop - start);
            solution_3_ray_trace_runtimes[y][z] = runtime;

            std::cerr << runtime.count() << " microseconds" << std::endl;

        }

        std::cerr << "\nDone with second new OpenMP implementation part " << y << "\n";

    }
    */

    /*
    // third new OpenMP Implementation
    for (int y = 0; y < 8; y++) {

        for (int z = 0; z < 10; z++) {

            // recording starting point
            auto start = high_resolution_clock::now();
            third_new_solution(image_width, spps[y], max_depth, file_names_5[y][z]);
            auto stop = high_resolution_clock::now();

            auto runtime = duration_cast<microseconds>(stop - start);
            solution_4_ray_trace_runtimes[y][z] = runtime;

            std::cerr << runtime.count() << " microseconds" << std::endl;

        }

        std::cerr << "\nDone with third new OpenMP implementation part " << y << "\n";

    }
    */

    /*
    // fourth new OpenMP Implementation
    for (int y = 0; y < 7; y++) {

        for (int z = 0; z < 10; z++) {

            // recording starting point
            auto start = high_resolution_clock::now();
            fourth_new_solution(image_width, spps[y], max_depth, file_names_6[y][z]);
            auto stop = high_resolution_clock::now();

            auto runtime = duration_cast<microseconds>(stop - start);
            opencl_ray_trace_runtimes[y][z] = runtime;

            std::cerr << runtime.count() << " microseconds" << std::endl;

        }

        std::cerr << "\nDone with fourth new OpenMP implementation part " << y << "\n";

    }
    */
    
    
    // fifth new OpenMP Implementation
    for (int y = 0; y < 5; y++) {

        for (int z = 0; z < 10; z++) {

            // recording starting point
            auto start = high_resolution_clock::now();
            fifth_new_solution(image_width, spps[y], max_depth, file_names_6[y][z]);
            auto stop = high_resolution_clock::now();

            auto runtime = duration_cast<microseconds>(stop - start);
            //opencl_ray_trace_runtimes[y][z] = runtime;

            std::cerr << runtime.count() << " microseconds" << std::endl;

        }

        std::cerr << "\nDone with fifth new OpenMP implementation part " << y << "\n";

    }
    

    /*
    // calculate mean runtimes
    for (int x = 0; x < 8; x++) {

        std::chrono::microseconds mean_runtime;

        for (std::chrono::microseconds runtime : original_omp_ray_trace_runtimes[x]) {

            mean_runtime = mean_runtime + runtime;

        }

        mean_runtime = mean_runtime / 10;

        mean_runtimes[1][x] = mean_runtime;

    }

    for (int x = 0; x < 8; x++) {

        std::chrono::microseconds mean_runtime;

        for (std::chrono::microseconds runtime : solution_1_ray_trace_runtimes[x]) {

            mean_runtime = mean_runtime + runtime;

        }

        mean_runtime = mean_runtime / 10;

        mean_runtimes[2][x] = mean_runtime;

    }

    for (int x = 0; x < 8; x++) {

        std::chrono::microseconds mean_runtime;

        for (std::chrono::microseconds runtime : solution_2_ray_trace_runtimes[x]) {

            mean_runtime = mean_runtime + runtime;

        }

        mean_runtime = mean_runtime / 10;

        mean_runtimes[3][x] = mean_runtime;

    }

    for (int x = 0; x < 8; x++) {

        std::chrono::microseconds mean_runtime;

        for (std::chrono::microseconds runtime : solution_3_ray_trace_runtimes[x]) {

            mean_runtime = mean_runtime + runtime;

        }

        mean_runtime = mean_runtime / 10;

        mean_runtimes[4][x] = mean_runtime;

    }

    for (int x = 0; x < 8; x++) {

        std::chrono::microseconds mean_runtime;

        for (std::chrono::microseconds runtime : solution_4_ray_trace_runtimes[x]) {

            mean_runtime = mean_runtime + runtime;

        }

        mean_runtime = mean_runtime / 10;

        mean_runtimes[5][x] = mean_runtime;

    }
    */

    //std::string summary_filename = "data summary.csv";

    /*
    // Image
    //auto aspect_ratio = 16.0 / 9.0;
    //image_height = static_cast<int>(image_width / aspect_ratio);

    // presets (testing)
    //image_width = 1280;
    //samples_per_pixel = 16384;
    //max_depth = 16;

    // World
    //world = preset_scene();
    //coordinate look_from = coordinate(13, 2, 3);
    //coordinate look_at = coordinate(0, 0, 0);
    //auto v_fov = 40.0;

    // choose either blue or black
    //background = colour(0, 0, 0);
    //background = colour(0.0823, 0.6745, 0.9294);
    
    // Eye
    //const vec3 v_up(0, 1, 0);
    //const auto distance_to_focus = 10.0;
    //auto aperture = 0.0;

    //eye camera(look_from, look_at, v_up, v_fov, aspect_ratio, aperture, distance_to_focus, 0.0, 1.0);
    */

    /*
    // Render type
    enum render_type {
        base_ray_trace,
        original_omp_ray_trace,
        new_omp_ray_trace,
        new_opencl_ray_trace,
        new_omp_opencl_ray_trace
    };
    */

    /*
    render_type render = new_omp_ray_trace;

    // change the render type based on the input
    if (trace_type == 1) render = base_ray_trace;
    else if (trace_type == 2) render = original_omp_ray_trace;
    else if (trace_type == 3) render = new_omp_ray_trace;
    else if (trace_type == 4) render = new_opencl_ray_trace;
    else if (trace_type == 5) render = new_omp_opencl_ray_trace;
    else render = new_omp_ray_trace;
   */

    /*
    if (render == base_ray_trace) { // if base_ray_trace, performs ray trace without any parallelization

        
        // ppm file header
        std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

        for (int j = image_height - 1; j >= 0; --j) {

            std::cerr << "\rLines remaining: " << j << ' ' << std::flush;

            for (int i = 0; i < image_width; ++i) {

                colour pixel_colour(0, 0, 0);

                for (int s = 0; s < samples_per_pixel; ++s) {

                    auto u = (i + random_double()) / (image_width - 1);
                    auto v = (j + random_double()) / (image_height - 1);
                    ray r = camera.get_ray(u, v);
                    pixel_colour += ray_colour(r, background, world, max_depth);

                }

                write_colour(std::cout, pixel_colour, samples_per_pixel);

            }

        }
        

    }

    else if (render == original_omp_ray_trace) {    // if using original_omp_ray_trace, ray trace computed with original parallelization

        
        // ppm file header
        std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

        for (int j = image_height - 1; j >= 0; --j) {

            std::cerr << "\rLines remaining: " << j << ' ' << std::flush;

            for (int i = 0; i < image_width; ++i) {

                colour pixel_colour(0, 0, 0);

                #pragma omp parallel for schedule(dynamic, 4)
                for (int s = 0; s < samples_per_pixel; ++s) {

                    auto u = (i + random_double()) / (image_width - 1);
                    auto v = (j + random_double()) / (image_height - 1);
                    ray r = camera.get_ray(u, v);
                    pixel_colour += ray_colour(r, background, world, max_depth);

                }

                write_colour(std::cout, pixel_colour, samples_per_pixel);

            }

        }
        

    }

    else if (render == new_omp_ray_trace) {
        
        
        // ppm file header
        std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

        for (int j = image_height - 1; j >= 0; --j) {

            std::cerr << "\rLines remaining: " << j << ' ' << std::flush;

            for (int i = 0; i < image_width; ++i) {

                colour pixel_colour(0, 0, 0);

                //#pragma omp parallel for ordered schedule(dynamic, 12) // new_solution_1
                //#pragma omp parallel for schedule(dynamic, 12) // new_solution_2
                //#pragma omp parallel for ordered schedule(guided) // new_solution_3
                #pragma omp parallel for schedule(guided) // new_solution_4
                for (int s = 0; s < samples_per_pixel; ++s) {

                    auto u = (i + random_double()) / (image_width - 1);
                    auto v = (j + random_double()) / (image_height - 1);
                    ray r = camera.get_ray(u, v);
                    pixel_colour += ray_colour(r, background, world, max_depth);

                }

                write_colour(std::cout, pixel_colour, samples_per_pixel);

            }

        }
        

    }

    else if (render == new_opencl_ray_trace) {

        // ppm file header
        std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

        for (int j = image_height - 1; j >= 0; --j) {

            std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;

            for (int i = 0; i < image_width; ++i) {

                colour pixel_colour(0, 0, 0);

                for (int s = 0; s < samples_per_pixel; ++s) {

                    auto u = (i + random_double()) / (image_width - 1);
                    auto v = (j + random_double()) / (image_height - 1);
                    ray r = camera.get_ray(u, v);
                    pixel_colour += ray_colour(r, background, world, max_depth);

                }

                write_colour(std::cout, pixel_colour, samples_per_pixel);

            }

        }

    }

    else if (render == new_omp_opencl_ray_trace) {

        // ppm file header
        std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

        for (int j = image_height - 1; j >= 0; --j) {

            std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;

            for (int i = 0; i < image_width; ++i) {

                colour pixel_colour(0, 0, 0);

                for (int s = 0; s < samples_per_pixel; ++s) {

                    auto u = (i + random_double()) / (image_width - 1);
                    auto v = (j + random_double()) / (image_height - 1);
                    ray r = camera.get_ray(u, v);
                    pixel_colour += ray_colour(r, background, world, max_depth);

                }

                write_colour(std::cout, pixel_colour, samples_per_pixel);

            }

        }

    } 

    else { // Automatically does new omp ray trace



    }
    */

    //std::cerr << "\nDone.\n";

    // get the end time of the program
    //auto stop = high_resolution_clock::now();

    // calculate time taken
    //auto runtime = duration_cast<microseconds>(stop - start);

    // output time
    //std::cerr << runtime.count() << " microseconds " << std::endl;

}