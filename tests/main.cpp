#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "tyco.hpp"

namespace cs
{
struct world {};
struct camera {};
};

template<typename Cs>
using point3 = tyco::P3::column_vector<double, Cs>;

template<typename Cs>
using plane3 = tyco::P3::row_vector<double, Cs>;

template<typename CsLeft, typename CsRight>
using homography3 = tyco::P3::homography<double, CsLeft, CsRight>;

int main()
{
    using namespace std;
    cout << "TEST" << endl;

    auto point_world  = point3<cs::world>{{1, 1, 1, 1}};
    auto point_camera = point3<cs::camera>{{1, 1, 1, 0}};

    auto plane_world  = plane3<cs::world>();
    auto plane_camera = plane3<cs::camera>();

    auto camera_from_world  = homography3<cs::camera, cs::world>();
    auto world_from_camera  = homography3<cs::world,  cs::camera>();
    auto world_from_world   = homography3<cs::world,  cs::world>();
    auto camera_from_camera = homography3<cs::camera, cs::camera>();

    world_from_camera = inverse(camera_from_world);
    camera_from_world = inverse(world_from_camera);

    point_camera = camera_from_world * point_world;
    point_world = world_from_camera * point_camera;

    plane_camera = plane_world * world_from_camera;
    plane_world = plane_camera * camera_from_world;

    world_from_world = world_from_camera * camera_from_world;
    camera_from_camera = camera_from_world * world_from_camera;
}
