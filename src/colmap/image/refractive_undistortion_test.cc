#include "colmap/image/refractive_undistortion.h"

#include "colmap/scene/camera.h"

#include <tuple>
#include <variant>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

namespace colmap {
namespace {

bool ApproximatelyEqual(double x, double y) {
  const double tolerance = 0.01;
  return abs(x - y) < tolerance;
}
// bool operator==(const Camera& lhs, const Camera& rhs);
bool operator==(const Camera& lhs, const Camera& rhs) {
  return lhs.camera_id == rhs.camera_id;
}

bool CameraVectorEquality(std::vector<std::tuple<double, Camera>> vec1,
                          std::vector<std::tuple<double, Camera>> vec2) {
  std::vector<std::tuple<double, Camera>> vec1_filtered =
      std::vector<std::tuple<double, Camera>>();
  for (std::tuple<double, Camera> e : vec1) {
    if (!ApproximatelyEqual(0, std::get<0>(e))) {
      vec1_filtered.push_back(e);
    }
  }
  std::vector<std::tuple<double, Camera>> vec2_filtered =
      std::vector<std::tuple<double, Camera>>();
  for (std::tuple<double, Camera> e : vec2) {
    if (!ApproximatelyEqual(0, std::get<0>(e))) {
      vec2_filtered.push_back(e);
    }
  }
  if (vec1_filtered.size() != vec2_filtered.size()) {
    return false;
  }
  for (std::tuple<double, Camera> camera_tuple1 : vec1_filtered) {
    bool found = false;
    for (size_t i = 0; i < vec2_filtered.size(); i++) {
      std::tuple<double, Camera> camera_tuple2 = vec2_filtered[i];
      if (std::get<1>(camera_tuple1) == std::get<1>(camera_tuple2) &&
          ApproximatelyEqual(std::get<0>(camera_tuple1),
                             std::get<0>(camera_tuple2))) {
        vec2_filtered.erase(vec2_filtered.begin() + i);
        found = true;
        break;
      }
    }
    if (!found) {
      return false;
    }
  }
  return vec2_filtered.empty();
}

TEST(InterpolateCameraQuadTree, SingleCamera) {
  const size_t width = 100;
  const size_t height = 100;
  Camera camera = Camera::CreateFromModelId(
      0, CameraModelId::kSimplePinhole, 100, 100, 100);
  CameraQuadTree tree = CameraQuadTreeLeaf(camera);
  for (Eigen::Vector2d point2d : {Eigen::Vector2d({0, 0})}) {
    ASSERT_TRUE(CameraVectorEquality(
        {std::tuple<double, Camera>({1, camera})},
        InterpolateCameras(tree, point2d, ImageWindow(0, height, 0, width))));
  }
}

TEST(InterpolateCameraQuadTree, FourCameras) {
  const size_t width = 100;
  const size_t height = 100;
  const Camera camera1 = Camera::CreateFromModelId(
      1, CameraModelId::kSimplePinhole, 100, 100, 100);
  const Camera camera2 = Camera::CreateFromModelId(
      1, CameraModelId::kSimplePinhole, 100, 100, 100);
  const Camera camera3 = Camera::CreateFromModelId(
      1, CameraModelId::kSimplePinhole, 100, 100, 100);
  const Camera camera4 = Camera::CreateFromModelId(
      1, CameraModelId::kSimplePinhole, 100, 100, 100);
  const CameraQuadTree branch1 = CameraQuadTreeLeaf(camera1);
  const CameraQuadTree branch2 = CameraQuadTreeLeaf(camera2);
  const CameraQuadTree branch3 = CameraQuadTreeLeaf(camera3);
  const CameraQuadTree branch4 = CameraQuadTreeLeaf(camera4);
  CameraQuadTree tree = CameraQuadTreeNode(branch1, branch2, branch3, branch4);
  for (std::tuple<Eigen::Vector2d, std::vector<std::tuple<double, Camera>>>
           test_data :
       {// The point in the top left center should be equivalent to just camera
        // 1
        std::tuple<Eigen::Vector2d, std::vector<std::tuple<double, Camera>>>(
            {Eigen::Vector2d(width * 0.25, height * 0.25), {{1, camera1}}}),
        // The point in the top right center should be equivalent to just camera
        // 2
        std::tuple<Eigen::Vector2d, std::vector<std::tuple<double, Camera>>>(
            {Eigen::Vector2d(width * 0.75, height * 0.25), {{1, camera2}}}),
        // The point in the bottom left center should be equivalent to just
        // camera 3
        std::tuple<Eigen::Vector2d, std::vector<std::tuple<double, Camera>>>(
            {Eigen::Vector2d(width * 0.25, height * 0.75), {{1, camera3}}}),
        // The point in the top right center should be equivalent to just camera
        // 4
        std::tuple<Eigen::Vector2d, std::vector<std::tuple<double, Camera>>>(
            {Eigen::Vector2d(width * 0.75, height * 0.75), {{1, camera4}}}),
        // The point in the center should be equivalent to just all 4 cameras
        // equally
        std::tuple<Eigen::Vector2d, std::vector<std::tuple<double, Camera>>>(
            {Eigen::Vector2d(width * 0.5, height * 0.5),
             {{0.25, camera1},
              {0.25, camera2},
              {0.25, camera3},
              {0.25, camera4}}})}) {
    Eigen::Vector2d image_point = std::get<0>(test_data);
    std::vector<std::tuple<double, Camera>> expected = std::get<1>(test_data);
    ASSERT_TRUE(CameraVectorEquality(
        InterpolateCameras(tree, image_point, ImageWindow(0, height, 0, width)),
        expected));
  }
}

}  // namespace
}  // namespace colmap