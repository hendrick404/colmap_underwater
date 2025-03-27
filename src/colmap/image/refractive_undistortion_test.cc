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
  if (vec1.size() != vec2.size()) {
    return false;
  }
  for (std::tuple<double, Camera> camera_tuple1 : vec1) {
    bool found = false;
    for (size_t i = 0; i < vec2.size(); i++) {
      std::tuple<double, Camera> camera_tuple2 = vec2[i];
      if (std::get<1>(camera_tuple1) == std::get<1>(camera_tuple2) &&
          ApproximatelyEqual(std::get<0>(camera_tuple1),
                             std::get<0>(camera_tuple2))) {
        vec2.erase(vec2.begin() + i);
        found = true;
        break;
      }
    }
    if (!found) {
      return false;
    }
  }
  return vec2.empty();
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

}  // namespace
}  // namespace colmap