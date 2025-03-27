#include "colmap/image/refractive_undistortion.h"

#include "colmap/estimators/two_view_geometry.h"
#include "colmap/scene/camera.h"
#include "colmap/scene/point2d.h"
#include "colmap/scene/reconstruction.h"
#include "colmap/sensor/bitmap.h"

#include <cmath>
#include <vector>

namespace colmap {

CameraQuadTreeNode::CameraQuadTreeNode(CameraQuadTree& top_left,
                                       CameraQuadTree& top_right,
                                       CameraQuadTree& bottom_left,
                                       CameraQuadTree& bottom_right)
    : top_left(top_left),
      top_right(top_right),
      bottom_left(bottom_left),
      bottom_right(bottom_right) {}

CameraQuadTreeLeaf::CameraQuadTreeLeaf(const Camera& camera) : camera(camera) {}

double Points3dVariance(
    std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> point_pairs) {
  std::vector<double> distances = std::vector<double>();
  double mean_distance = 0;
  for (std::tuple<Eigen::Vector2d, Eigen::Vector3d> point_pair : point_pairs) {
    Eigen::Vector3d point3d = std::get<1>(point_pair);
    double distance =
        sqrt(pow(point3d[0], 2) + pow(point3d[1], 2) + pow(point3d[2], 2));
    distances.push_back(distance);
    mean_distance += distance;
  }
  mean_distance /= point_pairs.size();
  double variance = 0;
  for (double distance : distances) {
    variance += pow(distance - mean_distance, 2);
  }
  variance /= point_pairs.size();
  return variance;
}

std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> FilterPointPairs(
    std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> point_pairs,
    struct ImageWindow window) {
  std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> filtered_pairs =
      std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>>();
  for (std::tuple<Eigen::Vector2d, Eigen::Vector3d> point_pair : point_pairs) {
    Eigen::Vector2d point2d = std::get<0>(point_pair);
    if (point2d[0] >= window.left && point2d[0] < window.right &&
        point2d[1] >= window.top && point2d[1] < window.bottom) {
      filtered_pairs.push_back(point_pair);
    }
  }
  return filtered_pairs;
}

CameraQuadTree BestFitNonRefracCameraQuadTree(
    CameraModelId tgt_model_id,
    const Camera& camera,
    std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> point_pairs,
    ImageWindow window) {
  if (point_pairs.size() < camera_estimation_threshold) {
    return CameraQuadTreeLeaf(camera);
  } else if (Points3dVariance(point_pairs) > max_distance_variance &&
             window.width() > 2 * min_window_size &&
             window.height() >= 2 * min_window_size) {
    size_t middle_x = (window.left + window.right) / 2;
    size_t middle_y = (window.top + window.bottom) / 2;

    struct ImageWindow top_left_window = {
        window.top, middle_y, window.left, middle_x};
    struct ImageWindow top_right_window = {
        window.top, middle_y, middle_x, window.right};
    struct ImageWindow bottom_left_window = {
        middle_y, window.bottom, window.left, middle_x};
    struct ImageWindow bottom_right_window = {
        middle_y, window.bottom, middle_x, window.right};
    CameraQuadTree top_left_tree = BestFitNonRefracCameraQuadTree(
        tgt_model_id,
        camera,
        FilterPointPairs(point_pairs, top_left_window),
        top_left_window);
    CameraQuadTree top_right_tree = BestFitNonRefracCameraQuadTree(
        tgt_model_id,
        camera,
        FilterPointPairs(point_pairs, top_right_window),
        top_right_window);
    CameraQuadTree bottom_left_tree = BestFitNonRefracCameraQuadTree(
        tgt_model_id,
        camera,
        FilterPointPairs(point_pairs, bottom_left_window),
        bottom_left_window);
    CameraQuadTree bottom_right_tree = BestFitNonRefracCameraQuadTree(
        tgt_model_id,
        camera,
        FilterPointPairs(point_pairs, bottom_right_window),
        bottom_right_window);
    return CameraQuadTreeNode(
        top_left_tree, top_right_tree, bottom_left_tree, bottom_right_tree);
  } else {
    return CameraQuadTreeLeaf(
        BestFitNonRefracCameraFromPoints(tgt_model_id, camera, point_pairs));
  }
}

CameraQuadTree BestFitNonRefracCameraQuadTree(
    CameraModelId tgt_model_id,
    const Camera& camera,
    const Reconstruction& reconstruction,
    image_t image_id) {
  CHECK(camera.IsCameraRefractive())
      << "Camera is not refractive, cannot compute the best approximated "
         "non-refractive camera";
  const Image& image = reconstruction.Image(image_id);
  Rigid3d cam_from_world = image.CamFromWorld();
  std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> point_pairs =
      std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>>();
  for (const struct Point2D& point2d : image.Points2D()) {
    if (point2d.HasPoint3D()) {
      point_pairs.push_back(
          {point2d.xy,
           cam_from_world * reconstruction.Point3D(point2d.point3D_id).xyz});
    }
  }

  struct ImageWindow window = {0, camera.height, 0, camera.width};
  return BestFitNonRefracCameraQuadTree(
      tgt_model_id, camera, point_pairs, window);
}

std::vector<std::tuple<double, Camera>> InterpolateCameras(
    CameraQuadTree camera_quad_tree,
    Eigen::Vector2d image_point,
    struct ImageWindow window) {
  if (std::holds_alternative<CameraQuadTreeLeaf>(camera_quad_tree)) {
    return {std::tuple<double, Camera>(
        1.0, std::get<CameraQuadTreeLeaf>(camera_quad_tree).GetCamera())};
  }

  const double left_center_x = 0.25 * window.width() + window.left;
  const double right_center_x = 0.75 * window.width() + window.left;
  const double top_center_y = 0.25 * window.height() + window.top;
  const double bottom_center_y = 0.75 * window.height() + window.top;

  const double left_factor =
      (right_center_x - image_point[0]) / (right_center_x - left_center_x);
  const double right_factor =
      (image_point[0] - left_center_x) / (right_center_x - left_center_x);
  const double top_factor =
      (bottom_center_y - image_point[1]) / (bottom_center_y - top_center_y);
  const double bottom_factor =
      (image_point[1] - top_center_y) / (bottom_center_y - top_center_y);

  std::vector<std::tuple<double, Camera>> cameras =
      std::vector<std::tuple<double, Camera>>();

  CameraQuadTreeNode camera_quad_tree_node =
      std::get<CameraQuadTreeNode>(camera_quad_tree);
  // Add top left cameras
  for (std::tuple<double, Camera> camera :
       InterpolateCameras(camera_quad_tree_node.TopLeft(),
                          image_point,
                          ImageWindow(window.top,
                                      (window.top + window.bottom) / 2,
                                      window.left,
                                      (window.left + window.right) / 2))) {
    cameras.push_back(std::tuple<double, Camera>(
        left_factor * top_factor * std::get<0>(camera), std::get<1>(camera)));
  }
  // Add top right cameras
  for (std::tuple<double, Camera> camera :
       InterpolateCameras(camera_quad_tree_node.TopRight(),
                          image_point,
                          ImageWindow(window.top,
                                      (window.top + window.bottom) / 2,
                                      (window.left + window.right) / 2,
                                      window.right))) {
    cameras.push_back(std::tuple<double, Camera>(
        right_factor * top_factor * std::get<0>(camera), std::get<1>(camera)));
  }
  // Add bottom left cameras
  for (std::tuple<double, Camera> camera :
       InterpolateCameras(camera_quad_tree_node.BottomLeft(),
                          image_point,
                          ImageWindow((window.top + window.bottom) / 2,
                                      window.bottom,
                                      window.left,
                                      (window.left + window.right) / 2))) {
    cameras.push_back(std::tuple<double, Camera>(
        left_factor * bottom_factor * std::get<0>(camera),
        std::get<1>(camera)));
  }
  // Add bottom right cameras
  for (std::tuple<double, Camera> camera :
       InterpolateCameras(camera_quad_tree_node.BottomRight(),
                          image_point,
                          ImageWindow((window.top + window.bottom) / 2,
                                      window.bottom,
                                      (window.left + window.right) / 2,
                                      window.right))) {
    cameras.push_back(std::tuple<double, Camera>(
        right_factor * bottom_factor * std::get<0>(camera),
        std::get<1>(camera)));
  }

  return cameras;
}

void WarpBetweenCameras(const CameraQuadTree& source_camera,
                        const Camera& target_camera,
                        const Bitmap& source_image,
                        Bitmap* target_image);
}  // namespace colmap
