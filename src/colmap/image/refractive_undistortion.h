#pragma once

#include "colmap/scene/camera.h"
#include "colmap/scene/point2d.h"
#include "colmap/scene/reconstruction.h"
#include "colmap/sensor/bitmap.h"

#include <variant>
#include <vector>

namespace colmap {

const int camera_estimation_threshold = 10;
const double max_distance_variance = 0.1;
const int min_window_size = 20;

class CameraQuadTreeNode;

class CameraQuadTreeLeaf {
 public:
  CameraQuadTreeLeaf(const Camera& camera);

  const Camera& GetCamera() { return camera; }

 private:
  const Camera& camera;
};

typedef std::variant<CameraQuadTreeNode, CameraQuadTreeLeaf> CameraQuadTree;

class CameraQuadTreeNode {
 public:
  CameraQuadTreeNode(CameraQuadTree& top_left,
                     CameraQuadTree& top_right,
                     CameraQuadTree& bottom_left,
                     CameraQuadTree& bottom_right);

  CameraQuadTree& TopLeft() { return top_left; }
  CameraQuadTree& TopRight() { return top_right; }
  CameraQuadTree& BottomLeft() { return bottom_left; }
  CameraQuadTree& BottomRight() { return bottom_right; }

 private:
  CameraQuadTree& top_left;
  CameraQuadTree& top_right;
  CameraQuadTree& bottom_left;
  CameraQuadTree& bottom_right;
};

struct ImageWindow {
  size_t top;
  size_t bottom;
  size_t left;
  size_t right;

  ImageWindow(size_t top, size_t bottom, size_t left, size_t right)
      : top(top), bottom(bottom), left(left), right(right) {}

  size_t width() { return right - left; }

  size_t height() { return bottom - top; }
};

CameraQuadTree BestFitNonRefracCameraQuadTree(
    CameraModelId tgt_model_id,
    const Camera& camera,
    const Reconstruction& reconstruction,
    image_t image_id);

std::vector<std::tuple<double, Camera>> InterpolateCameras(
    CameraQuadTree camera_quad_tree,
    Point2D image_point,
    int width,
    int height);

void WarpBetweenCameras(const CameraQuadTree& source_camera,
                        const Camera& target_camera,
                        const Bitmap& source_image,
                        Bitmap* target_image);
}  // namespace colmap