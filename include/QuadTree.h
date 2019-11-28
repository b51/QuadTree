/*************************************************************************
 *
 *              Author: b51
 *                Mail: b51live@gmail.com
 *            FileName: QuadTree.h
 *
 *          Created On: Sat 26 Oct 2019 12:24:42 PM CST
 *     Licensed under The MIT License [see LICENSE for details]
 *
 ************************************************************************/

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <memory>
#include <vector>

struct Rectangle {
  Rectangle() : x(0), y(0), w(0), h(0) {}
  Rectangle(double _x, double _y, double _w, double _h)
      : x(_x), y(_y), w(_w), h(_h) {}
  double x, y, w, h;
};

class QuadTree {
 public:
  QuadTree();
  QuadTree(Rectangle boundary);
  QuadTree(Rectangle boundary, int n);

  // ~QuadTree();

  void AddPoint(const Eigen::Vector2d& point);

  void DivideQuadTree();

  std::vector<Eigen::Vector2d>& points() { return points_; }

  const std::vector<Eigen::Vector2d> points() const { return points_; }

  inline const int capacity() const { return capacity_; }

  inline const bool has_subtree() const { return has_subtree_; }

  double GetNearestPoint(const Eigen::Vector2d& point,
                         Eigen::Vector2d& nearest_point);

  bool IsIntersectWithRectangle(Rectangle rectangle);

  int GetPointsNearby(const Eigen::Vector2d& point, double r,
                      std::vector<Eigen::Vector2d>& points);

  Rectangle boundary_;
  bool has_subtree_;
  QuadTree *n1_, *n2_, *n3_, *n4_;


 private:
  void Init();

  std::vector<Eigen::Vector2d> points_;
  int capacity_;
};
