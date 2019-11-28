/*************************************************************************
 *
 *              Author: b51
 *                Mail: b51live@gmail.com
 *            FileName: QuadTree.cc
 *
 *          Created On: Sat 26 Oct 2019 12:24:36 PM CST
 *     Licensed under The MIT License [see LICENSE for details]
 *
 ************************************************************************/

#include "QuadTree.h"
#include <glog/logging.h>

QuadTree::QuadTree() : capacity_(4) {
  Init();
}

QuadTree::QuadTree(Rectangle boundary) : boundary_(boundary), capacity_(4) {
  Init();
}

QuadTree::QuadTree(Rectangle boundary, int n)
    : boundary_(boundary), capacity_(n) {
  Init();
}

void QuadTree::Init() {
  has_subtree_ = false;
  points_.reserve(capacity_);
}

void QuadTree::AddPoint(const Eigen::Vector2d& point) {
  if (point.x() < boundary_.x or point.x() > boundary_.x + boundary_.w or
      point.y() < boundary_.y or point.y() > boundary_.y + boundary_.h)
    return;

  if ((int)points_.size() < capacity_) {
    points_.emplace_back(point);
    return;
  }

  if (!has_subtree_) {
    DivideQuadTree();
    has_subtree_ = true;
  }
  if (point.x() < n2_->boundary_.x) {
    if (point.y() < n3_->boundary_.y)
      n1_->AddPoint(point);
    else
      n3_->AddPoint(point);
  } else {
    if (point.y() < n4_->boundary_.y)
      n2_->AddPoint(point);
    else
      n4_->AddPoint(point);
  }
}

void QuadTree::DivideQuadTree() {
  double x = boundary_.x;
  double y = boundary_.y;
  double half_w = std::floor(boundary_.w / 2.0);
  double half_h = std::floor(boundary_.h / 2.0);

  Rectangle boundary_1(x, y, half_w, half_h);
  Rectangle boundary_2(x + half_w, y, half_w, half_h);
  Rectangle boundary_3(x, y + half_h, half_w, half_h);
  Rectangle boundary_4(x + half_w, y + half_h, half_w, half_h);

  n1_ = new QuadTree(boundary_1, capacity_);
  n2_ = new QuadTree(boundary_2, capacity_);
  n3_ = new QuadTree(boundary_3, capacity_);
  n4_ = new QuadTree(boundary_4, capacity_);
}

double QuadTree::GetNearestPoint(const Eigen::Vector2d& point,
                                 Eigen::Vector2d& nearest_point) {
  if (point.x() < boundary_.x or point.x() > boundary_.x + boundary_.w or
      point.y() < boundary_.y or point.y() > boundary_.y + boundary_.h) {
    nearest_point = Eigen::Vector2d(-1, -1);
    return -1.0;
  }

  double x = point.x();
  double y = point.y();
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < points_.size(); i++) {
    double dist_x = points_[i].x() - x;
    double dist_y = points_[i].y() - y;
    double dist = sqrt(dist_x * dist_x + dist_y * dist_y);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_point = points_[i];
    }
  }

  if (has_subtree_) {
    Eigen::Vector2d candidate_point_1 = Eigen::Vector2d::Zero();
    Eigen::Vector2d candidate_point_2 = Eigen::Vector2d::Zero();
    Eigen::Vector2d candidate_point_3 = Eigen::Vector2d::Zero();
    Eigen::Vector2d candidate_point_4 = Eigen::Vector2d::Zero();
    double dist_1 = n1_->GetNearestPoint(point, candidate_point_1);
    double dist_2 = n2_->GetNearestPoint(point, candidate_point_2);
    double dist_3 = n3_->GetNearestPoint(point, candidate_point_3);
    double dist_4 = n4_->GetNearestPoint(point, candidate_point_4);

    if (dist_1 > 0. and dist_1 < min_dist) {
      min_dist = dist_1;
      nearest_point = candidate_point_1;
    }
    if (dist_2 > 0. and dist_2 < min_dist) {
      min_dist = dist_2;
      nearest_point = candidate_point_2;
    }
    if (dist_3 > 0. and dist_3 < min_dist) {
      min_dist = dist_3;
      nearest_point = candidate_point_3;
    }
    if (dist_4 > 0. and dist_4 < min_dist) {
      min_dist = dist_4;
      nearest_point = candidate_point_4;
    }
  }
  return min_dist;
}

bool QuadTree::IsIntersectWithRectangle(Rectangle rectangle) {
  if (boundary_.x + boundary_.w < rectangle.x or
      boundary_.x > rectangle.x + rectangle.w or
      boundary_.y + boundary_.h < rectangle.y or
      boundary_.y > rectangle.y + rectangle.h) {
    return false;
  }
  return true;
}

int QuadTree::GetPointsNearby(const Eigen::Vector2d& point, double r,
                              std::vector<Eigen::Vector2d>& points) {
  double x = point.x() - r;
  double y = point.y() - r;
  double w = 2 * r;
  double h = 2 * r;
  Rectangle rectangle(x, y, w, h);
  int number_points_nearby = 0;

  if (!IsIntersectWithRectangle(rectangle))
    return number_points_nearby;

  for (const auto& p : points_) {
    if (p.x() >= x and p.x() < x + w and p.y() >= y and p.y() < y + h) {
      points.emplace_back(p);
      number_points_nearby++;
    }
  }
  if (has_subtree_) {
    number_points_nearby += n1_->GetPointsNearby(point, r, points);
    number_points_nearby += n2_->GetPointsNearby(point, r, points);
    number_points_nearby += n3_->GetPointsNearby(point, r, points);
    number_points_nearby += n4_->GetPointsNearby(point, r, points);
  }
  return number_points_nearby;
}
