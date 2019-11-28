/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: test_quad_tree.cc
*
*          Created On: Sat 26 Oct 2019 04:11:21 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#include <iostream>
#include <glog/logging.h>
#include <random>
#include <list>
#include <limits>

#include "QuadTree.h"

DEFINE_int32(r, 10, "search radius");

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_minloglevel = google::INFO;
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  std::random_device rd;
  std::mt19937 generator(rd());
  std::uniform_int_distribution<> dist(1, 200);
  Rectangle boundary(0, 0, 200, 200);
  QuadTree quad_tree(boundary);

  // Generate random 100 points and insert to quad tree
  for (int i = 0; i < 100; i++) {
    Eigen::Vector2d point(dist(generator), dist(generator));
    quad_tree.AddPoint(point);
  }

  std::list<QuadTree*> tree_list;
  tree_list.push_back(&quad_tree);
  int sum = 0;
  while (!tree_list.empty()) {
    QuadTree* node = tree_list.front();
    for (size_t i = 0; i < node->points().size(); i++) {
      sum++;
    }
    if (node->has_subtree()) {
      tree_list.push_back(node->n1_);
      tree_list.push_back(node->n2_);
      tree_list.push_back(node->n3_);
      tree_list.push_back(node->n4_);
    }
    tree_list.pop_front();
  }

  // random generate a seed point
  Eigen::Vector2d rand_point(dist(generator), dist(generator));
  LOG(INFO) << "rand_point: " << rand_point.transpose();

  // Get nearest point coordination
  Eigen::Vector2d nearest_point = Eigen::Vector2d::Zero();
  double distance = quad_tree.GetNearestPoint(rand_point, nearest_point);
  LOG(INFO) << "nearest point at (" << nearest_point[0] << ", " << nearest_point[1]
            << "), distance: " << distance;

  // Get all points near center with a radius FLAGS_r
  std::vector<Eigen::Vector2d> nearby_points;
  int number_points_nearby = quad_tree.GetPointsNearby(rand_point, FLAGS_r, nearby_points);
  LOG(INFO) << "numbers nearby: " << number_points_nearby;
  for (auto p : nearby_points) {
    double dx = p.x() - rand_point.x();
    double dy = p.y() - rand_point.y();
    LOG(INFO) << "distance: " << sqrt(dx * dx + dy * dy);
    LOG(INFO) << p.transpose();
  }
  return 0;
}
