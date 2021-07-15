/*
 * global_utils
 *  Created on: Sep 21, 2020
 *      Author: olegzilm
 */
#ifndef GLOBAL_UTILS_H_
#define GLOBAL_UTILS_H_

#define _USE_MATH_DEFINES

#include "eigen3/Eigen/Dense"
#include "constants.h"
#include "math/line2d.h"
#include "math/math_util.h"
#include <iostream>
#include <sstream>
#include <cmath>
#include "visualization/visualization.h"
#include "sensor_msgs/LaserScan.h"
#include <string>



namespace debug {
    template <typename T>
    void print_line(const geometry::Line<T>& line,std::string prefix = "") {
        std::cout << prefix << " (" << line.p0.x() << ","
             << line.p0.y() << ")-(" << line.p1.x()
             << "," << line.p1.y() << ")" << std::endl;
    }
    void inline print_loc(const Eigen::Vector2f& loc,std::string prefix = "", bool endline=true) {
            std::cout << prefix << " (" << loc.x() << "," << loc.y() << ")";
            if (endline)
              std::cout << std::endl;
    }
}

namespace visualization {

    inline void DrawCar(float w, float l, uint32_t color,
                        amrl_msgs::VisualizationMsg& msg) {
        Eigen::Vector2f corner1{(l + CarDims::wheelbase) / 2.f, w / 2.f};
        Eigen::Vector2f corner2{(l + CarDims::wheelbase) / 2.f, -w / 2.f};
        Eigen::Vector2f corner3{-(l - CarDims::wheelbase) / 2.f, -w / 2.f};
        Eigen::Vector2f corner4{-(l - CarDims::wheelbase) / 2.f, w / 2.f};
        visualization::DrawLine(corner1, corner2, color, msg);
        visualization::DrawLine(corner2, corner3, color, msg);
        visualization::DrawLine(corner3, corner4, color, msg);
        visualization::DrawLine(corner4, corner1, color, msg);

    }

    inline void DrawPointCloud(const std::vector<Eigen::Vector2f>& p_cloud, uint32_t color,
                                amrl_msgs::VisualizationMsg& msg ) {
        for (size_t i = 0; i < p_cloud.size(); ++i) {
            visualization::DrawPoint(p_cloud[i], color, msg);
        }
    }

}

namespace navigation {
    struct PoseSE2 {
            explicit PoseSE2(): loc{0,0}, angle{0} {};
            explicit PoseSE2(float x,float y,float angle_init):
                    loc{x,y}, angle{angle_init} {};
            explicit PoseSE2(Eigen::Vector2f loc, float angle_init):
                    loc{loc}, angle{angle_init} {};

            Eigen::Vector2f loc;
            float angle;

            const PoseSE2 operator+(const PoseSE2& rhs) {
                PoseSE2 result;
                result.loc = this->loc + rhs.loc;
                result.angle = this->angle + rhs.angle;
                return result;
            }

            PoseSE2& operator+=(const PoseSE2& rhs) {
                this->loc += rhs.loc;
                this->angle += rhs.angle;
                return *this;
            }

            void pprint(std::string prefix="Pose:",bool endline=true) const {

                std::cout << prefix << " (" << loc.x() << "," << loc.y() << ")"
                           << " a:" << angle;
                if (endline)
                    std::cout << std::endl;
            }
    };

    /*const PoseSE2 operator*(double ratio, const PoseSE2& rhs ) {
                return PoseSE2(ratio*(rhs.loc),ratio*(rhs.angle));
    }*/

    // Oleg TODO: consolidate with latencytracker class.
    struct OdomMeasurement {
        float vel;
        float c;
        Eigen::Vector2f loc;
        float angle;
        double timestamp;
    };

    struct ControlCommand {
        explicit ControlCommand(float vel_init,float c_init,double timestamp_init):
                       vel(vel_init),c(c_init),timestamp(timestamp_init) {};
        float vel;
        float c;
        double timestamp;
    };


}

using navigation::PoseSE2;

namespace collision {

    inline bool is_point_in_circle(Eigen::Vector2f circle_center, float radius, Eigen::Vector2f point) {
        if ((circle_center-point).norm() <= radius) {
            return true;
        }
        return false;
    }

    inline bool is_point_in_path(float r, float clearance, const Eigen::Vector2f& point) {
        Eigen::Vector2f turning_center(0,r);
        float collision_ring_internal_r = r-CarDims::w/2-clearance;
        float collision_ring_external_r = r+CarDims::w/2+clearance;
        if (!is_point_in_circle(turning_center,collision_ring_internal_r,point) &&
             is_point_in_circle(turning_center,collision_ring_external_r,point)) {
            return true;
        }
        return false;
    }

    inline float calc_distance_on_curve_from_angle_to_point(float angle, float c) {
        float arc = angle*2;
        if (fabs(c) < GenConsts::kEpsilon) {
            return std::numeric_limits<float>::infinity();
        }
        float r = 1/c;
        float dist = r*arc;
        return dist;

    }

    inline float calc_distance_on_curve_to_point(float c, const Eigen::Vector2f& point) {
        if (fabs(c) < GenConsts::kEpsilon) {
           return point.norm();
        }
        //float angle = atan2(point.y()/point.x());
        float angle = atan(point.y()/point.x());

        return calc_distance_on_curve_from_angle_to_point(angle, c);

    }

    inline float check_collision_curvature_zero(std::vector<Eigen::Vector2f> point_cloud) {
        float upper_y_bound = CarDims::w/2 + CarDims::default_safety_margin;
        float lower_y_bound = -CarDims::w/2 - CarDims::default_safety_margin;
        float fpl = GenConsts::FPL_max_bound;
        for (size_t i = 0; i < point_cloud.size(); ++i) {
            Eigen::Vector2f curr_p = point_cloud[i];
            if (curr_p.y() < upper_y_bound && curr_p.y() > lower_y_bound) {
                fpl = curr_p.x() - CarDims::wheelbase - CarDims::default_safety_margin;
            }
        }
        return fpl;
    }
}

namespace tf {
    inline PoseSE2 transform_pose_to_glob_frame(const PoseSE2& frame_delta,
                                         const PoseSE2& pose_loc_frame) {
        // frame delta is how ahead is the pose of loc frame base from the glob frame
        float new_frame_angle = frame_delta.angle + pose_loc_frame.angle;
        Eigen::Vector2f rotated_displacement =
                Eigen::Rotation2Df(frame_delta.angle) * pose_loc_frame.loc;
        Eigen::Vector2f new_frame_loc = rotated_displacement + frame_delta.loc;
        PoseSE2 new_frame_pose(new_frame_loc,new_frame_angle);
        return new_frame_pose;
    }

    inline PoseSE2 transform_pose_to_loc_frame(const PoseSE2& frame_delta,
                                             const PoseSE2& pose_glob_frame) {
            // frame delta is how ahead is the pose of loc frame base from the glob frame
            float new_frame_angle = pose_glob_frame.angle - frame_delta.angle;

            Eigen::Vector2f new_frame_loc =
                    Eigen::Rotation2Df( -frame_delta.angle) * (pose_glob_frame.loc-frame_delta.loc);
            PoseSE2 new_frame_pose(new_frame_loc,new_frame_angle);
            return new_frame_pose;
     }


    inline PoseSE2 transform_pose_to_loc_frame_x_axis(const PoseSE2& frame_delta,
                                        const PoseSE2& pose_glob_frame) {
        // frame delta is how ahead is the loc frame (of pose_curr_frame) from the glob frame
        float new_frame_angle = 0;
        Eigen::Vector2f rotated_displacement =
            Eigen::Rotation2Df(-frame_delta.angle) * pose_glob_frame.loc;
        Eigen::Vector2f new_frame_loc =
                rotated_displacement + Eigen::Vector2f(-frame_delta.loc.norm(),0);
        PoseSE2 new_frame_pose(new_frame_loc,new_frame_angle);
        return new_frame_pose;
    }


    inline Eigen::Vector2f transform_point_to_glob_frame(const PoseSE2& frame_delta,
                                           const Eigen::Vector2f& point_loc_frame) {
        // frame_delta is the cooridnates of the loc frame base in the global frame
        Eigen::Vector2f rotated_displacement = Eigen::Rotation2Df(frame_delta.angle) * point_loc_frame;
        Eigen::Vector2f new_frame_loc = rotated_displacement + frame_delta.loc;
        return new_frame_loc;
    }

    inline void transform_points_to_glob_frame(const PoseSE2& frame_delta,
                                               const std::vector<Eigen::Vector2f>& points_loc_frame,
                                               std::vector<Eigen::Vector2f>& new_frame_points) {
        size_t cloud_size = points_loc_frame.size();
        new_frame_points.resize(cloud_size);
        for (size_t i = 0; i < cloud_size; ++i) {
            new_frame_points[i] = transform_point_to_glob_frame(frame_delta, points_loc_frame[i]);
        }

    }

    inline Eigen::Vector2f transform_point_to_loc_frame(const PoseSE2& frame_delta,
                                              const Eigen::Vector2f& point_glob_frame) {
           // frame_delta is the cooridnates of the loc frame base in the global frame
           Eigen::Vector2f new_frame_loc = Eigen::Rotation2Df(-frame_delta.angle) * (point_glob_frame-frame_delta.loc);
           return new_frame_loc;
    }

    inline void transform_points_to_loc_frame(const PoseSE2& frame_delta,
                                                   const std::vector<Eigen::Vector2f>& points_glob_frame,
                                                   std::vector<Eigen::Vector2f>& new_frame_points) {
            size_t cloud_size = points_glob_frame.size();
            new_frame_points.resize(cloud_size);
            for (size_t i = 0; i < cloud_size; ++i) {
                new_frame_points[i] = transform_point_to_loc_frame(frame_delta, points_glob_frame[i]);
            }
    }

    inline Eigen::Vector2f transform_point_to_loc_frame_x_axis(const PoseSE2& frame_delta,
                                          const Eigen::Vector2f& point_glob_frame) {
           // frame_delta is the cooridnates of the loc frame base in the global frame
        Eigen::Vector2f rotated_displacement =
            Eigen::Rotation2Df(-frame_delta.angle) * point_glob_frame;
        Eigen::Vector2f new_frame_loc =
            rotated_displacement + Eigen::Vector2f(-frame_delta.loc.norm(),0);
        return new_frame_loc;
    }

    inline void proj_lidar_2_pts(const sensor_msgs::LaserScan& msg,
                                 std::vector<Eigen::Vector2f>& point_cloud,
                                 const Eigen::Vector2f& kLaserLoc,
                                 unsigned int n, bool filter_max_range=false,
                                 float max_range=-1) {

      unsigned int num_ranges;
      float angle_max_true;
      if (max_range < 0)
          max_range = msg.range_max-PhysicsConsts::radar_noise_std;

      if (n != 1) {
          unsigned int downsample_truncated_pts = ((msg.ranges.size()-1)%n);
          //cout << "downsample_truncated_pts:" << downsample_truncated_pts;
          num_ranges = std::floor((msg.ranges.size()-downsample_truncated_pts)/n)+1;
          angle_max_true = msg.angle_min + (msg.ranges.size()-downsample_truncated_pts-1)*msg.angle_increment;
      } else {
          num_ranges = msg.ranges.size();
          angle_max_true = msg.angle_min + (msg.ranges.size()-1)*msg.angle_increment;
      }

      static std::vector<float> downsampled_ranges;
      downsampled_ranges.resize(num_ranges);
      //cout << "LIDAR scan size:" << ranges.size() << endl;
      unsigned int downsample_cnt = 0;
      for (size_t i=0; i < msg.ranges.size(); ++i) {
            if (i%n == 0) {
                downsampled_ranges[downsample_cnt] = msg.ranges[i];
                downsample_cnt++;
            }
      }

      point_cloud.clear();
      float angle_incr = (angle_max_true - msg.angle_min)/num_ranges;
      float curr_laser_angle = msg.angle_min;
      for (size_t i = 0; i < num_ranges; ++i) {
        float curr_range = downsampled_ranges[i];
        if (curr_range >= msg.range_min && curr_range <= msg.range_max) {
          if (!filter_max_range ||
              curr_range <= (max_range)) {
            float x = cos(curr_laser_angle)*curr_range;
            float y = sin(curr_laser_angle)*curr_range;
            Eigen::Vector2f baselink_loc(Eigen::Vector2f(x,y) + kLaserLoc);
            point_cloud.push_back(baselink_loc);
          }
        }
        curr_laser_angle += angle_incr;
      }
    }
}

namespace geometry {
    template <typename T>
    bool is_point_in_circle(const Eigen::Matrix<T,2,1>& center,
                            float radius,
                            const Eigen::Matrix<T,2,1>& point) {
        return (center-point).norm() < radius + GenConsts::kEpsilon;
    }

    inline float normalize_angle(float angle) {
        double decimal;
        float frac = modf(angle/(2*M_PI), &decimal);
        if (angle >= 0)
          return frac*2*M_PI;
        else
          return 2*M_PI*(1+frac);
    }

    template <typename T>
    Line<T> get_sub_segment_in_circle(const Line<T>& segment,
                                      const Eigen::Matrix<T,2,1>& center,
                                      float radius) {

       if (is_point_in_circle(center, radius, segment.p0) &&
                is_point_in_circle(center, radius, segment.p1)) {
           return segment;
       }

       // Calculate intersection points between circle and segment's line.
       Eigen::Matrix<T,2,1> intersect_point_1;
       Eigen::Matrix<T,2,1> intersect_point_2;
       unsigned int num_intersections;
       if (fabs(segment.p1.x()-segment.p0.x()) < GenConsts::kEpsilon) {
           /*
            Solution to equation set
                  r^2 = (x-c_x)^2 + (y-c_y)^2
                  x = b
            is solving the SqEq:
                  y^2 + -2c_y*y+(b-c_x)^2+c_y^2-R^2) = 0
           */
           float y_intersect_1, y_intersect_2;
           float b = segment.p1.x();
           num_intersections = math_util::SolveQuadratic(
                          1.0f,
                          -2*center.y(),
                          (math_util::Sq(b-center.x()) +
                           math_util::Sq(center.y()) -
                           math_util::Sq(radius)),
                          &y_intersect_1, &y_intersect_2);

           if (num_intersections == 2) {
              intersect_point_1 << b,y_intersect_1;
              intersect_point_2 << b,y_intersect_2;
          }
      } else {
           float x_intersect_1, x_intersect_2;
           float a = (segment.p1.y()-segment.p0.y())/(segment.p1.x()-segment.p0.x());
           float b = segment.p1.y() - a*segment.p1.x();
           /*
             Solution to equation set
                   r^2 = (x-c_x)^2 + (y-c_y)^2
                   y = ax+b
             is solving the SqEq:
                   (1+a^2)x^2 + (2ab-2c_x-2c_y*a)x+(c_x^2 + c_y^2 + b^2 - r^2 -2c_y*b = 0
            */
           num_intersections = math_util::SolveQuadratic(
               1+math_util::Sq(a),
               2*(a*b - center.x() - a*center.y()),
               math_util::Sq(center.x())+ math_util::Sq(center.y()) +
               math_util::Sq(b)- math_util::Sq(radius)- 2*b*center.y(),
               &x_intersect_1, &x_intersect_2);

           if (num_intersections == 2) {
               intersect_point_1 << x_intersect_1,a*x_intersect_1+b;
               intersect_point_2 << x_intersect_2,a*x_intersect_2+b;
           }
      }

       Line<T> part_in_circle;
       if (num_intersections < 2) {
           part_in_circle.Set(Eigen::Matrix<T,2,1>(0,0),Eigen::Matrix<T,2,1>(0,0));
       } else {
           if (!is_point_in_circle(center, radius, segment.p0) &&
               !is_point_in_circle(center, radius, segment.p1)) {
               if (IsBetween(segment.p0,segment.p1,
                             intersect_point_1,
                             GenConsts::kEpsilon)
                   && IsBetween(segment.p0,segment.p1,
                                intersect_point_2,
                                GenConsts::kEpsilon)) {
                   part_in_circle.Set(intersect_point_1, intersect_point_2);
               } else {
                   part_in_circle.Set(Eigen::Matrix<T,2,1>(0,0),Eigen::Matrix<T,2,1>(0,0));
               }
           } else {
               Eigen::Matrix<T,2,1> inside_point;
               if (is_point_in_circle(center, radius, segment.p0)) {
                   inside_point = segment.p0;
               } else {
                   inside_point = segment.p1;
               }

               if (IsBetween(segment.p0,segment.p1,
                             intersect_point_1,
                             GenConsts::kEpsilon)) {
                   part_in_circle.Set(inside_point,intersect_point_1);
               } else {
                   part_in_circle.Set(inside_point,intersect_point_2);
               }
           }
       }

       return part_in_circle;
    }

    template <typename T>
    unsigned int line_circle_intersect(const Line<T>& segment,
                                       const Eigen::Matrix<T,2,1>& center,
                                       float radius,
                                       Eigen::Matrix<T,2,1>& intersect_point_1,
                                       Eigen::Matrix<T,2,1>& intersect_point_2) {

       if (is_point_in_circle(center, radius, segment.p0) &&
                is_point_in_circle(center, radius, segment.p1)) {
           return 0;
       }

       // Calculate intersection points between circle and segment's line.

       unsigned int num_intersections;
       if (fabs(segment.p1.x()-segment.p0.x()) < GenConsts::kEpsilon) {
          /*
            Solution to equation set
                  r^2 = (x-c_x)^2 + (y-c_y)^2
                  x = b
            is solving the SqEq:
                  y^2 + -2c_y*y+(b-c_x)^2+c_y^2-R^2) = 0
          */
          float y_intersect_1, y_intersect_2;
          float b = segment.p1.x();
          num_intersections = math_util::SolveQuadratic(
                          1.0f,
                          -2*center.y(),
                          (math_util::Sq(b-center.x()) +
                           math_util::Sq(center.y()) -
                           math_util::Sq(radius)),
                          &y_intersect_1, &y_intersect_2);

          if (num_intersections == 2) {
              intersect_point_1 << b,y_intersect_1;
              intersect_point_2 << b,y_intersect_2;
          } else if (num_intersections == 1) {
              intersect_point_1 << b,y_intersect_1;
          }
      } else {
           float x_intersect_1, x_intersect_2;
           float a = (segment.p1.y()-segment.p0.y())/(segment.p1.x()-segment.p0.x());
           float b = segment.p1.y() - a*segment.p1.x();
           /*
             Solution to equation set
                   r^2 = (x-c_x)^2 + (y-c_y)^2
                   y = ax+b
             is solving the SqEq:
                   (1+a^2)x^2 + (2ab-2c_x-2c_y*a)x+(c_x^2 + c_y^2 + b^2 - r^2 -2c_y*b = 0
            */
           num_intersections = math_util::SolveQuadratic(
               1+math_util::Sq(a),
               2*(a*b - center.x() - a*center.y()),
               math_util::Sq(center.x())+ math_util::Sq(center.y()) +
               math_util::Sq(b)- math_util::Sq(radius)- 2*b*center.y(),
               &x_intersect_1, &x_intersect_2);

           if (num_intersections == 2) {
               intersect_point_1 << x_intersect_1, a*x_intersect_1+b;
               intersect_point_2 << x_intersect_2, a*x_intersect_2+b;
           } else if (num_intersections == 1) {
               intersect_point_1 << x_intersect_1, a*x_intersect_1+b;
           }
      }

       if (num_intersections == 0) {
           return 0;
       } else if (num_intersections == 1) {
           return 1;
       } else {
           if (!is_point_in_circle(center, radius, segment.p0) &&
               !is_point_in_circle(center, radius, segment.p1)) {
               if (IsBetween(segment.p0,segment.p1,
                             intersect_point_1,
                             GenConsts::kEpsilon)
                   && IsBetween(segment.p0,segment.p1,
                                intersect_point_2,
                                GenConsts::kEpsilon)) {
                   return 2;
               } else {
                   return 0;
               }
           } else {
               if (IsBetween(segment.p0,segment.p1,
                             intersect_point_1,
                             GenConsts::kEpsilon)) {
                   return 1;
               } else {
                   intersect_point_1 = intersect_point_2;
                   return 1;
               }
           }
       }
    }

    inline float dist_arc_point(Eigen::Vector2f center, float r, Eigen::Vector2f point,
                                float start_angle, float end_angle, bool& is_end_point) {
        if (start_angle >= end_angle) {
            cout << "ERROR: geometry::dist_arc_point:: end angle has to be bigger then start angle";
            throw "ERROR: geometry::dist_arc_point:: end angle has to be bigger then start angle";
        }

        // angle of vector between point and center
        float angle = std::atan2(point.y()-center.y(),point.x()-center.x());

        //if start_angle is positive, we need to reverse angle if it is negative, so that the condition bellow will work
        if (start_angle >= 0 ) {
            start_angle = normalize_angle(start_angle);
            if ((end_angle-2*M_PI) < 0.0000001)
                end_angle = normalize_angle(end_angle);
            if (angle < 0)
                 angle = 2*M_PI+angle;
        }

        //cout << "dist_arc_point::angle:" << angle << " start_angle:"
        //      << start_angle << " end_angle" << end_angle << endl;

        if (angle >= start_angle && angle <= end_angle) {
            // Line of point-2-center crosses the arc, then dist is just the dist to the cross point
            //cout << "dist_arc_point:: in arc" << endl;
            is_end_point = false;
            return fabs((point-center).norm()-r);

        } else {
            // Line of point2center not on the arc, then min dist is the dist to one of the start/end point of the arc
            // Find point on cirle at start/end angle
            Eigen::Vector2f start_p(center.x() + std::cos(start_angle)*r,
                                    center.y() + std::sin(start_angle)*r);
            float dist2start = (point-start_p).norm();
            Eigen::Vector2f end_p(center.x() + std::cos(end_angle)*r,
                                  center.y() + std::sin(end_angle)*r);
            float dist2end = (point-end_p).norm();
            is_end_point = true;
            return std::min(dist2end,dist2start);
        }
    }

    inline float dist_line_point(Eigen::Vector2f l0, Eigen::Vector2f l1,
                                 Eigen::Vector2f point, bool& is_end_point) {
        Eigen::Vector2f proj_point = ProjectPointOntoLineSegment(point,l0,l1);
        if (proj_point == l0 || proj_point == l1) {
            float dist2start = (point-l0).norm();
            float dist2end = (point-l1).norm();
            is_end_point = true;
            return std::min(dist2end,dist2start);
        } else {
            Eigen::Vector2f vec_to_point = (point-l0);
            float angle_between_vecs = std::acos(
                    vec_to_point.dot((l1-l0))/(vec_to_point.norm()*(l1-l0).norm())
            );

            is_end_point = false;
            return fabs(std::sin(angle_between_vecs)*vec_to_point.norm());

        }
    }

}

#endif // GLOBAL_UTILS_H_
