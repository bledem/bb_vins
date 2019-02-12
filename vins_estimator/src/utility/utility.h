#pragma once

#include <cmath>
#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <cassert>
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include "geometry_msgs/Point.h"

#include <vector>
#include <opencv2/core/core.hpp>


class Utility
{
  public:
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        return dq;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
        ans << typename Derived::Scalar(0), -q(2), q(1),
            q(2), typename Derived::Scalar(0), -q(0),
            -q(1), q(0), typename Derived::Scalar(0);
        return ans;
    }

    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
    {
        //printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
        //Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
        //printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
        //return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
        return q;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
    {
        Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
        ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
        return ans;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
    {
        Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
        ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
        return ans;
    }

    static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / M_PI * 180.0;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t y = ypr(0) / 180.0 * M_PI;
        Scalar_t p = ypr(1) / 180.0 * M_PI;
        Scalar_t r = ypr(2) / 180.0 * M_PI;

        Eigen::Matrix<Scalar_t, 3, 3> Rz;
        Rz << cos(y), -sin(y), 0,
            sin(y), cos(y), 0,
            0, 0, 1;

        Eigen::Matrix<Scalar_t, 3, 3> Ry;
        Ry << cos(p), 0., sin(p),
            0., 1., 0.,
            -sin(p), 0., cos(p);

        Eigen::Matrix<Scalar_t, 3, 3> Rx;
        Rx << 1., 0., 0.,
            0., cos(r), -sin(r),
            0., sin(r), cos(r);

        return Rz * Ry * Rx;
    }

    static Eigen::Matrix3d g2R(const Eigen::Vector3d &g);

    template <size_t N>
    struct uint_
    {
    };

    template <size_t N, typename Lambda, typename IterT>
    void unroller(const Lambda &f, const IterT &iter, uint_<N>)
    {
        unroller(f, iter, uint_<N - 1>());
        f(iter + N);
    }

    template <typename Lambda, typename IterT>
    void unroller(const Lambda &f, const IterT &iter, uint_<0>)
    {
        f(iter);
    }

    template <typename T>
    static T normalizeAngle(const T& angle_degrees) {
      T two_pi(2.0 * 180);
      if (angle_degrees > 0)
      return angle_degrees -
          two_pi * std::floor((angle_degrees + T(180)) / two_pi);
      else
        return angle_degrees +
            two_pi * std::floor((-angle_degrees + T(180)) / two_pi);
    }

    template <typename T>
      using Vector3 = Eigen::Matrix<T, 3, 1>;

template <typename T>
  struct bbox {
  float xmin, ymin, xmax, ymax, prob;

  int num;
 std::string Class;
};

template <typename T>
  struct imgBboxes {

  std::vector<Utility::bbox<T>> list;
  float time;
};

template <typename T>
    struct ray {      
       Eigen::Vector3d  p_GR;
      T time;
      int state_id;
      int last_correlated_id;
      std::vector<size_t> tracked_feature_ids;
    };



  template <typename T>
    using Quaternion = Eigen::Quaternion<T>;


      template <typename T>
        using Matrix3 = Eigen::Matrix<T, 3, 3>;

      template <typename T>
        using Matrix4 = Eigen::Matrix<T, 4, 4>;

      template <typename T>
        using MatrixX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

      template <typename T>
        using RowVector3 = Eigen::Matrix<T, 1, 3>;

      template <typename T>
        using Vector2 = Eigen::Matrix<T, 2, 1>;



      template <typename T>
        using Vector4 = Eigen::Matrix<T, 4, 1>;

      template <typename T>
        using VectorX = Eigen::Matrix<T, Eigen::Dynamic, 1>;

      template <typename T>
        using Point = Vector3<T>;

      template <typename T>
        using GyroscopeReading = Vector3<T>;

      template <typename T>
        using AccelerometerReading = Vector3<T>;

      template <typename T>
        using Isometry3 = Eigen::Transform<T,3,Eigen::Isometry>;


    template <typename T>
       struct imuState {
         Eigen::Vector3d p_I_G;
         Eigen::Quaterniond q_IG;
       };

       template <typename T>
           struct bboxState {
                size_t bbox_id;
               size_t feature_id[4]= {0,0,0,0}; //track id
               double depth[4]={1.0,1.0,1.0,1.0};
		cv::Point deduced_pixel[4]= {cv::Point(0,0)};
		cv::Point undistort_pixel[4]= {cv::Point(0,0)};
		cv::Point2f final_bb[4]= {cv::Point(0,0)};

               ray<T> r_tl, r_br;
                Eigen::Vector3d  p_f_G_tl, p_f_G_br ;
		Eigen::Vector3d w_corner[4]; //to initialize to 0
		Eigen::Vector3d locked_corner[4];
		Eigen::Vector3d locked_corner_def[4];

	      std::array<float,3> avg;
                std::vector<Eigen::Vector3d> poses_vec;
 		std::pair<std::vector<Eigen::Vector3d>, Eigen::Vector3d> locked_bbox; //vec of the locked parallepiped
                std::vector<Eigen::Matrix3d> rotations_vec;
                std::vector<bbox<T>> pixel, un_pixel;
		
		std::pair<int,int> w_l;
             double time;
       //last YOLO detection values (not updated)
               bbox<T> prev_detection, cur_detection;
               int age, nb_detected; //for every new object
               float prev_time_detection;
             std::string class_;
               bool associated,locked_def;
		int lock_proba=0;
		cv::Mat last_img;
		std::string type_detection; // cnn, optical, repro, matching
           };


           struct locked_box {
                size_t bbox_id;
	        Eigen::Vector3d center;
 		std::vector<Eigen::Vector3d> corner_vec;
                Eigen::Vector2d bbox_wid_len_vec;
 };

           struct all_locked_box {
std::vector<size_t> bbox_id_vec;
std::vector<locked_box> locked_boxes;
int count=0;

};
	
    };


