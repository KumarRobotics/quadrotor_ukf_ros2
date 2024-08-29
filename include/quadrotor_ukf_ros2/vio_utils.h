/*
* vio_utils.h
*
*  Created on: 04.30.2015
*      Author: Giuseppe Loianno
*/
#ifndef VIO_UTIL_H
#define VIO_UTIL_H
#define EIGEN_INITIALIZE_MATRICES_BY_NAN


#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
//using namespace std;

struct VIOUtil
{
    static Eigen::Matrix<double,4,4> expSE3(const Eigen::Matrix<double, 6, 1>& mu);
    static Eigen::Matrix<double,6,1> LogSE3(const Eigen::Matrix<double,4,4>& SE3);
    static Eigen::Matrix<double,3,3> expSO3(const Eigen::Matrix<double, 3, 1>& w);
    static Eigen::Matrix<double,3,1> LogSO3(const Eigen::Matrix<double,3,3>& SO3);
    static Eigen::Matrix<double,3,1> get_translation(const Eigen::Matrix<double, 4, 4>& SE3);
    static Eigen::Matrix<double,3,3> get_rotation(const Eigen::Matrix<double, 4, 4>& SE3);
    static void rodrigues_so3_exp(const Eigen::Matrix<double,3,1>& w, const double A, const double B, Eigen::Matrix<double,3,3>& R);
    static Eigen::Matrix<double,3,1> TriangulatePointLinearEigen(Eigen::Matrix<double,4,4> se3AfromB, const Eigen::Matrix<double,2,1> &v2A, Eigen::Matrix<double,2,1> &v2B);
    static Eigen::Matrix<double,3,1> TriangulatePointLinearLS(Eigen::Matrix<double,4,4> se3AfromB, const Eigen::Matrix<double,2,1> &v2A, Eigen::Matrix<double,2,1> &v2B);
    static double depthFromTriangulation(Eigen::Matrix<double,4,4> se3BtoA, Eigen::Matrix<double,2,1> &v2A, Eigen::Matrix<double,2,1> &v2B, Eigen::Matrix<double,3, 1> &point);
    static double depthFromTriangulationSecondFrame(Eigen::Matrix<double,4,4> se3BtoA, Eigen::Matrix<double,2,1> &v2A, Eigen::Matrix<double,2,1> &v2B, Eigen::Matrix<double,3, 1> &point);
    static void PointCovariance(Eigen::Matrix<double,4,4>& se3AtoB, const Eigen::Matrix<double,3,1> &p, double focal_length, Eigen::Matrix<double,3, 3> &cov);
    static Eigen::Matrix<double, 4,1> MatToQuat(const Eigen::Matrix<double, 3,3>& Rot);
    static Eigen::Matrix<double, 3,3> QuatToMat(const Eigen::Matrix<double, 4,1>& Quat);
    static Eigen::Matrix<double, 3,1> R_to_ypr(const Eigen::Matrix<double, 3,3>& R);
    static Eigen::Matrix<double, 3,3> ypr_to_R(const Eigen::Matrix<double, 3,1>& ypr);
    static Eigen::Matrix<double, 3,3> getSkew(const Eigen::Matrix<double, 3,1>& twist);
    static Eigen::Matrix<double, 6, 6> adjSE3(const Eigen::Matrix<double, 4, 4> T);
    static Eigen::Matrix<double, 4, 4> MeanofSigmaPointsManifold(const std::vector<Eigen::Matrix<double, 4, 4> >& T, const Eigen::Matrix<double, Eigen::Dynamic, 1>& w);
    static Eigen::Matrix<double, 3, 3> MeanofSigmaPointsManifoldSO3(const std::vector<Eigen::Matrix<double, 3, 3> >& R, const Eigen::Matrix<double, Eigen::Dynamic, 1>& w);


    // calculates the parallel transport of the covariance sigma along the line $exp_G(dx*t)$ for $t \in [0,1]$
    static Eigen::Matrix<double,6,6> parallel_transport(const Eigen::Matrix<double,6,1> &dx);
    static Eigen::Matrix<double,6,6> parallel_transport_trans(const Eigen::Matrix<double, 6, 1> &dx);
    static Eigen::Matrix<double,6,6> parallel_transport_helper(const Eigen::Matrix<double, 6, 1> &dx);


};

struct Conversions {
  static Eigen::Matrix<double,3,1> PointToVec(const geometry_msgs::Point &p);
  static geometry_msgs::Point VecToPoint(const Eigen::Matrix<double,3,1> &v);
  static Eigen::Matrix<double,3,1> Vector3ToVec(const geometry_msgs::Vector3 &p);
  static geometry_msgs::Vector3 VecToVector3(const Eigen::Matrix<double,3,1> &v);
  static Eigen::Quaternion<double> QuatToQuat(const geometry_msgs::Quaternion &q);
};

class SE3State{
private:
      Eigen::Matrix<double,4,4> T_;

public:
      SE3State(){T_.setIdentity();}
      Eigen::Matrix<double,3,3> get_R() {return VIOUtil::get_rotation(T_);}
      Eigen::Matrix<double,3,1> get_t() {return VIOUtil::get_translation(T_);}
      void set_Tfrom_R(const Eigen::Matrix<double,3,3>& R, const Eigen::Matrix<double,3,1>& t) {
                  T_.block(0,0,3,3) = R;
                  T_.block(0,3,3,1) = t;
            }
      void set_Tfrom_T(Eigen::Matrix<double,4,4> T) {
            T_ = T;
            }
      Eigen::Matrix<double,4,4> get_T(){return T_;}
};
#endif// VIO_UTIL_H
