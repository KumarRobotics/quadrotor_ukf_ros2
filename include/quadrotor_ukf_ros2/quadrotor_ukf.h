#ifndef QUADROTOR_UKF_H
#define QUADROTOR_UKF_H

#include <iostream>
#include <string.h>
#include <math.h>
#include <list>
#include <vector>
#include <algorithm>

#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "rclcpp/time.hpp"
#include <quadrotor_ukf_ros2/vio_utils.h>

//#define PI 3.14159265359
#define NUM_INF 999999.9

class QuadrotorUKF
{
  private:

    // State History and Covariance
    std::list<Eigen::Matrix<double, Eigen::Dynamic, 1> >  xHist_;
    std::list<Eigen::Matrix<double, Eigen::Dynamic, 1> >  uHist_;
    std::list<rclcpp::Time> xTimeHist_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P_;

    // Process Covariance Matrix
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Rv_;

    // Instance sigma points
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Xa_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Va_;

    // Initial process update indicator
    bool initMeasure_;
    bool initGravity_;

    // Dimemsions
    int stateCnt_;
    int procNoiseCnt_;
    int measNoiseSLAMCnt_;
    int L_;

    // Gravity
    double g_;

    // UKF Parameters
    double alpha_;
    double beta_;
    double kappa_;
    double lambda_;
    double gamma_;

    // UKF Weights
    Eigen::Matrix<double, 1, Eigen::Dynamic>  wm_;
    Eigen::Matrix<double, 1, Eigen::Dynamic>  wc_;

    // Private functions
    void GenerateWeights();
    void GenerateSigmaPoints();
    Eigen::Matrix<double, Eigen::Dynamic, 1> ProcessModel(const Eigen::Matrix<double, Eigen::Dynamic, 1>& x, const Eigen::Matrix<double, 6, 1>& u, const Eigen::Matrix<double, Eigen::Dynamic, 1>& v, double dt);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MeasurementModelSLAM();
    void PropagateAprioriCovariance(const rclcpp::Time time, std::list<Eigen::Matrix<double, Eigen::Dynamic, 1> >::iterator& kx, std::list<Eigen::Matrix<double, Eigen::Dynamic, 1> >::iterator& ku, std::list<rclcpp::Time>::iterator& kt);
    void PropagateAposterioriState(std::list<Eigen::Matrix<double, Eigen::Dynamic, 1> >::iterator kx, std::list<Eigen::Matrix<double, Eigen::Dynamic, 1> >::iterator ku, std::list<rclcpp::Time>::iterator kt);

  public:

    QuadrotorUKF();
    ~QuadrotorUKF();

    bool isInitialized();
    Eigen::Matrix<double, Eigen::Dynamic, 1> GetState();
    rclcpp::Time GetStateTime();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> GetStateCovariance();

    void SetGravity(double _g);
    void SetImuCovariance(const Eigen::Matrix<double, Eigen::Dynamic,  Eigen::Dynamic>& _Rv);
    void SetUKFParameters(double _alpha, double _beta, double _kappa);
    void SetInitPose(Eigen::Matrix<double, Eigen::Dynamic, 1> p, rclcpp::Time time);

    bool ProcessUpdate(Eigen::Matrix<double, Eigen::Dynamic, 1> u, rclcpp::Time time);
    bool MeasurementUpdateSLAM(const Eigen::Matrix<double, Eigen::Dynamic, 1>&  z, const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>&  RnSLAM, rclcpp::Time time);
};

#endif
