#include <quadrotor_ukf_ros2/quadrotor_ukf.h>

#include <iostream>

QuadrotorUKF::QuadrotorUKF()
{
  // Init Dimensions
  stateCnt_         = 12;
  procNoiseCnt_     = 9;
  measNoiseSLAMCnt_ = 6;
  L_ = stateCnt_ + procNoiseCnt_;
  // Init State
  xHist_.clear();
  uHist_.clear();
  xTimeHist_.clear();
  Xa_.setZero(stateCnt_, 2*L_+1);
  Va_.setZero(procNoiseCnt_, 2*L_+1);
  // Initialize P^a_0
  P_.setZero(stateCnt_,stateCnt_);
  P_(0,0) = 0.5*0.5;
  P_(1,1) = 0.5*0.5;
  P_(2,2) = 0.1*0.1;
  P_(3,3) = 0.1*0.1;
  P_(4,4) = 0.1*0.1;
  P_(5,5) = 0.1*0.1;
  P_(6,6) = 10*M_PI/180*10*M_PI/180;
  P_(7,7) = 10*M_PI/180*10*M_PI/180;
  P_(8,8) = 10*M_PI/180*10*M_PI/180;
  P_(9,9)   =  0.01*0.01;
  P_(10,10) =  0.01*0.01;
  P_(11,11) =  0.01*0.01;
  Rv_.setIdentity(procNoiseCnt_,procNoiseCnt_);

  // Init Sigma Points
  alpha_ = 0.1;
  beta_  = 2;
  kappa_ = 0;
  GenerateWeights();
  // Other Inits
  g_ = 9.81;
  initMeasure_ = false;
  initGravity_ = false;
}

QuadrotorUKF::~QuadrotorUKF() { }

bool QuadrotorUKF::isInitialized() { return (initMeasure_); }

Eigen::Matrix<double, Eigen::Dynamic, 1> QuadrotorUKF::GetState() { return xHist_.front(); }

rclcpp::Time QuadrotorUKF::GetStateTime(){ return xTimeHist_.front(); }

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>       QuadrotorUKF::GetStateCovariance() { return P_; }

void QuadrotorUKF::SetGravity(double _g)
{
  g_ = _g;
  initGravity_ = true;
  std::cout << "Gravity Initialized " << g_ << std::endl;
}

void QuadrotorUKF::SetImuCovariance(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& _Rv) { Rv_ = _Rv; }

void QuadrotorUKF::SetUKFParameters(double _alpha, double _beta, double _kappa)
{
  alpha_ = _alpha;
  beta_  = _beta;
  kappa_ = _kappa;
  GenerateWeights();
}

void QuadrotorUKF::SetInitPose(Eigen::Matrix<double, Eigen::Dynamic, 1> p, rclcpp::Time time)
{
  Eigen::Matrix<double, Eigen::Dynamic, 1> x;
  x.setZero(stateCnt_, 1);
  //x.rows(0,2)  = p.rows(0,2);
  //x.rows(6,8)  = p.rows(3,5);
  x.block<3,1>(0,0) = p.block<3,1>(0,0);
  x.block<3,1>(6,0) = p.block<3,1>(3,0);


  xHist_.push_front(x);
  uHist_.push_front(Eigen::MatrixXd::Zero(6, 1));
  xTimeHist_.push_front(time);
  initMeasure_ = true;

}

bool QuadrotorUKF::ProcessUpdate(Eigen::Matrix<double, Eigen::Dynamic, 1> u, 
                                 rclcpp::Time time)
{
  if (!initMeasure_ || !initGravity_)
    return false;

  // Just update state, defer covariance update
  double dt = (time-xTimeHist_.front()).seconds();
  Eigen::Matrix<double, Eigen::Dynamic, 1> x = ProcessModel(xHist_.front(), u, Eigen::MatrixXd::Zero(procNoiseCnt_,1), dt);
  xHist_.push_front(x);
  uHist_.push_front(u);
  xTimeHist_.push_front(time);

  return true;
}

bool QuadrotorUKF::MeasurementUpdateSLAM(const Eigen::Matrix<double, Eigen::Dynamic, 1>& z, 
                                         const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& RnSLAM, 
                                         rclcpp::Time time)
{
  // Init
  if (!initMeasure_ || !initGravity_)
   return false;

  // A priori covariance
  std::list<Eigen::Matrix<double, Eigen::Dynamic, 1> >::iterator kx;
  std::list<Eigen::Matrix<double, Eigen::Dynamic, 1> >::iterator ku;
  std::list<rclcpp::Time>::iterator kt;

  PropagateAprioriCovariance(time, kx, ku, kt);

  Eigen::Matrix<double, Eigen::Dynamic, 1> x = *kx;
  // Get Measurement
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> H = MeasurementModelSLAM();
  Eigen::Matrix<double, Eigen::Dynamic, 1> za = H * x;

  // Compute Kalman Gain
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> S = H * P_ * H.transpose() + RnSLAM;

  // Clean way to get S_inv
  double det_S = S.determinant();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> S_inv;
  
  if (det_S != 0.0) {
    S_inv = S.inverse();
  } else {
    Eigen::HouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> qr(S);
    S_inv = qr.solve(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(S.rows(), S.cols()));
  }

  // The H matrix is only used to disregard differential rates (linear/angular velocities) 
  // H.transpose * S_inv is NOT supposed to be P_.inverse()
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K = P_ * H.transpose() * S_inv;

  bool s_inv_solution_exists = (S * S_inv).isApprox(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(S.rows(), S.cols())); 
  if (!s_inv_solution_exists) {
  }
  // Innovation
  Eigen::Matrix<double, Eigen::Dynamic, 1> inno = z - za;

  // Handle angle jumps
  inno(3,0) = asin(sin(inno(3,0)));
  // Posteriori Mean
  x += K * inno;
  *kx = x;
  // Posteriori Covariance
  P_ = P_ - K * H * P_;
  // Propagate Aposteriori State
  PropagateAposterioriState(kx, ku, kt);

  return true;
}

void QuadrotorUKF::GenerateWeights()
{
  // State + Process noise
  lambda_ = alpha_*alpha_*(L_+kappa_)-L_;
  //wm = zeros<rowvec>(2*L+1);
  //wc = zeros<rowvec>(2*L+1);
  wm_.setZero(1,2*L_+1);
  wc_.setZero(1,2*L_+1);
  wm_(0,0) = lambda_ / (L_+lambda_);
  wc_(0,0) = lambda_ / (L_+lambda_) + (1-alpha_*alpha_+beta_);
  for (int k = 1; k <= 2*L_; k++)
  {
    wm_(0,k) = 1 / (2 * (L_+lambda_));
    wc_(0,k) = 1 / (2 * (L_+lambda_));
  }
  gamma_ = sqrt(L_ + lambda_);
}

void QuadrotorUKF::GenerateSigmaPoints()
{
  // Expand state
  Eigen::Matrix<double, Eigen::Dynamic, 1> x  = xHist_.back();
  Eigen::Matrix<double, Eigen::Dynamic, 1> xaa;
  xaa.setZero(L_,1);
  xaa.block(0,0,stateCnt_,1) = x;
  //xaa.rows(0,stateCnt-1) = x;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Xaa;
  Xaa.setZero(L_, 2*L_+1);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Paa;
  Paa.setZero(L_,L_);
  //Paa.submat(0, 0, stateCnt_-1, stateCnt_-1) = P_;
  //Paa.submat(stateCnt_, stateCnt_, L_-1, L_-1) = Rv;
  Paa.block(0,0,stateCnt_,stateCnt_) = P_;
  Paa.block(stateCnt_,stateCnt_, L_ - stateCnt_, L_ - stateCnt_) = Rv_;
  // Matrix square root
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> sqrtPaa = Paa.llt().matrixL();
  // Mean
  //Xaa.col(0) = xaa;
  //Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>  xaaMat = repmat(xaa,1,L);
  //Xaa.cols(1,L) = xaaMat + gamma * sqrtPaa;
  //Xaa.cols(L+1,L+L) = xaaMat - gamma * sqrtPaa;

  // Create a matrix with columns the increased state 0
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> xaaMat;
  xaaMat.setZero(L_, L_);
  for (int i = 0; i < L_; i++)
    xaaMat.col(i) = xaa;

  Xaa.col(0) = xaa;
  Xaa.block(0,1,L_,L_) =   xaaMat.block(0,0,L_,L_) + gamma_ * sqrtPaa;
  Xaa.block(0,L_+1,L_,L_) = xaaMat.block(0,0,L_,L_) - gamma_ * sqrtPaa;
  //Xa = Xaa.rows(0, stateCnt-1);
  //Va = Xaa.rows(stateCnt, L-1);
  Xa_ = Xaa.block(0,0,stateCnt_, 2*L_+1);
  Va_ = Xaa.block(stateCnt_,0,L_-stateCnt_, 2*L_+1);
}

Eigen::Matrix<double, Eigen::Dynamic, 1> QuadrotorUKF::ProcessModel(const Eigen::Matrix<double, Eigen::Dynamic, 1>& x, 
                                                                    const Eigen::Matrix<double, 6, 1>& u, 
                                                                    const Eigen::Matrix<double, Eigen::Dynamic, 1>& v, 
                                                                    double dt)
{
  Eigen::Matrix<double, 3, 3> R;
  R = VIOUtil::ypr_to_R(x.block<3,1>(6,0));//x.rows(6,8)
  Eigen::Matrix<double, 3, 1> ag;
  ag(0,0) = 0;
  ag(1,0) = 0;
  ag(2,0) = g_;
  // Acceleration
  Eigen::Matrix<double, Eigen::Dynamic, 1> a = u.block<3,1>(0,0) + v.block<3,1>(0,0);//u.rows(0,2) + v.rows(0,2);
  Eigen::Matrix<double, Eigen::Dynamic, 1> ddx = R * (a - x.block<3,1>(9,0)) - ag;//R * (a - x.rows(9,11)) - ag;
 // Rotation
  Eigen::Matrix<double, 3, 1> w = u.block<3,1>(3,0) + v.block<3,1>(3,0);//u.rows(3,5) + v.rows(3,5);
  /*Eigen::Matrix<double, 3, 3> dR;//eye<mat>(3,3);
  dR.setIdentity();
  dR(0,1) = -w(2,0) * dt;
  dR(0,2) =  w(1,0) * dt;
  dR(1,0) =  w(2,0) * dt;
  dR(1,2) = -w(0,0) * dt;
  dR(2,0) = -w(1,0) * dt;
  dR(2,1) =  w(0,0) * dt;*/
  Eigen::Matrix<double, 3, 3> dR = VIOUtil::expSO3(w * dt);
  Eigen::Matrix<double, 3, 3> Rt = R * dR;
  // State
  Eigen::Matrix<double, Eigen::Dynamic, 1> xt = x;
  //xt.rows(0,2)  = x.rows(0,2) + x.rows(3,5)*dt + ddx*dt*dt/2;
  //xt.rows(3,5)  =               x.rows(3,5)    + ddx*dt     ;
  //xt.rows(6,8)  = R_to_ypr(Rt);
  //xt.rows(9,11) = x.rows(9,11)  + v.rows(6,8) *dt;
  xt.block<3,1>(0,0)  = x.block<3,1>(0,0) + x.block<3,1>(3,0)*dt + ddx*dt*dt/2;
  xt.block<3,1>(3,0)  = x.block<3,1>(3,0) + ddx*dt;
  xt.block<3,1>(6,0)  = VIOUtil::R_to_ypr(Rt);
  xt.block<3,1>(9,0)  = x.block<3,1>(9,0)  + v.block<3,1>(6,0) *dt;
  return xt;
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> QuadrotorUKF::MeasurementModelSLAM()
{
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> H;// = zeros<mat>(measNoiseSLAMCnt, stateCnt);
  // 6 rows x 12 columns
  H.setZero(measNoiseSLAMCnt_, stateCnt_);
  H(0,0) = 1;
  H(1,1) = 1;
  H(2,2) = 1;
  H(3,6) = 1;
  H(4,7) = 1;
  H(5,8) = 1;
  return H;
}

void QuadrotorUKF::PropagateAprioriCovariance(const rclcpp::Time time,
                                              std::list<Eigen::Matrix<double, Eigen::Dynamic, 1> >::iterator& kx, 
                                              std::list<Eigen::Matrix<double, Eigen::Dynamic, 1> >::iterator& ku, 
                                              std::list<rclcpp::Time>::iterator& kt)
{
  // Find aligned state, Time
  double mdt = NUM_INF;
  std::list<Eigen::Matrix<double, Eigen::Dynamic, 1> >::iterator k1 = xHist_.begin();
  std::list<Eigen::Matrix<double, Eigen::Dynamic, 1> >::iterator k2 = uHist_.begin();
  std::list<rclcpp::Time>::iterator k3 = xTimeHist_.begin();
  int k4 = 0;
  for (; k1 != xHist_.end(); k1++, k2++, k3++, k4++)
  {
    double dt = fabs((*k3 - time).seconds());
    if (dt < mdt)
    {
      mdt = dt;
      kx  = k1;
      ku  = k2;
      kt  = k3;
    }
    else
    {
      break;
    }
  }
  Eigen::Matrix<double, Eigen::Dynamic, 1> cx = *kx;
  rclcpp::Time ct = *kt;
  Eigen::Matrix<double, Eigen::Dynamic, 1> px = xHist_.back();
  rclcpp::Time pt = xTimeHist_.back();
  double dt = (ct - pt).seconds();
  if (fabs(dt) < 0.001)
  {
    kx = xHist_.begin();
    ku = uHist_.begin();
    kt = xTimeHist_.begin();
    return;
  }
  // Delete redundant states
  xHist_.erase(k1, xHist_.end());
  uHist_.erase(k2, uHist_.end());
  xTimeHist_.erase(k3, xTimeHist_.end());
  // rot, gravity
  Eigen::Matrix<double, 3, 3> pR = VIOUtil::ypr_to_R(px.block<3,1>(6,0));//px.rows(6,8)
  Eigen::Matrix<double, 3, 1> ag;
  ag(0,0) = 0;
  ag(1,0) = 0;
  ag(2,0) = g_;
  // Linear Acceleration
  Eigen::Matrix<double, 3, 1> dv = cx.block<3,1>(3,0) - px.block<3,1>(3,0);//rows(3,5);
  Eigen::Matrix<double, 3, 1> a = pR.transpose() * (dv / dt + ag) + px.block<3,1>(9,0);//.rows(9,11);
  // Angular Velocity
  Eigen::Matrix<double, 3, 3> dR = pR.transpose() * VIOUtil::ypr_to_R(cx.block<3,1>(6,0));//cx.rows(6,8)
  Eigen::Matrix<double, 3, 1> w = VIOUtil::LogSO3(dR)/dt;
  //w(0,0) = dR(2,1) / dt;
  //w(1,0) = dR(0,2) / dt;
  //w(2,0) = dR(1,0) / dt;
  // Assemble state and control
  Eigen::Matrix<double, 6, 1> u;
  u.block<3,1>(0,0) = a;
  u.block<3,1>(3,0) = w;
  // Generate sigma points
  GenerateSigmaPoints();
  // Mean
  for (int k = 0; k < 2*L_+1; k++)
  {
    Xa_.col(k) = ProcessModel(Xa_.col(k), u, Va_.col(k), dt);
  }

  // Handle jump between +M_PI and -M_PI !
  //Eigen::MatrixXd::Index maxRow, maxCol;
  double minYaw = Xa_.row(6).minCoeff();// = min(Xa.row(6), 1);
  double maxYaw = Xa_.row(6).maxCoeff();// = max(Xa.row(6), 1);
  if (fabs(minYaw - maxYaw) > M_PI)
  {
    for (int k = 0; k < 2*L_+1; k++)
    {
      if (Xa_(6,k) < 0)
        Xa_(6,k) += 2*M_PI;
    }
  }
  // Now we can get the mean...
  Eigen::Matrix<double, Eigen::Dynamic, 1> xa;
  xa.setZero(Xa_.rows(),1);
  for (int i = 0; i < 2 * L_ + 1; i++)
  {
    xa += wm_(0,i) * Xa_.col(i);// = sum( repmat(wm,stateCnt,1) % Xa, 1 );
  }

  // Covariance
  P_.setZero(Xa_.rows(), Xa_.rows());//.zeros();
  for (int k = 0; k < 2*L_+1; k++)
  {
    Eigen::Matrix<double, Eigen::Dynamic, 1> d = Xa_.col(k) - xa;
    P_ += wc_(0,k) * d * d.transpose();
  }

  return;
}

void QuadrotorUKF::PropagateAposterioriState(std::list<Eigen::Matrix<double, Eigen::Dynamic, 1> >::iterator kx, std::list<Eigen::Matrix<double, Eigen::Dynamic, 1> >::iterator ku, std::list<rclcpp::Time>::iterator kt)
{
  for (; kx != xHist_.begin(); kx--, ku--, kt--)
  {
    std::list<Eigen::Matrix<double, Eigen::Dynamic, 1> >::iterator _kx = kx;
    _kx--;
    std::list<Eigen::Matrix<double, Eigen::Dynamic, 1>>::iterator _ku = ku;
    _ku--;
    std::list<rclcpp::Time>::iterator _kt = kt;
    _kt--;
    *_kx = ProcessModel(*kx, *_ku, Eigen::MatrixXd::Zero(procNoiseCnt_,1), (*_kt - *kt).seconds());
  }
}


