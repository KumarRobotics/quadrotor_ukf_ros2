/*created by
 *Giuseppe Loianno*/

#include <quadrotor_ukf/vio_utils.h>

//Get a translation from SE3
Eigen::Matrix<double,3,1> VIOUtil::get_translation(const Eigen::Matrix<double, 4, 4>& SE3)
{
  Eigen::Matrix<double,3,1>  result;
  result(0,0) = SE3(0,3);
  result(1,0) = SE3(1,3);
  result(2,0) = SE3(2,3);
  return result;
}

//Get a rotation from SE3
Eigen::Matrix<double,3,3> VIOUtil::get_rotation(const Eigen::Matrix<double, 4, 4>& SE3)
{
  Eigen::Matrix<double,3,3>  result = SE3.block(0,0,3,3);
  return result;
}

//Rodrigues formula from a vector to an homogeneous matrix
Eigen::Matrix<double,4,4> VIOUtil::expSE3(const Eigen::Matrix<double, 6, 1>& mu)
{
  static const double one_6th = 1.0/6.0;
  static const double one_20th = 1.0/20.0;

  //the Inital SE3 should be initialized to the identity
  Eigen::Matrix<double, 4, 4> result;
  result.setIdentity();

  const Eigen::Matrix<double, 3, 1> w = mu.block(3,0,3,1);//mu.tail<3>();
  const double theta_sq = w.transpose()*w;
  const double theta = sqrt(theta_sq);
  double A, B;

  Eigen::Matrix<double, 3, 1> mu_top = mu.block(0,0,3,1);
  const Eigen::Matrix<double,3,1> temp = w.cross(mu_top);//mu.head<3>()
  if (theta_sq < 1e-8)
  {
    A = 1.0 - one_6th * theta_sq;
    B = 0.5;
     //result.get_translation() = mu.block<0,0>(3,0) + 0.5 * temp;
    result.block(0,3,3,1) = mu_top + 0.5 * temp;
   }
   else
   {
    double C;
    if (theta_sq < 1e-6)
    {
      C = one_6th*(1.0 - one_20th * theta_sq);
      A = 1.0 - theta_sq * C;
      B = 0.5 - 0.25 * one_6th * theta_sq;
    }
    else
    {
      const double inv_theta = 1.0/theta;
      A = sin(theta) * inv_theta;
      B = (1 - cos(theta)) * (inv_theta * inv_theta);
      C = (1 - A) * (inv_theta * inv_theta);
    }
    //result.get_translation() = mu.slice<0,3>() + B*temp + C*(w.cross(temp));
    result.block(0,3,3,1) = mu_top + B*temp + C*(w.cross(temp));
  }

  Eigen::Matrix<double,3,3> Rot;
  VIOUtil::rodrigues_so3_exp(w, A, B, Rot);
  result.block(0,0,3,3) = Rot;
  return result;
}

//Exponential of SO3 to get the rotation matrix
Eigen::Matrix<double,3,3> VIOUtil::expSO3(const Eigen::Matrix<double, 3, 1>& w)
{
  static const double one_6th = 1.0/6.0;
  static const double one_20th = 1.0/20.0;

  //the Inital SE3 should be initialized to the identity
  Eigen::Matrix<double,3,3> result;
  result.setIdentity();

  const double theta_sq = w.transpose()*w;
  const double theta = sqrt(theta_sq);
  double A, B;
  //Use a Taylor series expansion near zero. This is required for
  //accuracy, since sin t / t and (1-cos t)/t^2 are both 0/0.
  if (theta_sq < 1e-8)
  {
    A = 1.0 - one_6th * theta_sq;
    B = 0.5;
  }
  else
  {
    if (theta_sq < 1e-6)
    {
      B = 0.5 - 0.25 * one_6th * theta_sq;
      A = 1.0 - theta_sq * one_6th*(1.0 - one_20th * theta_sq);
    }
    else
    {
      const double inv_theta = 1.0/theta;
      A = sin(theta) * inv_theta;
      B = (1 - cos(theta)) * (inv_theta * inv_theta);
    }
  }
  rodrigues_so3_exp(w, A, B, result);
  return result;
}

//Additional SO3
void VIOUtil::rodrigues_so3_exp(const Eigen::Matrix<double,3,1>& w, const double A, const double B, Eigen::Matrix<double,3,3>& R)
{
  const double wx2 = w(0,0)*w(0,0);
  const double wy2 = w(1,0)*w(1,0);
  const double wz2 = w(2,0)*w(2,0);

  R(0,0) = 1.0 - B*(wy2 + wz2);
  R(1,1) = 1.0 - B*(wx2 + wz2);
  R(2,2) = 1.0 - B*(wx2 + wy2);

  double a = A*w(2,0);
  double b = B*(w(0,0)*w(1,0));

  R(0,1) = b - a;
  R(1,0) = b + a;

  a = A*w(1,0);
  b = B*(w(0,0)*w(2,0));

  R(0,2) = b + a;
  R(2,0) = b - a;

  a = A*w(0,0);
  b = B*(w(1,0)*w(2,0));

  R(1,2) = b - a;
  R(2,1) = b + a;
}

Eigen::Matrix<double, 4, 4> VIOUtil::MeanofSigmaPointsManifold(const std::vector<Eigen::Matrix<double, 4, 4> >& T, const Eigen::Matrix<double, Eigen::Dynamic, 1>& w)
{
  Eigen::Matrix<double, 4, 4> mu = T.at(0);

  //sum of the errors
  Eigen::Matrix<double, 6, 1> sum_res;
  do
  {
    sum_res.setZero();
    for (unsigned int i = 0; i < T.size(); i++)
    {
      sum_res+=w(i,0) * VIOUtil::LogSE3(mu.inverse() * T.at(i));

    }
    //now compute the final mean
    mu = mu*VIOUtil::expSE3(sum_res);
  }
  while(sum_res.norm() > 0.001);

  return mu;
}

Eigen::Matrix<double, 3, 3> VIOUtil::MeanofSigmaPointsManifoldSO3(const std::vector<Eigen::Matrix<double, 3, 3> >& R, const Eigen::Matrix<double, Eigen::Dynamic, 1>& w)
{
  Eigen::Matrix<double, 3, 3> mu = R.at(0);

  //sum of the errors
  Eigen::Matrix<double, 3, 1> sum_res;
  do
  {
    sum_res.setZero();
    for (unsigned int i = 0; i < R.size(); i++)
    {
      sum_res+=w(i,0) * VIOUtil::LogSO3(mu.transpose() * R.at(i));

    }
    //now compute the final mean
    mu = mu*VIOUtil::expSO3(sum_res);

  }
  while(sum_res.norm() > 0.1);

  return mu;
}

//Logarithm formula from an homogeneous to a vector
Eigen::Matrix<double, 6, 1> VIOUtil::LogSE3(const Eigen::Matrix<double,4,4>& SE3)
{
  Eigen::Matrix<double,3,1> rot = LogSO3(get_rotation(SE3));
  const double theta = sqrt(rot.transpose()*rot);

  double shtot = 0.5;
   if(theta > 0.00001)
   {
       shtot = sin(theta/2)/theta;
   }

   // now do the rotation
  const Eigen::Matrix<double,3,3> halfrotator = VIOUtil::expSO3(rot * -0.5);
  Eigen::Matrix<double,3,1> rottrans = halfrotator * VIOUtil::get_translation(SE3);

  if(theta > 0.001)
  {
      rottrans -=(rot*((VIOUtil::get_translation(SE3).transpose() * rot) * (1.0-2.0*shtot) / (rot.transpose()*rot)));
  }
  else
  {
      rottrans -=(rot*((VIOUtil::get_translation(SE3).transpose() * rot)/24.0));
  }

  rottrans /= (2 * shtot);

  Eigen::Matrix<double,6,1> result;
  result.setZero();
  result.block(0,0,3,1) = rottrans;
  result.block(3,0,3,1) = rot;

  return result;
}

//Logarithm formula from an homogeneous matrix to a vector for SO3
Eigen::Matrix<double,3,1> VIOUtil::LogSO3(const Eigen::Matrix<double,3,3>& SO3)
{
  Eigen::Matrix<double, 3, 1> result;
  result.setZero();
  const double cos_angle = (SO3(0,0) + SO3(1,1) + SO3(2,2) - 1.0) * 0.5;
  result(0,0) = (SO3(2,1)-SO3(1,2))/2;
  result(1,0) = (SO3(0,2)-SO3(2,0))/2;
  result(2,0) = (SO3(1,0)-SO3(0,1))/2;

  double sin_angle_abs = sqrt(result.transpose()*result);

  if (cos_angle > M_SQRT1_2) // [0 - Pi/4[ use asin
  {
    if(sin_angle_abs > 0)
    {
      result *= asin(sin_angle_abs) / sin_angle_abs;
    }
  }
  else if( cos_angle > -M_SQRT1_2) // [Pi/4 - 3Pi/4[ use acos, but antisymmetric part
  {
    const double angle = acos(cos_angle);
    result *= angle / sin_angle_abs;
  }
  else
  {
    // rest use symmetric part
    // antisymmetric part vanishes, but still large rotation, need information from symmetric part
    const double angle = M_PI - asin(sin_angle_abs);
    const double d0 = SO3(0,0) - cos_angle,
        d1 = SO3(1,1) - cos_angle,
        d2 = SO3(2,2) - cos_angle;
        Eigen::Matrix<double,3,1> r2;

    if(d0*d0 > d1*d1 && d0*d0 > d2*d2) // first is largest, fill with first column
    {
      r2(0,0) = d0;
      r2(1,0) = (SO3(1,0) + SO3(0,1))/2;
      r2(2,0) = (SO3(0,2) + SO3(2,0))/2;
    }
    else if(d1*d1 > d2*d2)           // second is largest, fill with second column
    {
      r2(0,0) = (SO3(1,0) + SO3(0,1))/2;
      r2(1,0) = d1;
      r2(2,0) = (SO3(2,1) + SO3(1,2))/2;
    }
    else    // third is largest, fill with third column
    {
      r2(0,0) = (SO3(0,2) + SO3(2,0))/2;
      r2(1,0) = (SO3(2,1) + SO3(1,2))/2;
      r2(2,0) = d2;
    }

    // flip, if we point in the wrong direction!
    if(r2.transpose() * result < 0)
      r2 *= -1;

    //normalize the vector
    r2.normalize();

    //result = TooN::operator*(angle,r2);
    result = angle*r2;
  }
  return result;
}

// Finds 3D coordinates of point in reference frame A from two z=1 plane projections
Eigen::Matrix<double,3,1> VIOUtil::TriangulatePointLinearEigen(Eigen::Matrix<double,4,4> se3BtoA, const Eigen::Matrix<double,2,1> &v2A, Eigen::Matrix<double,2,1> &v2B)
{
  Eigen::Matrix<double,3,4> PDash;
  PDash.setIdentity();
  PDash.block(0,0,3,3) = VIOUtil::get_rotation(se3BtoA);
  PDash.block(0,3,3,1) = VIOUtil::get_translation(se3BtoA);

  //Compute the matrix for the least square system
  Eigen::Matrix<double,4,4> A;
  A(0,0) = -1.0; A(0,1) =  0.0; A(0,2) = v2A(0,0); A(0,3) = 0.0;
  A(1,0) =  0.0; A(1,1) = -1.0; A(1,2) = v2A(1,0); A(1,3) = 0.0;
  A.row(2) = v2B(0,0) * PDash.row(2) - PDash.row(0);
  A.row(3) = v2B(1,0) * PDash.row(2) - PDash.row(1);

  Eigen::JacobiSVD<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> > svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<double,4,4> temp = svd.matrixV();
  //The smallest is the last column
  Eigen::Matrix<double,4,1> v4Smallest = temp.transpose().block(0,3,4,1);
  if(v4Smallest(3,0) == 0.0)
    v4Smallest(3,0) = 0.00001;

  //return project(v4Smallest);
  Eigen::Matrix<double,3,1> result;
  result = v4Smallest.block(0,0,3,1)/v4Smallest(3,0);

  return result;
}

// Finds 3d coords of point in reference frame A from two z=1 plane projections
Eigen::Matrix<double,3,1> VIOUtil::TriangulatePointLinearLS(Eigen::Matrix<double,4,4> se3BtoA, const Eigen::Matrix<double,2,1> &v2A, Eigen::Matrix<double,2,1> &v2B)
{
  Eigen::Matrix<double,3,4> PDash;
  PDash.setIdentity();
  PDash.block(0,0,3,3) = VIOUtil::get_rotation(se3BtoA);
  PDash.block(0,3,3,1) = VIOUtil::get_translation(se3BtoA);

  //Define the matrix A initially as dimension 4
  Eigen::Matrix<double,4,4> A;
  A(0,0) = -1.0; A(0,1) =  0.0; A(0,2) = v2A(0,0); A(0,3) = 0.0;
  A(1,0) =  0.0; A(1,1) = -1.0; A(1,2) = v2A(1,0); A(1,3) = 0.0;
  A.row(2) = v2B(0,0) * PDash.row(2) - PDash.row(0);
  A.row(3) = v2B(1,0) * PDash.row(2) - PDash.row(1);

  //Reduce the matrix A to dimension 4 by 3
  Eigen::Matrix<double,4,3> A_red;
  A_red = A.block(0,0,4,3);

  //Define the explicit term of the system Ax=b
  Eigen::Matrix<double,4,1> b;
  b = -A.block(0,3,4,1);

  //Solve using the pseudMatrixXfo-inverse
  Eigen::JacobiSVD<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> > svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

  //Solve the least square system
  Eigen::Matrix<double,3,1> result = svd.solve(b);
  return result;
}

//This is the most efficient since it just involves a small matrix
double VIOUtil::depthFromTriangulation(Eigen::Matrix<double,4,4> se3BtoA, Eigen::Matrix<double,2,1> &v2A, Eigen::Matrix<double,2,1> &v2B, Eigen::Matrix<double,3, 1> &point)
{
  Eigen::Matrix<double,3,1> f_cur(v2A(0), v2A(1), 1);
  f_cur.normalize();
  Eigen::Matrix<double,3,1> f_ref(v2B(0), v2B(1), 1);
  f_ref.normalize();

  Eigen::Matrix<double,3,2> A;
  Eigen::Matrix<double,3,1> f_ref_rot = get_rotation(se3BtoA) * f_ref;
  A << f_cur, -f_ref_rot;
  Eigen::Matrix<double,2,2> AtA = A.transpose()*A;

   //Check if the parallax is small and give an estimation of the quality of the triangulation
  double quality = sqrt(1 - pow(f_cur.dot(f_ref_rot ),2));
  Eigen::Matrix<double,2,1> depth2 = AtA.inverse()*A.transpose()*VIOUtil::get_translation(se3BtoA);
  double depth = fabs(depth2(0,0));
  point =depth*f_cur;

  return quality;
}

//This is the most efficient since it just involves a small matrix
double VIOUtil::depthFromTriangulationSecondFrame(Eigen::Matrix<double,4,4> se3AtoB, Eigen::Matrix<double,2,1> &v2A, Eigen::Matrix<double,2,1> &v2B, Eigen::Matrix<double,3, 1> &point)
{
  Eigen::Matrix<double,3,1> f_cur(v2A(0), v2A(1), 1);
  f_cur.normalize();
  Eigen::Matrix<double,3,1> f_ref(v2B(0), v2B(1), 1);
  f_ref.normalize();
  Eigen::Matrix<double,3,2> A;
  Eigen::Matrix<double,3,1> f_cur_rot = get_rotation(se3AtoB) * f_cur;
  A << -f_cur_rot, f_ref;
  Eigen::Matrix<double,2,2> AtA = A.transpose()*A;

  //Check if the parallax is small and give an estimation of the quality of the triangulation
  double quality = sqrt(1 - pow(f_ref.dot(f_cur_rot ),2));
  Eigen::Matrix<double,2,1> depth2 = AtA.inverse()*A.transpose()*VIOUtil::get_translation(se3AtoB);
  double depth = fabs(depth2(1,0));
  point =depth*f_ref;

  return quality;
}

//Given the triangulated point and the estimated it computes the covariance the image coordinates ahave to be normalized one
void VIOUtil::PointCovariance(Eigen::Matrix<double,4,4>& se3BtoA, const Eigen::Matrix<double,3,1> &p, double focal_length, Eigen::Matrix<double,3, 3> &cov)
{
  const Eigen::Matrix<double,3,1> translation = VIOUtil::get_translation(se3BtoA);
  const Eigen::Matrix<double,3,1> t = translation.normalized();
  const Eigen::Matrix<double,3,1> f = p.normalized();
  const Eigen::Matrix<double,3,1> a = p - translation;
  const double alpha = std::acos(f.dot(t));
  const double beta = std::acos(-a.normalized().dot(t));
  const double pixel_error = 1;
  const double beta_plus = beta + 2*std::atan(pixel_error/(2*focal_length));
  const double gamma = static_cast<double>(M_PI) - alpha - beta_plus;
  const Eigen::Matrix<double,3,1> p_plus = translation * std::sin(beta_plus)/std::sin(gamma);
  Eigen::Matrix<double,3,1> delta_p = p_plus - p;
  cov = delta_p*delta_p.transpose();
}

Eigen::Matrix<double,6,6> VIOUtil::parallel_transport_helper(const Eigen::Matrix<double, 6, 1> &dx)
{
  Eigen::Matrix<double,6,6> M = Eigen::Matrix<double,6,6>::Zero();
  M.block<3,3>(3,3) = Eigen::Matrix<double,3,3>::Identity();
  M.block<3,3>(0,0) = VIOUtil::expSO3(0.5*dx.block<3,1>(0,0));

  Eigen::Matrix<double,3,1> w = dx.block<3,1>(3,0);
  const double theta = w.norm()/2.0;
  const double theta_sq = theta*theta;

  double A, B;
  //Use a Taylor series expansion near zero. This is required for
  //accuracy, since (1/t + (1-cos t)/t^2)  and (1/th^2 - (sin t)/t^3) are both 0/0.
  if (theta_sq < 1e-8)
  {
    A = 0.5;
    B = 1.0/6.0;
  }
  else
  {
    if (theta_sq < 1e-3)
    {
      A = 0.5 - theta_sq/24.0 + theta_sq * theta_sq/720.0;
      B = 1.0/6.0 - theta_sq/120.0 + theta_sq * theta_sq/5040.0;
      // we can probably drop the last terms in these
    }
    else
    {
      const double inv_theta = 1.0/theta;
      A = inv_theta*inv_theta - cos(theta)*inv_theta*inv_theta;
      B = (theta - sin(theta)) * (inv_theta * inv_theta * inv_theta);
   }
 }

  Eigen::Matrix<double,3,3> bottom_left;

  VIOUtil::rodrigues_so3_exp(w, A, B, bottom_left);
  M.block<3,3>(3,0) = bottom_left * getSkew(dx.block<3,1>(3,0));

  return M;
}

Eigen::Matrix<double,6,6> VIOUtil::parallel_transport(const Eigen::Matrix<double, 6, 1> &dx)
{
  return parallel_transport_helper(dx);
}

Eigen::Matrix<double,6,6> VIOUtil::parallel_transport_trans(const Eigen::Matrix<double, 6, 1> &dx)
{
  Eigen::Matrix<double,6,1> xi;
  xi << dx.block<3,1>(3,0), dx.block<3,1>(0,0);
  Eigen::Matrix<double,6,6> M = parallel_transport_helper(xi);

  M.block<3,3>(3,3) = M.block<3,3>(0,0);
  M.block<3,3>(0,0) = Eigen::Matrix<double,3,3>::Identity();

  M.block<3,3>(0,3) = M.block<3,3>(3,0);
  M.block<3,3>(3,0) = Eigen::Matrix<double,3,3>::Zero();

  return M;
}

//Adding more transformations
Eigen::Matrix<double, 4,1> VIOUtil::MatToQuat(const Eigen::Matrix<double, 3,3>& Rot)
{
  Eigen::Matrix<double, 4,1> Quat;
  double tr = Rot(0,0)+ Rot(1,1)+ Rot(2,2);
  int ii;
  ii=0;
  if (Rot(1,1) > Rot(0,0)) ii=1;
  if (Rot(2,2) > Rot(ii,ii)) ii=2;
  double s;
  if (tr >= 0)
  {
    s = sqrt((tr + 1));
    Quat(0,0) = s * 0.5;
    s = 0.5 / s;
    Quat(1,0) = (Rot(2,1) - Rot(1,2)) * s;
    Quat(2,0) = (Rot(0,2) - Rot(2,0)) * s;
    Quat(3,0) = (Rot(1,0) - Rot(0,1)) * s;
  }
  else
  {
    switch(ii)
    {
     case 0:
        s = sqrt((Rot(0,0)-Rot(1,1)-Rot(2,2)+1));
        Quat(1,0) = s * 0.5;
        s = 0.5 / s;

        Quat(2,0) = (Rot(1,0) + Rot(0,1)) * s;//Update pose estimation

        Quat(3,0) = (Rot(2,0) + Rot(0,2)) * s;
        Quat(0,0) = (Rot(2,1) - Rot(1,2)) * s;
        break;
     case 1:
        s = sqrt((Rot(1,1)-Rot(2,2)-Rot(0,0)+1));
        Quat(2,0) = s * 0.5;
        s = 0.5 / s;

        Quat(3,0) = (Rot(2,1) + Rot(1,2)) * s;
        Quat(1,0) = (Rot(0,1) + Rot(1,0)) * s;
        Quat(0,0) = (Rot(0,2) - Rot(2,0)) * s;
        break;
     case 2:
        s = sqrt((Rot(2,2)-Rot(0,0)-Rot(1,1)+1));
        Quat(3,0) = s * 0.5;
        s = 0.5 / s;

        Quat(1,0) = (Rot(0,2) + Rot(2,0)) * s;
        Quat(2,0) = (Rot(1,2) + Rot(2,1)) * s;
        Quat(0,0) = (Rot(1,0) - Rot(0,1)) * s;
        break;
    }
  }
  return Quat;
}


Eigen::Matrix<double, 3,3> VIOUtil::QuatToMat(const Eigen::Matrix<double, 4,1>& Quat)
{
  Eigen::Matrix<double, 3,3> Rot;
  double s = Quat(0,0);
  double x = Quat(1,0);
  double y = Quat(2,0);
  double z = Quat(3,0);
  Rot<< 1-2*(y*y+z*z),2*(x*y-s*z),2*(x*z+s*y),
          2*(x*y+s*z),1-2*(x*x+z*z),2*(y*z-s*x),
          2*(x*z-s*y),2*(y*z+s*x),1-2*(x*x+y*y);

  return Rot;
}

Eigen::Matrix<double, 3,1> VIOUtil::R_to_ypr(const Eigen::Matrix<double, 3,3>& R)
{
  Eigen::Matrix<double, 3,1> n = R.block(0,0,3,1);
  Eigen::Matrix<double, 3,1> o = R.block(0,1,3,1);
  Eigen::Matrix<double, 3,1> a = R.block(0,2,3,1);

  Eigen::Matrix<double, 3,1> ypr;
  double y = atan2(n(1,0), n(0,0));
  double p = atan2(-n(2,0), n(0,0)*cos(y)+n(1,0)*sin(y));
  double r = atan2(a(0,0)*sin(y)-a(1,0)*cos(y), -o(0,0)*sin(y)+o(1,0)*cos(y));
  ypr(0,0) = y;
  ypr(1,0) = p;
  ypr(2,0) = r;

  return ypr;
}

// Conversions
Eigen::Matrix<double,3,1> Conversions::PointToVec(const geometry_msgs::Point &p)
{
  Eigen::Matrix<double,3,1>  v;
  v << p.x, p.y, p.z, 0.0;
  return v;
}

geometry_msgs::Point Conversions::VecToPoint(const Eigen::Matrix<double,3,1> &v)
{
  geometry_msgs::Point p;
  p.x = v(0);
  p.y = v(1);
  p.z = v(2);
  return p;
}

Eigen::Matrix<double,3,1> Conversions::Vector3ToVec(const geometry_msgs::Vector3 &p)
{
  Eigen::Matrix<double,3,1>  v;
  v << p.x, p.y, p.z;
  return v;
}

geometry_msgs::Vector3 Conversions::VecToVector3(const Eigen::Matrix<double,3,1> &v)
{
  geometry_msgs::Vector3 p;
  p.x = v(0);
  p.y = v(1);
  p.z = v(2);
  return p;
}

Eigen::Quaternion<double> Conversions::QuatToQuat(const geometry_msgs::Quaternion &q)
{
  Eigen::Quaternion<double> quat(q.w,q.x,q.y,q.z);
  quat.normalize();
  return quat;
}

Eigen::Matrix<double, 3,3> VIOUtil::ypr_to_R(const Eigen::Matrix<double, 3,1>& ypr)
{

  ROS_DEBUG_STREAM("ypr_matrix: " << std::endl << ypr.matrix());
  double c, s;
  Eigen::Matrix<double, 3,3> Rz;
  Rz.setZero();
  double y = ypr(0,0);
  c = cos(y);
  s = sin(y);
  Rz(0,0) =  c;
  Rz(1,0) =  s;
  Rz(0,1) = -s;
  Rz(1,1) =  c;
  Rz(2,2) =  1;

  Eigen::Matrix<double, 3,3> Ry;
  Ry.setZero();
  double p = ypr(1,0);
  c = cos(p);
  s = sin(p);
  Ry(0,0) =  c;
  Ry(2,0) = -s;
  Ry(0,2) =  s;
  Ry(2,2) =  c;
  Ry(1,1) =  1;

  Eigen::Matrix<double, 3,3> Rx;
  Rx.setZero();
  double r = ypr(2,0);
  c = cos(r);
  s = sin(r);
  Rx(1,1) =  c;
  Rx(2,1) =  s;
  Rx(1,2) = -s;
  Rx(2,2) =  c;
  Rx(0,0) =  1;

  Eigen::Matrix<double, 3,3> R = Rz*Ry*Rx;
  return R;
}

Eigen::Matrix<double, 3, 3> VIOUtil::getSkew(const Eigen::Matrix<double, 3,1>& twist)
{
  Eigen::Matrix<double, 3, 3> S;
  S. setZero();
  S(0,1) = -twist(2,0);
  S(0,2) = twist(1,0);
  S(1,0) = twist(2,0);
  S(1,2) = -twist(0,0);
  S(2,0) = -twist(1,0);
  S(2,1) = twist(0,0);

  return S;
}

Eigen::Matrix<double, 6, 6> VIOUtil::adjSE3(const Eigen::Matrix<double, 4, 4> T)
{
  Eigen::Matrix<double, 6, 6> adj;
  adj.setZero();
  adj.block(0,0,3,3) = T.block(0,0,3,3);
  adj.block(0,3,3,3) = VIOUtil::getSkew(T.block(0,3,3,1))*T.block(0,0,3,3);
  adj.block(3,3,3,3) = T.block(0,0,3,3);
  return adj;
}
