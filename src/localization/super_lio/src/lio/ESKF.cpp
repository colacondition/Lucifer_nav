#include "lio/ESKF.h"

#include <algorithm>
#include <cmath>

using namespace BASIC;

namespace LI2Sup{

namespace
{

bool IsFiniteImu(const IMUData & imu)
{
  return std::isfinite(imu.secs) && imu.acc.allFinite() && imu.gyr.allFinite();
}

}


/// [1] SEfB: P195：（7.37a） P203: (7.76a) (7.77a)  P220: Table 7-2 
/// [2] VSLAM14: P71: (4.26)  P73: (4.32)
/// [3] Proportional Derivative (PD) Control on the Euclidean Group.  A_matrix
/// [1] = [2] <===> [3]
static inline M3d RightJacobianSO3(const V3& ang_vel, const scalar& dt){
  V3d ang_vel_d = ang_vel.template cast<double>();
  scalar ang_vel_norm = ang_vel_d.norm();
  if (ang_vel_norm < 1e-8){   /// noise? too small
    return M3d::Identity();
  }else{
    V3d r_axis = ang_vel_d / ang_vel_norm;
    double r_ang = ang_vel_norm * dt;
    M3d K;
    K << 0.0, -r_axis[2], r_axis[1], r_axis[2], 0.0, -r_axis[0], -r_axis[1], r_axis[0], 0.0;
    double a = (1 - std::cos(r_ang)) / r_ang;
    double b = 1 - std::sin(r_ang) / r_ang;
    return M3d::Identity() - a * K + b * K * K;   // J_r(phi) = J_l(-phi)
  }
}


/// left jacobian
M3d A_matrix(const V3 & v){
  M3d res;
  double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
	double norm = std::sqrt(squaredNorm);
	if(norm < 1e-6){
		res = M3d::Identity();
	}
	else{
    M3d K;
    K << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
		res = M3d::Identity() + (1 - std::cos(norm)) / squaredNorm * K 
        + (1 - std::sin(norm) / norm) / squaredNorm * K * K;
	}
  return res;
}


void ESKF::SetInitialConditions(Options options, const V3& init_bg, 
                                const V3& init_ba, const float imu_scale,
                                const V3& gravity) 
{
  BuildNoise(options);
  options_ = options;
  bg_ = init_bg;
  ba_ = init_ba;
  g_ = gravity;
  imu_scale_ = imu_scale;

  P_ = 1e-4 * M18::Identity();
  P_.template block<3, 3>(0, 0) = 0.1 * M_PI / 180.0 * M3::Identity();   // r

}


void ESKF::SetX(const SysState& x) {
  last_imu_time_ = x.timestamp;      // TODO: The timestamp update is not strictly consistent.
  current_time_ = last_imu_time_;
  R_ = x.R;
  p_ = x.p;
  v_ = x.v;
  bg_ = x.bg;
  ba_ = x.ba;
  fw_R_ = R_;
  fw_p_ = p_;
  fw_v_ = v_;
}


void ESKF::BuildNoise(const Options& options) {
  double et = options.gyro_var_;
  double ev = options.acce_var_;
  double eg = options.bias_gyro_var_;
  double ea = options.bias_acce_var_;

  double et2 = et;  // * et;
  double ev2 = ev;  // * ev;
  double eg2 = eg;  // * eg;
  double ea2 = ea;  // * ea;

  // set Q
  Q_.diagonal() << et2, et2, et2,    // ng: r
                   ev2, ev2, ev2,    // na: v  -> p
                   eg2, eg2, eg2,    // nbg: bg -> w -> r
                   ea2, ea2, ea2;    // nba: ba -> a -> v -> p
}

bool ESKF::Update() {
  if (!dx_.allFinite()) {
    return false;
  }

  const SO3 next_R = R_ * SO3::Exp(dx_.template block<3, 1>(0, 0));
  const V3 next_p = p_ + dx_.template block<3, 1>(3, 0);
  const V3 next_v = v_ + dx_.template block<3, 1>(6, 0);
  const V3 next_bg = bg_ + dx_.template block<3, 1>(9, 0);
  const V3 next_ba = ba_ + dx_.template block<3, 1>(12, 0);
  V3 next_g = g_ + dx_.template block<3, 1>(15, 0);
  const scalar gravity_norm = next_g.norm();
  if (!next_R.R_.allFinite() || !next_p.allFinite() || !next_v.allFinite() ||
      !next_bg.allFinite() || !next_ba.allFinite() || !next_g.allFinite() ||
      !std::isfinite(gravity_norm) || gravity_norm <= scalar(1e-6)) {
    return false;
  }

  next_g = g_gravity_norm * (next_g / gravity_norm);
  R_ = next_R;
  p_ = next_p;
  v_ = next_v;
  bg_ = next_bg;
  ba_ = next_ba;
  g_ = next_g;
  fw_R_ = R_;
  fw_p_ = p_;
  fw_v_ = v_;
  forward_time_ = current_obs_time_;
  return true;
}


bool ESKF::StateIsFinite() const {
  return R_.R_.allFinite() && p_.allFinite() && v_.allFinite() &&
         bg_.allFinite() && ba_.allFinite() && g_.allFinite() &&
         P_.allFinite() && std::isfinite(current_time_) &&
         std::isfinite(last_imu_time_) && std::isfinite(last_obs_time_) &&
         std::isfinite(current_obs_time_);
}


void ESKF::ResetForwardPrediction(const IMUData& imu) {
  fw_R_ = R_;
  fw_p_ = p_;
  fw_v_ = v_;
  forward_time_ = imu.secs;
  forward_last_imu_ = imu;
}


bool ESKF::Predict(const IMUData& imu, DynamicState& state_imu, DynamicState& state_robot){
  if(!init_ || !IsFiniteImu(imu) || !StateIsFinite()) {
    return false;
  }

  /// first time predict
  if(forward_time_ < 0){
    ResetForwardPrediction(imu);
    return false;
  }

  double dt = imu.secs - forward_time_;

  if(!std::isfinite(dt) || dt <= 0 || dt > 0.2){
    ResetForwardPrediction(imu);
    return false;
  }

  V3 acc = 0.5 * (imu.acc + forward_last_imu_.acc);
  acc = imu_scale_ * acc;
  acc = acc - ba_;
  
  V3 gyr = 0.5 * (imu.gyr + forward_last_imu_.gyr) - bg_;
  V3 new_p = fw_p_ + fw_v_ * dt + 0.5 * (fw_R_.R() * acc) * dt * dt + 0.5 * g_ * dt * dt;
  V3 new_v = fw_v_ + fw_R_.R() * acc * dt + g_ * dt;
  SO3 new_R = fw_R_ * SO3::Exp(gyr , dt);

  if (!acc.allFinite() || !gyr.allFinite() || !new_p.allFinite() ||
      !new_v.allFinite() || !new_R.R_.allFinite()) {
    ResetForwardPrediction(imu);
    return false;
  }

  fw_R_ = new_R;
  fw_v_ = new_v;
  fw_p_ = new_p;

  state_imu.time = imu.secs;
  state_imu.R = fw_R_.R_;
  state_imu.p = fw_p_;
  state_imu.v = fw_v_;
  state_imu.w = gyr;
  state_imu.a = acc;
  forward_time_ = imu.secs;
  forward_last_imu_ = imu;

  new_R.R_ = fw_R_.R_ * g_odom_robo.R_;
  new_p = fw_R_.R_ * ( - g_odom_robo.R_ * g_odom_robo.t_) + fw_p_;
  state_robot.time = imu.secs;
  state_robot.R = new_R.R_;
  state_robot.p = new_p;
  // todo: v, w, a

  return true;
}


bool ESKF::Predict(const IMUData& imu) {

  if (!IsFiniteImu(imu) || !StateIsFinite()) {
    return false;
  }

  if(last_imu_time_ < 0){
    last_imu_time_ = imu.secs;
    last_imu_ = imu;
    return false;
  }

  if(imu.secs <= last_obs_time_){
    last_imu_time_ = imu.secs;
    last_imu_ = imu;
    return false;
  }
  
  current_time_ = imu.secs;

  double dt;
  if(last_imu_time_ < last_obs_time_){
    dt = imu.secs - last_obs_time_;
  }else if (imu.secs > current_obs_time_){
    dt = current_obs_time_ - last_imu_time_;
    current_time_ = current_obs_time_;
  }else{
    dt = imu.secs - last_imu_time_;
  }

  if (!std::isfinite(dt) || dt <= 0 || dt > 0.2) {
    last_imu_time_ = imu.secs;
    last_imu_ = imu;
    return false;
  }

  V3 acc = 0.5 * (imu.acc + last_imu_.acc);
  acc = imu_scale_ * acc;
  acc = acc - ba_;
  body_omega_ = 0.5 * (imu.gyr + last_imu_.gyr) - bg_;
  M3 Jr_dt = (dt * RightJacobianSO3(body_omega_, dt)).cast<scalar>();   // J_l(-phi) = J_r(phi)

  M3 R_m3 = R_.R_;
  M3 R_dt = R_m3 * dt;

  F_X f_x = F_X::Identity();
  f_x.template block<3, 3>(0, 0) = SO3::Exp(-body_omega_, dt).R_;
  f_x.template block<3, 3>(0, 9) = - Jr_dt;
  f_x.template block<3, 3>(3, 6) = M3::Identity() * dt;
  f_x.template block<3, 3>(6, 0) = - R_m3 * SO3::hat(acc) * dt;
  f_x.template block<3, 3>(6, 12) = - R_dt;
  f_x.template block<3, 3>(6, 15) = M3::Identity() * dt;
  

  F_W f_w = F_W::Zero();
  f_w.template block<3, 3>(0, 0) = - Jr_dt;
  f_w.template block<3, 3>(6, 3) = - R_dt;                 // v -> na
  f_w.template block<3, 3>(9, 6) = M3::Identity() * dt;    // ba
  f_w.template block<3, 3>(12, 9) = M3::Identity() * dt;   // bg

  const M18 next_P = f_x * P_ * f_x.transpose() + f_w * Q_ * f_w.transpose();

  const V3 next_global_acc = R_.R() * acc + g_;
  const V3 next_p = p_ + v_ * dt + 0.5 * next_global_acc * dt * dt;
  const V3 next_v = v_ + next_global_acc * dt;
  const SO3 next_R = R_ * SO3::Exp(body_omega_, dt);
  if (!next_P.allFinite() || !next_global_acc.allFinite() ||
      !next_p.allFinite() || !next_v.allFinite() || !next_R.R_.allFinite()) {
    last_imu_time_ = imu.secs;
    last_imu_ = imu;
    return false;
  }

  P_ = next_P;
  global_acc_ = next_global_acc;
  p_ = next_p;
  v_ = next_v;
  R_ = next_R;

  last_imu_time_ = imu.secs;
  last_imu_ = imu;
  return true;
}


const int STATE_DIM = 18;
bool ESKF::UpdateObserve(ESKF::ObsFunc obs) {
  const SO3 previous_R = R_;
  const V3 previous_p = p_;
  const V3 previous_v = v_;
  const V3 previous_bg = bg_;
  const V3 previous_ba = ba_;
  const V3 previous_g = g_;
  const V3 previous_global_acc = global_acc_;
  const V3 previous_body_omega = body_omega_;
  const M18 previous_P = P_;
  const double previous_last_obs_time = last_obs_time_;
  const double previous_forward_time = forward_time_;
  const SO3 previous_fw_R = fw_R_;
  const V3 previous_fw_p = fw_p_;
  const V3 previous_fw_v = fw_v_;
  const auto restore = [&]() {
    R_ = previous_R;
    p_ = previous_p;
    v_ = previous_v;
    bg_ = previous_bg;
    ba_ = previous_ba;
    g_ = previous_g;
    global_acc_ = previous_global_acc;
    body_omega_ = previous_body_omega;
    P_ = previous_P;
    last_obs_time_ = previous_last_obs_time;
    forward_time_ = previous_forward_time;
    fw_R_ = previous_fw_R;
    fw_p_ = previous_fw_p;
    fw_v_ = previous_fw_v;
    dx_.setZero();
  };

  SO3 R_0 = R_;

  M6 HTVH;     // H^T * V^(-1) * H
  V6 HTVr;     // H^T * V^(-1) * residuals
  M18 Pk, Qk;

  need_converge_ = false;
  for (int iter = 0; iter < options_.num_iterations_; ++iter) {
    if(iter > 2){
      need_converge_ = true;
    }

    HTVH.setZero();
    HTVr.setZero();
    obs(GetKFState(), HTVH, HTVr);
    if (!HTVH.allFinite() || !HTVr.allFinite()) {
      restore();
      return false;
    }
    /**
     * J = diag(I_3, I_3, J_theta, I_3, I_3, I_3)
     * J_theta = I - 1/2 * δθ_k ∧
     * δθ_k = Log(R_k^T * R_0)  ?  --> Log(R_0^T * R_k) ?
     */
    Pk = P_;
    M3 J_theta = M3::Identity() - 0.5 * SO3::hat((R_0.inverse() * R_).log_vee());
    for(int j = 0; j < STATE_DIM; j+=3){
      Pk.block<3,3>(0, j).noalias() = J_theta * P_.block<3,3>(0, j);
    }
    for(int j = 0; j < STATE_DIM; j+=3){
      Pk.block<3,3>(j, 0) = Pk.block<3,3>(j, 0) * J_theta.transpose();
    }

    /**
     * The normal Kalman filter formula: 
     * K_k = P_k * H_k^T * (H_k * P_k * H_k^T + V)^(-1)
     * δx_k = K_k * (z - h(x_k))
     * 
     * Using SMW identity transformation：
     * (A * B * (D + C * A * B)^(-1)) = ((A^(-1) + B * D^(-1) * C)^(-1) * B * D^(-1))
     * K_k = (P_k^(-1) + H_k^T * V^(-1) * H_k)^(-1) * H_k^T * V^(-1)
     * Q_k = (P_k^(-1) + H_k^T * V^(-1) * H_k)^(-1)
     * HTVr = H_k^T * V^(-1) * residuals
     * residuals = (z - h(x_k))
     */

    if (!Pk.allFinite()) {
      restore();
      return false;
    }
    M18 Pk_inv = Pk.inverse();
    Pk_inv.block<6,6>(0,0) += HTVH;

    Qk = Pk_inv.inverse();
    if (!Pk_inv.allFinite() || !Qk.allFinite()) {
      restore();
      return false;
    }

    V18 error_dx = V18::Zero();
    error_dx.head(6) = HTVr;

    dx_ = Qk * error_dx;

    // update norm state
    if (!Update()) {
      restore();
      return false;
    }

    if (dx_.lpNorm<Eigen::Infinity>() < options_.quit_eps_ && iter > 0) {
      break;
    }
  }

  // update P
  M18 temp_cov = M18::Identity();
  temp_cov.block<6,6>(0,0) = HTVH;
  P_ = (M18::Identity() - Qk * temp_cov) * Pk;

  Pk = P_;

  // project P
  M3 J_theta = M3::Identity() - 0.5 * SO3::hat(dx_.template block<3, 1>(0, 0));
  for(int j = 0; j < STATE_DIM; j+=3){
    Pk.block<3,3>(0, j).noalias() = J_theta * P_.block<3,3>(0, j);
  }
  for(int j = 0; j < STATE_DIM; j+=3){
    Pk.block<3,3>(j, 0) = Pk.block<3,3>(j, 0) * J_theta.transpose();
  }

  P_ = Pk;

  if (!StateIsFinite()) {
    restore();
    return false;
  }

  dx_.setZero();
  
  last_obs_time_ = current_obs_time_;
  return true;
}


bool ESKF::ApplyVerticalConstraint(const scalar z_ref,
                                   const scalar pos_std,
                                   const scalar vel_std,
                                   const scalar max_pos_correction)
{
  if (!init_ || pos_std <= scalar(0) || !std::isfinite(z_ref)) {
    return false;
  }

  const bool constrain_velocity = vel_std > scalar(0);
  const int residual_dim = constrain_velocity ? 2 : 1;

  Eigen::Matrix<scalar, Eigen::Dynamic, 18> H(residual_dim, 18);
  Eigen::Matrix<scalar, Eigen::Dynamic, 1> residual(residual_dim);
  Eigen::Matrix<scalar, Eigen::Dynamic, Eigen::Dynamic> R(residual_dim, residual_dim);
  H.setZero();
  residual.setZero();
  R.setZero();

  residual(0) = z_ref - p_(2);
  if (max_pos_correction > scalar(0)) {
    residual(0) = std::clamp(residual(0), -max_pos_correction, max_pos_correction);
  }
  H(0, 5) = scalar(1);
  R(0, 0) = pos_std * pos_std;

  if (constrain_velocity) {
    residual(1) = -v_(2);
    H(1, 8) = scalar(1);
    R(1, 1) = vel_std * vel_std;
  }

  const M18 P_prev = P_;
  const Eigen::Matrix<scalar, Eigen::Dynamic, Eigen::Dynamic> S =
    H * P_prev * H.transpose() + R;
  const Eigen::Matrix<scalar, 18, Eigen::Dynamic> K =
    P_prev * H.transpose() * S.inverse();
  dx_ = K * residual;
  if (!Update()) {
    dx_.setZero();
    return false;
  }

  const M18 I = M18::Identity();
  const M18 IKH = I - K * H;
  P_ = IKH * P_prev * IKH.transpose() + K * R * K.transpose();
  P_ = scalar(0.5) * (P_ + P_.transpose()).eval();

  if (!StateIsFinite()) {
    P_ = P_prev;
    dx_.setZero();
    return false;
  }

  dx_.setZero();
  return true;
}

}
