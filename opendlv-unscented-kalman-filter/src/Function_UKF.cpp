#include "UKF.h"
/* \author Leo Wang & Varun Hegde*/
// Customized Supporting function for pointcloud processing 
// using PCL

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      03/2020
 */

// Set process noise covariance matrix Q [10, 10]
void 
 UKF::SetProcessNoiseCovatiance(const double &sGPS, const double &sCourse, 
                                 const double &sTurnRate){
   Eigen::Matrix<double, 10, 10> i(Eigen::MatrixXd::Identity(10, 10));  
   Eigen::DiagonalMatrix<double, 10> m;
   double dt(dt_);
   m.diagonal() << (0.5 * sGPS * dt * dt) * (0.5 * sGPS * dt * dt), (0.5 * sGPS * dt * dt) *(0.5 * sGPS * dt * dt), 
                   (0.5 * sGPS * dt * dt) * (0.5 * sGPS * dt * dt), (sGPS * dt) * (sGPS * dt), 
                   (sCourse * dt) * (sCourse * dt), (sCourse * dt) * (sCourse * dt), (sCourse * dt) * (sCourse * dt), 
                   (sTurnRate * dt) * (sTurnRate * dt), (sTurnRate * dt) * (sTurnRate * dt), (sTurnRate * dt) * (sTurnRate * dt);
   Q_ = i * m;
}

// Set measurement noise covariance R [8, 8]
void 
 UKF::SetMeasureNoiseCovatiance(const double &var_GPS, const double &var_speed, const double &var_course, const double &var_turn_angle){
   Eigen::Matrix<double, 8, 8> i(Eigen::MatrixXd::Identity(8, 8));  
   Eigen::DiagonalMatrix<double, 8> m;
   m.diagonal() << var_GPS * var_GPS, var_GPS * var_GPS, var_GPS * var_GPS, var_speed * var_speed, var_course * var_course,
                   var_turn_angle * var_turn_angle, var_turn_angle * var_turn_angle, var_turn_angle * var_turn_angle;
   R_ = i * m;
}

// Set initial uncertainty P0 [15, 15]
void 
 UKF::SetInitialCovariance(){
   Eigen::Matrix<double, 15, 15> i(Eigen::MatrixXd::Identity(15, 15));  
   P0_ = i * 1000.0;
}

// Initialize State x_f_ = X0 and p_f_ = P0 
void 
 UKF::Initialize(const Odometer &odom, const Oxts_Data &oxts_data){
   x_p_ = Eigen::MatrixXd::Zero(10,1);
   p_p_ = Eigen::MatrixXd::Zero(10,10);
   measurements_ = Eigen::MatrixXd::Zero(8, 1);
   // UKF Initialize noise covariance
   SetProcessNoiseCovatiance(8.8, 0.1, 1.0); // Initialize Q
   SetMeasureNoiseCovatiance(6.0, 1.0, 0.01, 0.0017); // Initialize R
   SetInitialCovariance(); // Initialize P0
   x_f_ << odom.mx_, odom.my_, odom.mz_, oxts_data.vf, 0.0, 0.0, oxts_data.yaw, oxts_data.wy/180*M_PI, 
                  oxts_data.wx/180*M_PI, oxts_data.wz/180*M_PI, gamma_a_, gamma_pitch_, gamma_roll_, gamma_yaw_, gamma_z_;
   p_f_ = P0_;
}

// Generate Sigma Points and sigma point Weights
void 
 UKF::GenerateSigmaPoints(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P){
   SP_.fill(0.0);
   W_.fill(0.0);
   SP_.col(0) = x; 
   // Calculate square root of P
   Eigen::MatrixXd A = P.llt().matrixL(); 
   for(int i = 0; i < num_x_; i++){
      SP_.col(i + 1) = x + sqrt(lambda_ + num_x_) * A.col(i);
      SP_.col(i + 1 + num_x_) = x - sqrt(lambda_ + num_x_) * A.col(i);
   }
   W_ = Eigen::MatrixXd::Ones(1, 31) * (0.5 / (num_x_ + lambda_)); // Sigma point weights [W]
   W_(0) = (double) lambda_ / (num_x_ + lambda_);
}

// Predict Sigma Points
void 
 UKF::PredictSigmaPoints(const Eigen::MatrixXd &SP, const Eigen::MatrixXd &W, const double &dt){
   x_p_.fill(0.0); // Initialize Predicted State Mean
   p_p_ = Q_; // Initialize Predicted State Covariance
   for(int i = 0; i < 2 * num_x_ + 1; i++){
      double p_x(SP(0, i)); // pos x
      double p_y(SP(1, i)); // pos y
      double p_z(SP(2, i)); // pos z
      double v  (SP(3, i)); // Speed
      double pitch(SP(4, i));  // Pitch
      double roll (SP(5, i));  // Roll
      double yaw  (SP(6, i));  // Yaw
      double pitch_rate(SP(7, i)); // Pitch Rate
      double roll_rate (SP(8, i)); // Roll Rate
      double yaw_rate  (SP(9, i)); // Yaw Rate
      double gamma_acc  (SP(10, i)); // Gamma_Acceleration
      double gamma_pitch(SP(11, i)); // Gamma_Pitch
      double gamma_roll (SP(12, i)); // Gamma_Roll
      double gamma_yaw  (SP(13, i)); // Gamma_Yaw
      double gamma_z    (SP(14, i)); // Gamma_Z
      // CTRV motion model
      if(fabs(yaw_rate) > 0.001){
         SP_predict_(0, i) = p_x + v/yaw_rate * ((double)sin(yaw + yaw_rate * dt) - (double)sin(yaw)) + 0.5 * dt * dt * (double)cos(yaw) * gamma_acc;  // Position X
         SP_predict_(1, i) = p_y + v/yaw_rate * ((double)cos(yaw) - (double)cos(yaw + yaw_rate * dt)) + 0.5 * dt * dt * (double)sin(yaw) * gamma_acc;  // Position Y
         SP_predict_(2, i) = p_z + dt * gamma_z;   // Position Z [correct]
         SP_predict_(3, i) = v + dt * gamma_acc;   // Speed V [correct]
         SP_predict_(4, i) = pitch + pitch_rate * dt + 0.5 * dt * dt * gamma_pitch; // Pitch
         SP_predict_(5, i) = roll + roll_rate * dt + 0.5 * dt * dt * gamma_roll;    // Roll
         SP_predict_(6, i) = yaw + yaw_rate * dt + 0.5 * dt * dt * gamma_yaw;       // Yaw [correct]
         SP_predict_(7, i) = pitch_rate + dt * gamma_pitch; // Pitch Rate
         SP_predict_(8, i) = roll_rate + dt * gamma_roll;   // Roll Rate
         SP_predict_(9, i) = yaw_rate + dt * gamma_yaw;     // Yaw Rate [correct]
      }
      // CV model
      else{
         SP_predict_(0, i) = p_x + v * (double)cos(yaw) * dt + 0.5 * dt * dt * (double)cos(yaw) * gamma_acc; // Position X 
         SP_predict_(1, i) = p_y + v * (double)sin(yaw) * dt + 0.5 * dt * dt * (double)sin(yaw) * gamma_acc; // Position Y
         SP_predict_(2, i) = p_z + dt * gamma_z;   // Position Z
         SP_predict_(3, i) = v + dt * gamma_acc;   // Speed V
         SP_predict_(4, i) = pitch + pitch_rate * dt + 0.5 * dt * dt * gamma_pitch; // Pitch
         SP_predict_(5, i) = roll + roll_rate * dt + 0.5 * dt * dt * gamma_roll;    // Roll
         SP_predict_(6, i) = yaw + yaw_rate * dt + 0.5 * dt * dt * gamma_yaw;       // Yaw
         SP_predict_(7, i) = pitch_rate + dt * gamma_pitch; // Pitch Rate
         SP_predict_(8, i) = roll_rate + dt * gamma_roll;   // Roll Rate
         SP_predict_(9, i) = yaw_rate + dt * gamma_yaw;     // Yaw Rate
      }
      x_p_ += SP_predict_.col(i) * W(i); // Predicted State Mean
   }
   for(int i = 0; i < 2 * num_x_ + 1; i++){
      // State Difference
      Eigen::Matrix<double, 10, 1> x_diff(SP_predict_.col(i) - x_p_);
      p_p_ += W(i) * x_diff * x_diff.transpose(); // Predicted State Covariance
   }
}

// Predict state vector and covariance
void 
 UKF::Prediction(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P){
   GenerateSigmaPoints(x, P);
   PredictSigmaPoints(SP_, W_, dt_);
}

// Read sensor data and save to 'measurement_' matrix [7x1]
void 
 UKF::GetMeasurement(const Odometer &odom, const Oxts_Data &oxts_data){
   measurements_ << odom.mx_, odom.my_, odom.mz_, oxts_data.vf, oxts_data.yaw, oxts_data.wy*M_PI/180, oxts_data.wx*M_PI/180, oxts_data.wz*M_PI/180;
}

void 
 UKF::Update(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P, 
             const Eigen::MatrixXd &measure, const Odometer &odom){
   // Generate Update Sigma Points 
   int lambda = 3 - P.rows();
   SP_U_.col(0) = x; 
   // Calculate square root of P
   Eigen::MatrixXd C = P.llt().matrixL(); // Cholskey decompposition
   for(int i = 0; i < x.rows(); i++){
      SP_U_.col(i + 1) = x + sqrt(lambda + x.rows()) * C.col(i);
      SP_U_.col(i + 1 + x.rows()) = x - sqrt(lambda + x.rows()) * C.col(i);
   }
   UKF::W_U_ = Eigen::MatrixXd::Ones(1, 21) * (0.5 / (x.rows() + lambda)); // Sigma point weights [W]
   UKF::W_U_(0) = (double) lambda / (x.rows() + lambda);

   // Update Measurements
   // Only IMU
   if(odom.ds_ == 0){
      Eigen::MatrixXd y_hat(Eigen::MatrixXd::Zero(3,1));
      for(int i = 0; i < W_U_.cols(); i++){
         y_hat(0) += SP_U_(7, i) * W_U_(i);
         y_hat(1) += SP_U_(8, i) * W_U_(i);
         y_hat(2) += SP_U_(9, i) * W_U_(i);
      }
      Eigen::Matrix<double, 10, 3> P_xy(Eigen::MatrixXd::Zero(10, 3));
      Eigen::Matrix<double, 3, 3> S;
      S << R_(4, 4), 0,  0, 
           0, R_(5, 5) , 0, 
           0, 0 , R_(6, 6);
      Eigen::Matrix<double, 3, 21> hSP;
      hSP << SP_U_.row(7), SP_U_.row(8), SP_U_.row(9);
      for(int i = 0; i < W_U_.cols(); i++){
         P_xy += W_U_(i) * (SP_U_.col(i) - x) * (hSP.col(i) - y_hat).transpose();
         S += W_U_(i) * (hSP.col(i) - y_hat) * (hSP.col(i) - y_hat).transpose();
      }
      // Calculate updated mean and covariance
      Eigen::Matrix<double, 3, 1> y;
      Eigen::Matrix<double, 10, 1> x_f;  // Predicted Estimates without augment
      Eigen::Matrix<double, 10, 10> p_f; // Predicted Error Covariance without augment
      y << measure(5), measure(6), measure(7); // Pitch rate, Roll rate, Yaw rate
      x_f = x + P_xy * S.inverse() * (y - y_hat);
      p_f = P - P_xy * S.inverse() * P_xy.transpose();
      x_f_.block<10,1>(0,0) = x_f;
      p_f_.block<10,10>(0,0) = p_f;
   }
   // Both GPS & IMU
   else{
      Eigen::MatrixXd y_hat(Eigen::MatrixXd::Zero(8,1));
      for(int i = 0; i < W_U_.cols(); i++){
         y_hat(0) += SP_U_(0, i) * W_U_(i); // Position X
         y_hat(1) += SP_U_(1, i) * W_U_(i); // Position Y
         y_hat(2) += SP_U_(2, i) * W_U_(i); // Position Z
         y_hat(3) += SP_U_(3, i) * W_U_(i); // Speed V
         y_hat(4) += SP_U_(6, i) * W_U_(i); // Yaw
         y_hat(5) += SP_U_(7, i) * W_U_(i); // Pitch Rate
         y_hat(6) += SP_U_(8, i) * W_U_(i); // Roll Rate
         y_hat(7) += SP_U_(9, i) * W_U_(i); // Yaw Rate
      }
      Eigen::Matrix<double, 10, 8> P_xy(Eigen::MatrixXd::Zero(10, 8));
      Eigen::Matrix<double, 8, 8> S(R_);
      Eigen::Matrix<double, 8, 21> hSP;
      hSP << SP_U_.row(0), SP_U_.row(1),SP_U_.row(2), SP_U_.row(3), SP_U_.row(6), SP_U_.row(7), SP_U_.row(8), SP_U_.row(9);
      for(int i = 0; i < W_U_.cols(); i++){
         P_xy += W_U_(i) * (SP_U_.col(i) - x) * (hSP.col(i) - y_hat).transpose();
         S += W_U_(i) * (hSP.col(i) - y_hat) * (hSP.col(i) - y_hat).transpose();
      }
      // Calculate updated mean and covariance
      Eigen::Matrix<double, 8, 1> y(measurements_); // GPS + IMU
      Eigen::Matrix<double, 10, 1> x_f;  // Predicted Estimates without augment
      Eigen::Matrix<double, 10, 10> p_f; // Predicted Error Covariance without augment
      x_f = x + P_xy * S.inverse() * (y - y_hat);
      p_f = P - P_xy * S.inverse() * P_xy.transpose();
      x_f_.block<10,1>(0,0) = x_f;
      p_f_.block<10,10>(0,0) = p_f;
   }
}

// void 
//  UKF::Plot(const std::vector<double> &GPSX, const std::vector<double> &GPSY, 
//           const std::vector<double> &filteredX, const std::vector<double> &filteredY){
//    matplotlibcpp::clf(); // Clear [matplotlib] previous plot
//    matplotlibcpp::scatter(GPSX, GPSY, 8);
//    matplotlibcpp::named_plot("Filtered", filteredX, filteredY, "r-"); // Filtered positions
//    matplotlibcpp::title("UKF Result");
//    matplotlibcpp::legend(); // Enable legend
//    matplotlibcpp::grid(true); // Enable Grid
//    matplotlibcpp::pause(0.001);  // Display plot continuously
// }

// Initialize odometers to 0. 
void 
 Odometer::Initialize(){
    Odometer::mx_ = 0.0;
    Odometer::my_ = 0.0;
    Odometer::mz_ = 0.0;
    Odometer::ds_ = 0.0; 
}

// Convert GPS data Latitude/Longitude/Altitude -> meters, update Odometer (mx, my, mz).
void
 Odometer::GPSConvertor(const Oxts_Data &sensor_data_now, const Oxts_Data &sensor_data_pre){
   double arc = 2.0 * M_PI * (double)(Odometer::earthRadius_ + sensor_data_now.alt)/360.0; 
   double dx = arc * (double) cos(sensor_data_now.lat * M_PI/180.0) * (sensor_data_now.lon - sensor_data_pre.lon); // [m]
   double dy = arc * (sensor_data_now.lat - sensor_data_pre.lat); // [m]
   double dz = sensor_data_now.alt - sensor_data_pre.alt;
   Odometer::mx_ += dx;
   Odometer::my_ += dy;
   Odometer::mz_ += dz;
   Odometer::ds_ = sqrt(dx*dx + dy*dy); 
}