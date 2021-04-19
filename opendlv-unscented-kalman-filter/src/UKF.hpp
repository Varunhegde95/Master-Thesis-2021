#ifndef UKF_HPP
#define UKF_HPP
/* \author Leo Wang & Varun Hegde*/
// Customized functions for Unscented Kalman Filter
// using Eigen 3

/**
 * Developer: Liangyu Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      03/2020
 */

#include <thread>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Eigen>
#include <Eigen/Dense>

class Sensor_Reading{
    public:
        Sensor_Reading(){};
        float lat = 0;   // ** GPS ** latitude of the oxts  - unit [deg]    
        float lon = 0;   // ** GPS ** longitude of the oxts - unit [deg]
        float alt = 0;   // ** GPS ** altitude of the oxts  - unit [m]
        float yaw = 0;   // heading(rad), 0 = east, positive = counter clockwise, range : [-pi .. + pi]
        float vf  = 0;   // forward velocity, i.e.parallel to earth - surface [m/s]
        float ax; // acceleration in x, i.e.in direction of vehicle front [m/s^2]
        float ay; // acceleration in y, i.e.in direction of vehicle left [m/s^2]
        float az; // acceleration in z, i.e.in direction of vehicle top [m/s^2]
        float wx; // angular rate around x [rad/s]
        float wy; // angular rate around y [rad/s]
        float wz; // angular rate around z [rad/s]
};

class Odometer
{
private:
      const int earthRadius_ = 6378388;
public:
      Odometer(){};
      float mx_;  
      float my_;  
      float mz_;  
      float ds_;

      /**
       * @brief  Initialize Odometer
       * 
       */
      void Initialize(){
         mx_ = 0.0f;
         my_ = 0.0f;
         mz_ = 0.0f;
         ds_ = 0.0f;
         std::cout << "Initialize Odometer, "; 
      }

      /** 
       * @brief Convert GPS data Latitude/Longitude/Altitude -> meters, update Odometer (mx, my). 
       * 
       * @param sensor_reading_now Latest GPS data frame
       * @param sensor_reading_pre Last GPS data frame
       */
      void GPSConvertor(const Sensor_Reading &sensor_reading_now, const Sensor_Reading &sensor_reading_pre){
         float arc = 2.0 * M_PI * (float)(earthRadius_ + sensor_reading_now.alt)/360.0; 
         float dx  = arc * (float) cos(sensor_reading_now.lat * M_PI/180.0) * (sensor_reading_now.lon - sensor_reading_pre.lon); // [m]
         float dy  = arc * (sensor_reading_now.lat - sensor_reading_pre.lat); // [m]
         //float dz = sensor_reading_now.alt - sensor_reading_pre.alt;
         mx_ += dx;
         my_ += dy;
         //mz_ += dz;
         ds_ = sqrt(dx*dx + dy*dy); 
      }
};

class UKF
{
private:
   // State dimension (Augumented)
   const int num_x_ = 15;
public:
   // sensor sampling rate 10 Hz 
   float dt_ = 0.1;
   // Sigma point spreading parameter
   const float lambda_ = 3 - num_x_;
   // Process Noise Covariance Matrix Q
   Eigen::Matrix<float, 10, 10> Q_;
   // Measurement Noise Covariance Matrix R
   Eigen::Matrix<float, 8, 8> R_;
   // Initial Uncertainty Covariance P0
   Eigen::Matrix<float, 15, 15> P0_; // 15 States: [x,y,z, V, theta(pitch),psi(roll),phi(yaw), dot_theta,dot_psi,dot_phi, 
                                    //             gamma_a, gamma_theta, gamma_psi, gamma_phi, gamma_z]
   const float gamma_a_ = 0.0;
   const float gamma_pitch_ = 0.0;
   const float gamma_roll_ = 0.0;
   const float gamma_yaw_ = 0.0;
   const float gamma_z_ = 0.0;

   Eigen::Matrix<float, 15, 1> x_f_;  // Filtered Estimates
   Eigen::Matrix<float, 15, 15> p_f_; // Filtered Error Covariance
   Eigen::Matrix<float, 10, 1> x_p_;  // Predicted Estimates
   Eigen::Matrix<float, 10, 10> p_p_; // Predicted Error Covariance

   // Sigma points and weights
   Eigen::Matrix<float, 15, 31> SP_; // Sigma points
   Eigen::Matrix<float, 1, 31> W_;   // Sigma point Weights
   Eigen::Matrix<float, 10, 31> SP_predict_; // f(Sigma points) [f: Motion model function(CTRV / CV)]
   Eigen::Matrix<float, 10, 21> SP_U_; // Sigma points in update step
   Eigen::Matrix<float, 1, 21> W_U_;   // Sigma point Weights in update step 

   // Measurement matrix
   Eigen::Matrix<float, 8, 1> measurements_;

   /**
    * @brief Set process noise covariance matrix Q [10, 10]
    * 
    * @param sGPS GPS noise covariance
    * @param sCourse Turn angle noise covariance
    * @param sTurnRate Turn rate noise covariance
    */
   void SetProcessNoiseCovatiance(const float &sGPS, const float &sCourse, 
                                    const float &sTurnRate){
      Eigen::Matrix<float, 10, 10> i(Eigen::MatrixXf::Identity(10, 10));  
      Eigen::DiagonalMatrix<float, 10> m;
      float dt(dt_);
      m.diagonal() << (0.5 * sGPS * dt * dt) * (0.5 * sGPS * dt * dt), (0.5 * sGPS * dt * dt) *(0.5 * sGPS * dt * dt), 
                     (0.5 * sGPS * dt * dt) * (0.5 * sGPS * dt * dt), (sGPS * dt) * (sGPS * dt), 
                     (sCourse * dt) * (sCourse * dt), (sCourse * dt) * (sCourse * dt), (sCourse * dt) * (sCourse * dt), 
                     (sTurnRate * dt) * (sTurnRate * dt), (sTurnRate * dt) * (sTurnRate * dt), (sTurnRate * dt) * (sTurnRate * dt);
      Q_ = i * m;
   }

   /**
    * @brief Set measurement noise covariance R [8, 8]
    * 
    * @param var_GPS 
    * @param var_speed 
    * @param var_course 
    * @param var_turn_angle 
    */
   void SetMeasureNoiseCovatiance(const float &var_GPS, const float &var_speed, const float &var_course, const float &var_turn_angle){
      Eigen::Matrix<float, 8, 8> i(Eigen::MatrixXf::Identity(8, 8));  
      Eigen::DiagonalMatrix<float, 8> m;
      m.diagonal() << var_GPS * var_GPS, var_GPS * var_GPS, var_GPS * var_GPS, var_speed * var_speed, var_course * var_course,
                     var_turn_angle * var_turn_angle, var_turn_angle * var_turn_angle, var_turn_angle * var_turn_angle;
      R_ = i * m;
   }

   /**
    * @brief Set initial uncertainty P0 [15, 15]
    * 
    */
   void SetInitialCovariance(){
      Eigen::Matrix<float, 15, 15> i(Eigen::MatrixXf::Identity(15, 15));  
      P0_ = i * 1000.0;
   }

   /**
    * @brief Initialize State: Q, R, P0, x_f_ = X0,  p_f_ = P0 
    * 
    * @param odom Odometer data
    * @param sensor_reading Sensor data
    */
   void Initialize(const Odometer &odom, const Sensor_Reading &sensor_reading){
      x_p_ = Eigen::MatrixXf::Zero(10,1);
      p_p_ = Eigen::MatrixXf::Zero(10,10);
      measurements_ = Eigen::MatrixXf::Zero(8, 1);
      // UKF Initialize noise covariance
      SetProcessNoiseCovatiance(8.8f, 0.1f, 1.0f); // Initialize Q
      SetMeasureNoiseCovatiance(10.0f, 1.0f, 0.05f, 0.0017f); // Initialize R
      SetInitialCovariance(); // Initialize P0
      x_f_ << odom.mx_, odom.my_, odom.mz_, sensor_reading.vf, 0.0f, 0.0f, sensor_reading.yaw, sensor_reading.wy/180*M_PI, 
               sensor_reading.wx/180*M_PI, sensor_reading.wz/180*M_PI, gamma_a_, gamma_pitch_, gamma_roll_, gamma_yaw_, gamma_z_;
      p_f_ = P0_;
      std::cout << "Initialize UKF" << std::endl;
   }

   /**
    * @brief  Generate Sigma Points and sigma point Weights
    * 
    * @param x Filtered state
    * @param P Filtered covariance
    */
   void GenerateSigmaPoints(const Eigen::MatrixXf &x, const Eigen::MatrixXf &P){
      SP_.fill(0.0);
      W_.fill(0.0);
      SP_.col(0) = x; 
      // Calculate square root of P
      Eigen::MatrixXf A = P.llt().matrixL(); 
      for(int i = 0; i < num_x_; i++){
         SP_.col(i + 1) = x + sqrt(lambda_ + num_x_) * A.col(i);
         SP_.col(i + 1 + num_x_) = x - sqrt(lambda_ + num_x_) * A.col(i);
      }
      W_ = Eigen::MatrixXf::Ones(1, 31) * (0.5 / (num_x_ + lambda_)); // Sigma point weights [W]
      W_(0) = (float) lambda_ / (num_x_ + lambda_);
   }

   /**
    * @brief Predict Sigma Points. Choose between CTRV and CV motion model based on the yaw rate.
    * 
    * @param SP Sigma points
    * @param W Sigma point weights
    * @param dt Time interval 
    */
   void PredictSigmaPoints(const Eigen::MatrixXf &SP, const Eigen::MatrixXf &W, const float &dt){
      x_p_.fill(0.0); // Initialize Predicted State Mean
      p_p_ = Q_; // Initialize Predicted State Covariance
      for(int i = 0; i < 2 * num_x_ + 1; i++){
         float p_x(SP(0, i));    // pos x
         float p_y(SP(1, i));    // pos y
         float p_z(SP(2, i));    // pos z
         float v  (SP(3, i));    // Speed
         float pitch(SP(4, i));  // Pitch
         float roll (SP(5, i));  // Roll
         float yaw  (SP(6, i));  // Yaw
         float pitch_rate(SP(7, i));   // Pitch Rate
         float roll_rate (SP(8, i));   // Roll Rate
         float yaw_rate  (SP(9, i));   // Yaw Rate
         float gamma_acc  (SP(10, i)); // Gamma_Acceleration
         float gamma_pitch(SP(11, i)); // Gamma_Pitch
         float gamma_roll (SP(12, i)); // Gamma_Roll
         float gamma_yaw  (SP(13, i)); // Gamma_Yaw
         float gamma_z    (SP(14, i)); // Gamma_Z
         // CTRV motion model
         if(fabs(yaw_rate) > 0.001){
            SP_predict_(0, i) = p_x + v/yaw_rate * ((float)sin(yaw + yaw_rate * dt) - (float)sin(yaw)) + 0.5 * dt * dt * (float)cos(yaw) * gamma_acc;  // Position X
            SP_predict_(1, i) = p_y + v/yaw_rate * ((float)cos(yaw) - (float)cos(yaw + yaw_rate * dt)) + 0.5 * dt * dt * (float)sin(yaw) * gamma_acc;  // Position Y
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
            SP_predict_(0, i) = p_x + v * (float)cos(yaw) * dt + 0.5 * dt * dt * (float)cos(yaw) * gamma_acc; // Position X 
            SP_predict_(1, i) = p_y + v * (float)sin(yaw) * dt + 0.5 * dt * dt * (float)sin(yaw) * gamma_acc; // Position Y
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
         Eigen::Matrix<float, 10, 1> x_diff(SP_predict_.col(i) - x_p_);
         p_p_ += W(i) * x_diff * x_diff.transpose(); // Predicted State Covariance
      }
   }

   /**
    * @brief Predict state vector and covariance
    * 
    * @param x Filtered state
    * @param P Filtered covariance
    */
   void Prediction(const Eigen::MatrixXf &x, const Eigen::MatrixXf &P){
      GenerateSigmaPoints(x, P);
      PredictSigmaPoints(SP_, W_, dt_);
   }

   /**
    * @brief Read sensor data and save to 'measurement_' matrix [7x1]
    * 
    * @param odom Odometer data
    * @param sensor_reading Sensor data
    */
   void GetMeasurement(const Odometer &odom, const Sensor_Reading &sensor_reading){
      measurements_ << odom.mx_, odom.my_, odom.mz_, sensor_reading.vf, sensor_reading.yaw, 
         sensor_reading.wy/180*M_PI, sensor_reading.wx/180*M_PI, sensor_reading.wz/180*M_PI;
   }

   /**
    * @brief UKF Update step
    * 
    * @param x Predicted state from 'Prediction' step
    * @param P Predicted covariance from 'Prediction' step
    * @param measure Measurement for updating
    * @param odom Odometer for updating
    */
   void Update(const Eigen::MatrixXf &x, const Eigen::MatrixXf &P, 
               const Eigen::MatrixXf &measure, const Odometer &odom){
      // Generate Update Sigma Points 
      int lambda = 3 - P.rows();
      SP_U_.col(0) = x; 
      // Calculate square root of P
      Eigen::MatrixXf C = P.llt().matrixL(); // Cholskey decompposition
      for(int i = 0; i < x.rows(); i++){
         SP_U_.col(i + 1) = x + sqrt(lambda + x.rows()) * C.col(i);
         SP_U_.col(i + 1 + x.rows()) = x - sqrt(lambda + x.rows()) * C.col(i);
      }
      UKF::W_U_ = Eigen::MatrixXf::Ones(1, 21) * (0.5 / (x.rows() + lambda)); // Sigma point weights [W]
      UKF::W_U_(0) = (float) lambda / (x.rows() + lambda);

      /*----- Update Measurements -----*/
      // Only IMU
      if(odom.ds_ == 0){
         Eigen::MatrixXf y_hat(Eigen::MatrixXf::Zero(3,1));
         for(int i = 0; i < W_U_.cols(); i++){
            y_hat(0) += SP_U_(7, i) * W_U_(i);
            y_hat(1) += SP_U_(8, i) * W_U_(i);
            y_hat(2) += SP_U_(9, i) * W_U_(i);
         }
         Eigen::Matrix<float, 10, 3> P_xy(Eigen::MatrixXf::Zero(10, 3));
         Eigen::Matrix<float, 3, 3> S;
         S << R_(4, 4), 0,  0, 
            0, R_(5, 5) , 0, 
            0, 0 , R_(6, 6);
         Eigen::Matrix<float, 3, 21> hSP;
         hSP << SP_U_.row(7), SP_U_.row(8), SP_U_.row(9);
         for(int i = 0; i < W_U_.cols(); i++){
            P_xy += W_U_(i) * (SP_U_.col(i) - x) * (hSP.col(i) - y_hat).transpose();
            S += W_U_(i) * (hSP.col(i) - y_hat) * (hSP.col(i) - y_hat).transpose();
         }
         // Calculate updated mean and covariance
         Eigen::Matrix<float, 3, 1> y;
         Eigen::Matrix<float, 10, 1> x_f;  // Predicted Estimates without augment
         Eigen::Matrix<float, 10, 10> p_f; // Predicted Error Covariance without augment
         y << measure(5), measure(6), measure(7); // Pitch rate, Roll rate, Yaw rate
         x_f = x + P_xy * S.inverse() * (y - y_hat);
         p_f = P - P_xy * S.inverse() * P_xy.transpose();
         x_f_.block<10,1>(0,0) = x_f;
         p_f_.block<10,10>(0,0) = p_f;
      }
      // Both GPS & IMU
      else{
         Eigen::MatrixXf y_hat(Eigen::MatrixXf::Zero(8,1));
         for(int i = 0; i < W_U_.cols(); i++){
            y_hat(0) += SP_U_(0, i) * W_U_(i); // Position X
            y_hat(1) += SP_U_(1, i) * W_U_(i); // Position Y
            y_hat(2) += SP_U_(2, i) * W_U_(i); // Position Z
            y_hat(3) += SP_U_(3, i) * W_U_(i); // Speed V
            y_hat(4) += SP_U_(6, i) * W_U_(i); // Yaw angle
            y_hat(5) += SP_U_(7, i) * W_U_(i); // Pitch Rate
            y_hat(6) += SP_U_(8, i) * W_U_(i); // Roll Rate
            y_hat(7) += SP_U_(9, i) * W_U_(i); // Yaw Rate
         }
         Eigen::Matrix<float, 10, 8> P_xy(Eigen::MatrixXf::Zero(10, 8));
         Eigen::Matrix<float, 8, 8> S(R_);
         Eigen::Matrix<float, 8, 21> hSP;
         hSP << SP_U_.row(0), SP_U_.row(1),SP_U_.row(2), SP_U_.row(3), SP_U_.row(6), SP_U_.row(7), SP_U_.row(8), SP_U_.row(9);
         for(int i = 0; i < W_U_.cols(); i++){
            P_xy += W_U_(i) * (SP_U_.col(i) - x) * (hSP.col(i) - y_hat).transpose();
            S += W_U_(i) * (hSP.col(i) - y_hat) * (hSP.col(i) - y_hat).transpose();
         }
         // Calculate updated mean and covariance
         Eigen::Matrix<float, 8, 1> y(measurements_); // GPS + IMU
         Eigen::Matrix<float, 10, 1> x_f;  // Predicted Estimates without augment
         Eigen::Matrix<float, 10, 10> p_f; // Predicted Error Covariance without augment
         x_f = x + P_xy * S.inverse() * (y - y_hat);
         p_f = P - P_xy * S.inverse() * P_xy.transpose();
         x_f_.block<10,1>(0,0) = x_f;
         p_f_.block<10,10>(0,0) = p_f;
      }
   }
};

/**
 * @brief Calculate the time consumption. 
 * Have to use "auto start_time = std::chrono::system_clock::now()" to start timer.
 * 
 * @param start_time Choose starting Timer
 * @param function_name Function name shown in terminal
 */
void timerCalculator (const std::chrono::_V2::system_clock::time_point &start_time,
                      const std::string &function){
    // Should use "auto start_fast = std::chrono::system_clock::now()" to start timer.
    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double time_passed = (double) duration.count() * 
            std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;
    std::cout << "[--Timer--] " << function <<" time used: " << time_passed << " [s]." << std::endl;
    //return time_passed; // [seconds]
}

#endif /*UKF_HPP*/