/* \author Leo Wang & Varun Hegde*/
// Customized Supporting function for pointcloud processing 
// using PCL

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      03/2020
 */
#ifndef UKF_H_
#define UKF_H_

#include <thread>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Eigen>
#include <Eigen/Dense>

class Oxts_Data{
    public:
        Oxts_Data(){};
        double lat;   // ** GPS ** latitude of the oxts  - unit [deg]    
        double lon;   // ** GPS ** longitude of the oxts - unit [deg]
        double alt;   // ** GPS ** altitude of the oxts  - unit [m]

        double roll;  // roll angle(rad), 0 = level, positive = left side up, range :  [-pi .. + pi]
        double pitch; // pitch angle(rad), 0 = level, positive = front down, range :   [-pi/2 .. + pi/2]
        double yaw;   // heading(rad), 0 = east, positive = counter clockwise, range : [-pi .. + pi]

        double vn; // velocity towards north [m/s]
        double ve; // velocity towards east [m/s]
        double vf; // forward velocity, i.e.parallel to earth - surface [m/s]
        double vl; // leftward velocity, i.e.parallel to earth - surface [m/s]
        double vu; // upward velocity, i.e.perpendicular to earth - surface [m/s]

        double ax; // acceleration in x, i.e.in direction of vehicle front [m/s^2]
        double ay; // acceleration in y, i.e.in direction of vehicle left [m/s^2]
        double az; // acceleration in z, i.e.in direction of vehicle top [m/s^2]
        double af; // forward acceleration [m/s^2]
        double al; // leftward acceleration [m/s^2]
        double au; // upward acceleration [m/s^2]

        double wx; // angular rate around x [rad/s]
        double wy; // angular rate around y [rad/s]
        double wz; // angular rate around z [rad/s]
        double wf; // angular rate around forward axis [rad/s]
        double wl; // angular rate around leftward axis [rad/s]
        double wu; // angular rate around upward axis [rad/s]
};

class Odometer
{
    public:
        Odometer(){};
        double mx_;  
        double my_;  
        double mz_;  
        double ds_;

        void Initialize();
        void GPSConvertor(const Oxts_Data &measurement_now, const Oxts_Data &measurement_pre);

    private:
        const int earthRadius_ = 6378388;

};

class UKF
{
private:
    // sensor sampling rate 10 Hz 
    const double dt_ = 0.1;
    // State dimension (Augumented)
    const int num_x_ = 15;
public:
    // Augmented state dimension
    //const int num_aug_x_;
    // Sigma point spreading parameter
    const double lambda_ = 3 - num_x_;
    // Process Noise Covariance Matrix Q
    Eigen::Matrix<double, 10, 10> Q_;
    // Measurement Noise Covariance Matrix R
    Eigen::Matrix<double, 8, 8> R_;
    // Initial Uncertainty Covariance P0
    Eigen::Matrix<double, 15, 15> P0_; // 15 States: [x,y,z, V, theta(pitch),psi(roll),phi(yaw), dot_theta,dot_psi,dot_phi, 
                                      //             gamma_a, gamma_theta, gamma_psi, gamma_phi, gamma_z]
    const double gamma_a_ = 0.0;
    const double gamma_pitch_ = 0.0;
    const double gamma_roll_ = 0.0;
    const double gamma_yaw_ = 0.0;
    const double gamma_z_ = 0.0;

    Eigen::Matrix<double, 15, 1> x_f_;  // Filtered Estimates
    Eigen::Matrix<double, 15, 15> p_f_; // Filtered Error Covariance
    Eigen::Matrix<double, 10, 1> x_p_;  // Predicted Estimates
    Eigen::Matrix<double, 10, 10> p_p_; // Predicted Error Covariance

    // Sigma points and weights
    Eigen::Matrix<double, 15, 31> SP_; // Sigma points
    Eigen::Matrix<double, 1, 31> W_;   // Sigma point Weights
    Eigen::Matrix<double, 10, 31> SP_predict_; // f(Sigma points) [f: Motion model function(CTRV / CV)]
    Eigen::Matrix<double, 10, 21> SP_U_; // Sigma points in update step
    Eigen::Matrix<double, 1, 21> W_U_;   // Sigma point Weights in update step 

    // Measurement matrix
    Eigen::Matrix<double, 8, 1> measurements_;

    void SetProcessNoiseCovatiance(const double &sGPS = 8.8, const double &sCourse = 0.1, const double &sTurnRate = 1.0);
    void SetMeasureNoiseCovatiance(const double &var_GPS = 6.0, const double &var_speed = 1.0, const double &var_course = 0.01, const double &var_turn_angle = 0.01);
    void SetInitialCovariance();
    void Initialize(const Odometer &odom, const Oxts_Data &oxts_data); // Initializa UKF (X_0, P_0)
    void GetMeasurement(const Odometer &odom, const Oxts_Data &oxts_data);
    void GenerateSigmaPoints(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P);
    void PredictSigmaPoints(const Eigen::MatrixXd &SP, const Eigen::MatrixXd &W, const double &dt);
    void Prediction(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P);
    void Update(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P, const Eigen::MatrixXd &measure, const Odometer &odom);
    void Plot(const std::vector<double> &GPSX, const std::vector<double> &GPSY, const std::vector<double> &filteredX, const std::vector<double> &filteredY);
};

#endif /* UKF_H_ */