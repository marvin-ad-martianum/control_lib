#ifndef KALMAN_LQR_CONTROLLER_H
#define KALMAN_LQR_CONTROLLER_H

#include <Eigen/Dense>
#include <tuple>
#include <string>

class KalmanLQRController {
public:
    KalmanLQRController(const std::string& json_file_path);
    KalmanLQRController();

    // Initialize the system with new parameters
    void start_system(const std::string& json_file_path);

    // Set a new reference (for MIMO, a vector matching the dimension of the output)
    void setNewReference(const Eigen::VectorXd& reference);

    // Compute the control input based on current state estimate and reference
    Eigen::VectorXd computeControlInput();

    // Retrieve the predicted output (dimension p×1, where p = number of outputs)
    Eigen::VectorXd getPredictedOutput() const;

    // Retrieve the current state estimate (dimension n×1)
    Eigen::VectorXd getStateEstimate() const;

    // Retrieve the steady-state input (dimension m×1)
    Eigen::VectorXd getUss() const;

    Eigen::VectorXd getYresidual() const;


    // Internal initialization and computations
    void intitalize();
    void loadLTIModel(const std::string& json_file_path);
    void computeLQRGain();
    void computeKalmanGain();
    void updateStateEstimate(const Eigen::VectorXd& u, const Eigen::VectorXd& y);
    

    // Steady-state computations
    std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> ComputeSteadyStateLSQ(const Eigen::VectorXd& target);

    // Dimension checks
    void checkDimensions() const;

private:
    // System Matrices
    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd C_;
    Eigen::MatrixXd D_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;

    // Noise Covariances
    Eigen::MatrixXd Qk_;
    Eigen::MatrixXd Rk_;

    // State and Kalman filter variables
    Eigen::VectorXd x_hat_;      // State estimate
    Eigen::VectorXd y_hat_;      // Output estimate
    Eigen::VectorXd y_residual;  // Residual
    Eigen::MatrixXd P_;          // Error covariance

    // LQR gain
    Eigen::MatrixXd K_;

    // Kalman gain
    Eigen::MatrixXd Kf_;

    // Steady-state values
    Eigen::VectorXd x_ss;
    Eigen::VectorXd y_ss;
    Eigen::VectorXd u_ss;

    // References
    Eigen::VectorXd current_reference_vec_; // Store current reference as a vector

    // Parameters
    double eps_ricatti_;
    unsigned int max_iter_ricatti_;
    double process_noise_cov_;
    double measurement_noise_cov_;


};

#endif // KALMAN_LQR_CONTROLLER_H
