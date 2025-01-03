#include "kalman_lqr_controller.h"
//#include <Eigen/Dense>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <iostream>


KalmanLQRController::KalmanLQRController(const std::string& json_file_path)
{
    // Initialize references to empty vector for now, todo move yaml to constructor
    std::cout << "kalman lqr system starting init..." << std::endl;
    loadLTIModel(json_file_path);
    std::cout << "json loaded computing ricatti..." << std::endl;
    intitalize();
    std::cout << "kalman lqr system initialized" << std::endl;
}

KalmanLQRController::KalmanLQRController()
{
    current_reference_vec_.resize(0);
}


// for the empty constructor
void KalmanLQRController::start_system(const std::string& json_file_path)
{
    std::cout << "kalman lqr system starting init..." << std::endl;
    loadLTIModel(json_file_path);
    std::cout << "json loaded computing ricatti..." << std::endl;
    intitalize();
    std::cout << "kalman lqr system initialized" << std::endl;
}


void KalmanLQRController::intitalize()
{
    // Initialize estimates
    x_hat_ = Eigen::VectorXd::Zero(A_.rows());
    y_hat_ = Eigen::VectorXd::Zero(C_.rows());
    y_residual = Eigen::VectorXd::Zero(C_.rows());

    // Initialize error covariance matrix
    P_ = Eigen::MatrixXd::Identity(A_.rows(), A_.rows());

    // Initialize process and measurement noise covariance matrices
    Qk_ = Eigen::MatrixXd::Identity(A_.rows(), A_.rows()) * process_noise_cov_;
    Rk_ = Eigen::MatrixXd::Identity(C_.rows(), C_.rows()) * measurement_noise_cov_;

    // Compute LQR gain
    computeLQRGain();

    // Initialize reference to zero vector (dimension = number of outputs)
    current_reference_vec_ = Eigen::VectorXd::Zero(C_.rows());
    std::tie(x_ss, y_ss, u_ss) = ComputeSteadyStateLSQ(current_reference_vec_);
}

void KalmanLQRController::loadLTIModel(const std::string& json_file_path)
{
    std::ifstream json_file(json_file_path);
    if (!json_file.is_open())
    {
        throw std::runtime_error("Unable to open JSON file: " + json_file_path);
    }

    Json::Value root;
    json_file >> root;

    auto jsonToMatrix = [](const Json::Value& jsonArray) -> Eigen::MatrixXd {
        int rows = jsonArray.size();
        int cols = jsonArray[0].size();
        Eigen::MatrixXd mat(rows, cols);
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < cols; ++j)
                mat(i, j) = jsonArray[i][j].asDouble();
        return mat;
    };

    A_ = jsonToMatrix(root["A"]);
    B_ = jsonToMatrix(root["B"]);
    C_ = jsonToMatrix(root["C"]);
    D_ = jsonToMatrix(root["D"]);
    Q_ = jsonToMatrix(root["Q"]);
    R_ = jsonToMatrix(root["R"]);

    // Perform dimension checks
    checkDimensions();

    // Optional: Print matrices
    std::cout << "A:\n" << A_ << std::endl;
    std::cout << "B:\n" << B_ << std::endl;
    std::cout << "C:\n" << C_ << std::endl;
    std::cout << "D:\n" << D_ << std::endl;

    // Load additional parameters
    if (root.isMember("eps_ricatti"))
        eps_ricatti_ = root["eps_ricatti"].asDouble();
    else
        throw std::runtime_error("Missing 'eps_ricatti' in JSON.");

    if (root.isMember("max_iter_ricatti"))
        max_iter_ricatti_ = root["max_iter_ricatti"].asUInt();
    else
        throw std::runtime_error("Missing 'max_iter_ricatti' in JSON.");

    if (root.isMember("process_noise_cov"))
        process_noise_cov_ = root["process_noise_cov"].asDouble();
    else
        throw std::runtime_error("Missing 'process_noise_cov' in JSON.");

    if (root.isMember("measurement_noise_cov"))
        measurement_noise_cov_ = root["measurement_noise_cov"].asDouble();
    else
        throw std::runtime_error("Missing 'measurement_noise_cov' in JSON.");

    std::cout << "Loaded system parameters:" << std::endl;
    std::cout << "eps_ricatti: " << eps_ricatti_ << std::endl;
    std::cout << "max_iter_ricatti: " << max_iter_ricatti_ << std::endl;
    std::cout << "process_noise_cov: " << process_noise_cov_ << std::endl;
    std::cout << "measurement_noise_cov: " << measurement_noise_cov_ << std::endl;
}

void KalmanLQRController::checkDimensions() const
{
    int n = A_.rows(); // state dimension
    if (A_.cols() != n) {
        throw std::runtime_error("Matrix A must be square.");
    }

    // Check B dimensions: must be n x m
    if (B_.rows() != n) {
        throw std::runtime_error("Matrix B must have the same number of rows as A.");
    }

    int m = B_.cols(); // input dimension

    // Check C dimensions: must be p x n
    if (C_.cols() != n) {
        throw std::runtime_error("Matrix C must have the same number of columns as A.");
    }

    int p = C_.rows(); // output dimension

    // Check D dimensions: must be p x m
    if ((D_.rows() != p) || (D_.cols() != m)) {
        throw std::runtime_error("Matrix D dimensions must match output and input dimensions.");
    }

    // Check Q dimensions: must be n x n
    if ((Q_.rows() != n) || (Q_.cols() != n)) {
        throw std::runtime_error("Matrix Q must be n x n, where n is the state dimension.");
    }

    // Check R dimensions: must be m x m
    if ((R_.rows() != m) || (R_.cols() != m)) {
        throw std::runtime_error("Matrix R must be m x m, where m is the input dimension.");
    }
}


void KalmanLQRController::computeLQRGain()
{
    // Initialize the Riccati solver parameters
    Eigen::MatrixXd P = Q_; // Start with Q_ as the initial guess
    Eigen::MatrixXd P_next = P;

    Eigen::MatrixXd AdT = A_.transpose();
    Eigen::MatrixXd BdT = B_.transpose();

    double diff;

    for (unsigned int i = 0; i < max_iter_ricatti_; ++i)
    {
        // Discrete Riccati equation iteration
        P_next = AdT * P * A_ - AdT * P * B_ * (R_ + BdT * P * B_).inverse() * BdT * P * A_ + Q_;

        // Compute the maximum absolute difference between P and P_next
        diff = (P_next - P).cwiseAbs().maxCoeff();

        // Update P for the next iteration
        P = P_next;

        // Check for convergence
        if (diff < eps_ricatti_)
        {
            std::cout << "Riccati solver converged at iteration: " << i << std::endl;
            break;
        }
    }

    // After the loop, P contains the solution to the Riccati equation
    std::cout << "Riccati equation solved." << std::endl;
    // std::cout << "P matrix:\n" << P << std::endl;

    // Compute the LQR gain K
    K_ = (R_ + BdT * P * B_).inverse() * BdT * P * A_;
    std::cout << "LQR gain K:\n" << K_ << std::endl;
}

void KalmanLQRController::computeKalmanGain()
{
    // Predict error covariance
    P_ = A_ * P_ * A_.transpose() + Qk_;

    // Compute Kalman gain
    Eigen::MatrixXd S = C_ * P_ * C_.transpose() + Rk_;
    Kf_ = P_ * C_.transpose() * S.inverse();
}

void KalmanLQRController::updateStateEstimate(const Eigen::VectorXd& u, const Eigen::VectorXd& y)
{
    // Check dimensions first!
    if (u.size() != B_.cols()) {
        throw std::runtime_error("Input vector u dimension mismatch.");
    }
    if (y.size() != C_.rows()) {
        throw std::runtime_error("Output vector y dimension mismatch.");
    }

    // Prediction step
    x_hat_ = A_ * x_hat_ + B_ * u;

    // Update error covariance
    /**P_ = A_ * P_ * A_.transpose() + Qk_;

    // Compute Kalman gain
    Eigen::MatrixXd S = C_ * P_ * C_.transpose() + Rk_;
    Kf_ = P_ * C_.transpose() * S.inverse();**/
    computeKalmanGain();

    // Measurement residual
    y_residual = y - C_ * x_hat_;

    // Update estimate with measurement
    x_hat_ = x_hat_ + Kf_ * y_residual;

    // Update error covariance
    P_ = (Eigen::MatrixXd::Identity(A_.rows(), A_.rows()) - Kf_ * C_) * P_;
}

Eigen::VectorXd KalmanLQRController::getPredictedOutput() const {
    return C_ * x_hat_;
}
Eigen::VectorXd KalmanLQRController::getYresidual() const {
    return y_residual;
}

Eigen::VectorXd KalmanLQRController::getStateEstimate() const{
    return x_hat_;
}

Eigen::VectorXd KalmanLQRController::getUss() const{
    return u_ss;
}

// add all components to the refrence todo:
void KalmanLQRController::setNewReference(const Eigen::VectorXd& reference)
{
    if ((reference - current_reference_vec_).norm() > 1e-12) {
        std::tie(x_ss, y_ss, u_ss) = ComputeSteadyStateLSQ(reference);
        current_reference_vec_ = reference;
    }
}

Eigen::VectorXd KalmanLQRController::computeControlInput()
{
    // control_signal = -K_*(x_hat_-x_ss) + u_ss
    // Here K_ is dimension m×n, x_hat_-x_ss is n×1, result is m×1
    auto control_signal = -(K_ * (x_hat_ - x_ss)) + u_ss; 
    return control_signal;
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> 
KalmanLQRController::ComputeSteadyStateLSQ(const Eigen::VectorXd& target) {
    std::cout << "start steady state computation lsq" << std::endl;

    int n = A_.rows();
    int p = C_.rows();

    if (target.size() != p) {
        throw std::runtime_error("Target vector dimension must match output dimension (C_.rows()).");
    }

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd M = I - A_;

    double det = M.determinant();
    if (std::abs(det) < 1e-12) {
        throw std::runtime_error("Matrix (I - A) is singular. Cannot compute steady state for this system.");
    }

    Eigen::MatrixXd inv_term = M.inverse();
    Eigen::MatrixXd CB = C_ * inv_term * B_;

    Eigen::VectorXd u_ss = CB.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(target);
    Eigen::VectorXd x_ss = inv_term * B_ * u_ss;
    Eigen::VectorXd y_ss = C_ * x_ss;

    return std::make_tuple(x_ss, y_ss, u_ss);
}
