/*
 * utilities.h
 *
 *  Created on: Aug 2, 2023
 *      Author: marvin
 */


#include <Eigen/Dense>
#include <mutex>

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <fstream>
#include <random>

#include <cmath>
#include <chrono>



#pragma once

using namespace Eigen;
using json = nlohmann::json;

inline void printVecXD(const std::vector<Eigen::VectorXd>& vec) {
    for (size_t i = 0; i < vec.size(); ++i) {
        std::cout << "Vector " << i << ": ";
        for (int j = 0; j < vec[i].size(); ++j) {
            std::cout << vec[i](j) << " ";  // Print each component of the vector
        }
        std::cout << std::endl;  // Move to the next line after printing one Eigen::VectorXd
    }
}

// this is a template for thread safe access of anything (pretty much i hope), defaults to "0" or whatever that is
template<typename Content, typename Mutex = std::mutex>
class ThreadSafe {
    mutable Mutex mtx;
    Content value;

public:
    // Default constructor - value-initialized Content
    ThreadSafe() : value(Content{}) {}

    // Copy constructor
    ThreadSafe(const Content& other) {
        std::scoped_lock lock(mtx);
        value = other;
    }

    // Move constructor
    ThreadSafe(Content&& other) noexcept(std::is_nothrow_move_constructible_v<Content>) {
        std::scoped_lock lock(mtx);
        value = std::move(other);
    }

    // Copy assignment operator
    ThreadSafe& operator=(const Content& other) {
        if (&other != &value) {
            std::scoped_lock lock(mtx);
            value = other;
        }
        return *this;
    }

    // Move assignment operator
    ThreadSafe& operator=(Content&& other) noexcept(std::is_nothrow_move_assignable_v<Content>) {
        if (&other != &value) {
            std::scoped_lock lock(mtx);
            value = std::move(other);
        }
        return *this;
    }

    // Type conversion operator to get the value
    operator Content() const {
        std::scoped_lock lock(mtx);
        return value;
    }

    Content get() const {
        std::scoped_lock lock(mtx);
        return value;
}
};



// Interface
template <typename Content, typename Mutex = std::mutex >
class MutexedWrapper
{
       mutable Mutex mtx;
       Content value;
public:
       Content Get() const { std::scoped_lock lock(mtx); return value; }
       void Set(Content novum) { std::scoped_lock lock(mtx); value = std::move(novum); }
};

inline std::tuple<VectorXd, VectorXd, VectorXd> compute_steady_state(const MatrixXd& A, const MatrixXd& B, const MatrixXd& C, double desired_temperature) {
    std::cout << "start steady state computation" << std::endl;

    MatrixXd I = MatrixXd::Identity(A.rows(), A.cols());
    MatrixXd inv_term = (I - A).inverse();

    VectorXd desired_T = VectorXd::Constant(1, desired_temperature);
    VectorXd u_ss = desired_T * (C * inv_term * B).inverse();
    VectorXd x_ss = inv_term * B * u_ss;
    VectorXd y_ss = C * x_ss;

    std::cout << "steady state " << x_ss << std::endl;
    std::cout << "Tss = " << y_ss << " and " << desired_temperature << std::endl;
    std::cout << "uss = " << u_ss << std::endl;

    return std::make_tuple(x_ss, y_ss, u_ss);
}

inline void printMatrix(const std::string& name, const Eigen::MatrixXd& M)
{
    std::cout << name << ":\n" << M << "\n\n";
}

inline void importMatrices(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D, Eigen::MatrixXd& Q, Eigen::MatrixXd& R,Eigen::MatrixXd& K, const std::string& filepath)
{
    // Create a json object
    nlohmann::json j;

    // Read from the file
    std::ifstream file(filepath);
    if (file.is_open())
    {
        file >> j;
        file.close();
    }
    else
    {
        std::cout << "Failed to open file " << filepath << std::endl;
        return;
    }
    // Loop through each matrix and fill
    for (auto& [key, value] : j.items())
    {
    // Check if this is one of the matrix keys we care about
    if (key == "A" || key == "B" || key == "C" || key == "D" || key == "Q" || key == "R" || key == "K") {
        // Ensure value is an array of arrays
        if (!value.is_array() || value.empty() || !value[0].is_array()) {
            std::cout << "Key " << key << " is not a matrix. Skipping.\n";
            continue;
        }

        int rows = static_cast<int>(value.size());
        int cols = static_cast<int>(value[0].size());
        Eigen::MatrixXd M(rows, cols);

        for (int i = 0; i < rows; i++)
        {
            for (int k = 0; k < cols; k++)
            {
                M(i, k) = value[i][k].get<double>();
            }
        }

        if (key == "A") A = M;
        else if (key == "B") B = M;
        else if (key == "C") C = M;
        else if (key == "D") D = M;
        else if (key == "Q") Q = M;
        else if (key == "R") R = M;
        else if (key == "K") K = M;
    } else {
        // Non-matrix keys (like eps_ricatti) can be handled elsewhere or ignored
    }
}


    // Print the matrices
    std::cout << "A:\n" << A << std::endl;
    std::cout << "B:\n" << B << std::endl;
    std::cout << "C:\n" << C << std::endl;
    std::cout << "D:\n" << D << std::endl;
    std::cout << "Q:\n" << Q << std::endl;
    std::cout << "R:\n" << R << std::endl;

}


inline double clipToRange(double value, double minValue, double maxValue) {
    return std::min(std::max(value, minValue), maxValue);
}


inline void solveRicatti(const Eigen::MatrixXd &AdT, const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                  const Eigen::MatrixXd &R, const Eigen::MatrixXd &BdT, const Eigen::MatrixXd &Q,
                  Eigen::MatrixXd &P, Eigen::MatrixXd &K_lqr,
                  double eps = 1e-6, uint max_iterations = 100000) {
  Eigen::MatrixXd P_next;
  Eigen::MatrixXd Rinv = R.inverse();
  double diff;

  for (uint i = 0; i < max_iterations; ++i) {
    // -- discrete solver --
    P_next = AdT * P * A - AdT * P * B * (R + BdT * P * B).inverse() * BdT * P * A + Q;

    diff = fabs((P_next - P).maxCoeff());
    P = P_next;
    if (diff < eps) {
      std::cout << "iteration number = " << i << std::endl;
      std::cout << "P: " << P << std::endl;
      break;
    }
  }
  std::cout << "Ricatti solved" << std::endl;

  K_lqr = Rinv * BdT * P;
}



using json = nlohmann::json;

// Function to expand '~' to the home directory in file paths
inline std::string expandTilde(const std::string& path) {
    if (!path.empty() && path[0] == '~') {
        const char* home = getenv("HOME");
        if (home || (home = getenv("USERPROFILE"))) {
            return std::string(home) + path.substr(1);
        } else {
            // If HOME is not set, return the path as is
            return path;
        }
    }
    return path;
}

// Helper functions to convert JSON arrays to Eigen matrices and vectors
inline Eigen::MatrixXd jsonToMatrix(const nlohmann::json& j) {
    size_t rows = j.size();
    size_t cols = j[0].size();
    Eigen::MatrixXd mat(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t k = 0; k < cols; ++k) {
            mat(i, k) = j[i][k];
        }
    }
    return mat;
}

inline Eigen::VectorXd jsonToVector(const nlohmann::json& j) {
    size_t size = j.size();
    Eigen::VectorXd vec(size);
    for (size_t i = 0; i < size; ++i) {
        vec(i) = j[i];
    }
    return vec;
}