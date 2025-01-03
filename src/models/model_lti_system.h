#pragma once

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <random>

class LTIModelSystem
{
public:
    LTIModelSystem() : generator_(std::random_device{}()) {}

    /**
     * Load the LTI model (A, B, C, D) from a YAML node.
     */
    void loadFromYaml(const YAML::Node &root)
    {
        if (!root["model_lti_system"]) {
            throw std::runtime_error("YAML missing 'model_lti_system' key");
        }
        YAML::Node node = root["model_lti_system"];

        // Parse matrices
        A_ = parse2DMatrix(node["A"], "A");
        B_ = parse2DMatrix(node["B"], "B");
        C_ = parse2DMatrix(node["C"], "C");
        D_ = parse2DMatrix(node["D"], "D");

        // Initialize state
        state_.resize(A_.rows());
        state_.setZero();

        // Dimension checks
        if (A_.rows() != A_.cols()) {
            throw std::runtime_error("Matrix A must be square");
        }
        if (B_.rows() != A_.rows()) {
            throw std::runtime_error("B's row count must match A's dimension");
        }
        if (C_.cols() != A_.cols()) {
            throw std::runtime_error("C's column count must match A's dimension");
        }
        if (D_.rows() != C_.rows() || D_.cols() != B_.cols()) {
            throw std::runtime_error("D must match dimension (PxM) where P=C.rows, M=B.cols");
        }
    }

    /**
     * Load from a YAML file path (optional convenience).
     */
    void loadFromFile(const std::string &yaml_path)
    {
        std::ifstream inFile(yaml_path);
        if (!inFile.is_open()) {
            throw std::runtime_error("Could not open LTI model YAML file: " + yaml_path);
        }
        YAML::Node root = YAML::Load(inFile);
        loadFromYaml(root);
    }

    /**
     * Step the system without noise:
     *   x_{k+1} = A * x_k + B * u_k
     *   y_k     = C * x_k + D * u_k
     */
    Eigen::VectorXd step(const Eigen::VectorXd &input)
    {
        // Dimension checks
        if (input.size() != B_.cols()) {
            throw std::runtime_error("Input dimension mismatch for LTI model");
        }

        // Compute next state
        Eigen::VectorXd next_state = A_ * state_ + B_ * input;

        // Compute output
        Eigen::VectorXd output = C_ * state_ + D_ * input;

        // Update internal state
        state_ = next_state;

        return output;
    }

    /**
     * Step the system with additive Gaussian noise:
     */
    Eigen::VectorXd stepWithNoise(const Eigen::VectorXd &input, double stddev)
    {
        // Generate Gaussian noise
        std::normal_distribution<double> dist(0.0, stddev);

        Eigen::VectorXd noise(state_.size());
        for (int i = 0; i < noise.size(); ++i)
        {
            noise(i) = dist(generator_);
        }

        // Compute next state with noise
        Eigen::VectorXd next_state = A_ * state_ + B_ * input + noise;

        // Compute output
        Eigen::VectorXd output = C_ * state_ + D_ * input;

        // Update internal state
        state_ = next_state;

        return output;
    }

        /**
     * Get the current state vector.
     */
    Eigen::VectorXd getState() const
    {
        return state_;
    }

    /**
     * Get the input size.
     */
    int getInputSize() const
    {
        return B_.cols();
    }


private:
    Eigen::MatrixXd A_, B_, C_, D_;
    Eigen::VectorXd state_;
    std::mt19937 generator_;

    Eigen::MatrixXd parse2DMatrix(const YAML::Node &node, const std::string &key)
    {
        if (!node) {
            throw std::runtime_error("Missing key: " + key);
        }

        std::vector<std::vector<double>> values = node.as<std::vector<std::vector<double>>>();
        int rows = values.size();
        int cols = values[0].size();

        Eigen::MatrixXd mat(rows, cols);
        for (int i = 0; i < rows; ++i)
        {
            if (values[i].size() != cols)
            {
                throw std::runtime_error("Inconsistent row sizes in matrix " + key);
            }
            for (int j = 0; j < cols; ++j)
            {
                mat(i, j) = values[i][j];
            }
        }
        return mat;
    }
};
