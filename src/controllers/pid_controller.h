#pragma once
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <stdexcept>
#include <sstream>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
using nlohmann::json;
using namespace std;

struct PIDConstants {
    double kp;
    double ki;
    double kd;
    double kdd;
};


class PIDController {
public:
    PIDController(double kp = 0.0, double ki = 0.0, double kd = 0.0, double kdd = 0.0, double integral_limit = 10.0)
        : integral_error_(0.0), kp_(kp), ki_(ki), kd_(kd), kdd_(kdd), integral_limit_(integral_limit){}

    void setGains(double kp, double ki, double kd, double kdd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        kdd_ = kdd;
    }

    void setIntegralLimit(double limit) {
        integral_limit_ = limit;
    }

    void resetIntegral() {
        integral_error_ = 0.0;
    }

    double compute(double reference, double measurement, double derivative = 0.0, double second_derivative = 0.0) {
        
        double error_p = reference - measurement;

        // Update integral
        integral_error_ += error_p;
        integral_error_ = std::clamp(integral_error_, -integral_limit_, integral_limit_);


        double p_term = kp_ * error_p;
        double i_term = ki_ * integral_error_;
        double d_term = kd_ * derivative;
        double dd_term = kdd_ * second_derivative;


        double control_value = p_term + i_term + d_term + dd_term;

        std::cout << "p total : " << p_term << " / i_term " << i_term << " / d_term " << d_term << std::endl;

        // Return negative of control_value if that is desired as per original code
        return control_value;
    }
    void printCurrentPidConstants() {
        cout << "Current PID Constants:" << endl;
        cout << "KP: " << kp_ << endl;
        cout << "KI: " << ki_ << endl;
        cout << "KD: " << kd_ << endl;
        cout << "KDD:" << kdd_ << endl;
    }

    double integral_error_;


private:
    double kp_;
    double ki_;
    double kd_;
    double kdd_;
    double integral_limit_;

};



NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PIDConstants, kp, ki, kd, kdd)

class PIDStateMachine {
private:
    map<string, PIDConstants> pidMap;

    

public:
    // updatable such that external inputs can change them.
    PIDConstants current_pid_constants;
    std::string file_path;
    double integral_limit_; 

    PIDStateMachine(){
        current_pid_constants = {1.0, 0.0, 0.0, 0.0}; // Default if no file loaded later
        integral_limit_ = 20;
    }

    void loadConfigurationFromJsonString(const std::string &jsonContent) {
        json j = json::parse(jsonContent);
        pidMap = j["pid_constants"].get<map<string, PIDConstants>>();
    }

    void loadConfigurationFromJsonPath(const std::string& json_path) {
        file_path = json_path;
        ifstream inFile(file_path);
        if (!inFile.is_open()) {
            throw runtime_error("Could not open file: " + file_path);
        }

        json j;
        inFile >> j;

        // Assuming PIDConstants can be correctly deserialized from JSON
        pidMap = j["pid_constants"].get<map<string, PIDConstants>>();
    }    

    // Function to compare if a value falls within a given range
    bool isInRange(double value, double lower, double upper, char openBrace, char closeBrace) {
        bool inRange = (openBrace == '(' ? (value > lower) : (value >= lower)) &&
                    (closeBrace == ')' ? (value < upper) : (value <= upper));

        // Print the range, lower, and upper values along with the result for debugging
        /**std::cout << "Checking range: " << openBrace << lower << "," << upper << closeBrace;
        std::cout << " for value: " << value;
        std::cout << " [Lower: " << lower << ", Upper: " << upper << "] => " << (inRange ? "In Range" : "Not In Range") << std::endl;
        **/
        return inRange;
    }

    void loadConfigurationFromYamlNode(const YAML::Node &root) 
    {
        // We expect a top-level "pid_controller" key
        YAML::Node pc = root["pid_controller"];
        if (!pc) {
            throw std::runtime_error("YAML must contain 'pid_controller' key at top-level");
        }
        // Now load the sub-key "pid_constants"
        YAML::Node constNode = pc["pid_constants"];
        if (!constNode) {
            throw std::runtime_error("YAML must contain 'pid_constants' under 'pid_controller'");
        }
        pidMap.clear();

        for (auto it = constNode.begin(); it != constNode.end(); ++it) {

            if (!it->second["kp"] || !it->second["ki"] ||
                !it->second["kd"] || !it->second["kdd"]) {
            throw std::runtime_error("Missing 'kp', 'ki', 'kd', or 'kdd' in one of the ranges");
            }
            // The rangeKey might be "[0.0,10.0]" etc.
            std::string rangeKey = it->first.as<std::string>();

            double kp  = it->second["kp"].as<double>();
            double ki  = it->second["ki"].as<double>();
            double kd  = it->second["kd"].as<double>();
            double kdd = it->second["kdd"].as<double>();

            pidMap[rangeKey] = PIDConstants{kp, ki, kd, kdd};
        }
        integral_limit_ = constNode["integral_limit"].as<double>();

        current_pid_constants = pidMap.begin()->second; // Load first constants by default
    }

    void loadConfigurationFromYamlPath(const std::string &yamlPath) 
    {
        file_path = yamlPath;

        std::ifstream inFile(file_path);
        if (!inFile.is_open()) {
            throw std::runtime_error("Could not open YAML file: " + file_path);
        }

        YAML::Node root = YAML::Load(inFile);
        loadConfigurationFromYamlNode(root);
    }
    // Function to find and switch state by value
    PIDConstants findAndSwitchStateByValue(double value) {
        for (const auto& pair : pidMap) {
            // Parse the range string
            std::string range = pair.first; // Use the full key directly
            double lower, upper;
            char openBrace, closeBrace, comma;
            std::stringstream ss(range);
            
            // Handle negative values correctly
            if (range[0] == '-') {
                ss >> openBrace >> lower >> comma >> upper >> closeBrace;
                lower = -lower; // Correct the sign of the lower bound
            } else {
                ss >> openBrace >> lower >> comma >> upper >> closeBrace;
            }

            // Check if the value is in the range
            bool inRange = isInRange(value, lower, upper, openBrace, closeBrace);

            if (inRange) {
                current_pid_constants = pair.second;
                //std::cout << "Switched to state: " << pair.first << std::endl;
                return current_pid_constants;
            }
        }
        std::cout << "No matching state found for value: " << value << std::endl;
        return current_pid_constants; // Return last values
    }

    // Function to switch the PID state by a given force reference range
    void switchState(const string& forceRefRange) {
        auto it = pidMap.find(forceRefRange);
        if (it != pidMap.end()) {
            current_pid_constants = it->second;
            cout << "Switched to state: " << forceRefRange << endl;
        } else {
            cout << "State not found: " << forceRefRange << endl;
        }
    }

    // Function to get the current PID constants
    PIDConstants getCurrentPidConstants() {
        return current_pid_constants;
    }

    // Function to print the current PID constants
    void printCurrentPidConstants() {
        cout << "Current PID Constants:" << endl;
        cout << "KP: " << current_pid_constants.kp << endl;
        cout << "KI: " << current_pid_constants.ki << endl;
        cout << "KD: " << current_pid_constants.kd << endl;
        cout << "KDD: " << current_pid_constants.kdd << endl;
    }

    // Function to print the entire PID state machine
    void printStateMachine() {
        cout << "PID State Machine:" << endl;
        for (const auto& pair : pidMap) {
            cout << "Force Reference Range: " << pair.first << endl;
            cout << "\tKP: " << pair.second.kp << endl;
            cout << "\tKI: " << pair.second.ki << endl;
            cout << "\tKD: " << pair.second.kd << endl;
            cout << "\tKDD: " << pair.second.kdd << endl;
        }
    }
};


class PIDControlSupervisor {
private:
    PIDController controller;
    PIDStateMachine stateMachine;

public:
    PIDControlSupervisor() 
      : controller(), stateMachine() 
    { }

    void loadConfigurationFromJsonPath(const std::string& jsonPath) {
        stateMachine.loadConfigurationFromJsonPath(jsonPath);
    }
    void loadConfigurationFromJsonString(const std::string &jsonContent) {
        stateMachine.loadConfigurationFromJsonString(jsonContent);
    }
    
    void loadConfigurationFromYamlNode(const YAML::Node &yamlData) {
        stateMachine.loadConfigurationFromYamlNode(yamlData);
        controller.setIntegralLimit(stateMachine.integral_limit_);
        std::cout << "set integral limit at: " << stateMachine.integral_limit_ << std::endl;
    }
    void loadConfigurationFromYamlPath(const std::string& yamlPath) {
        stateMachine.loadConfigurationFromYamlPath(yamlPath);
        controller.setIntegralLimit(stateMachine.integral_limit_);
        std::cout << "set integral limit at: " << stateMachine.integral_limit_ << std::endl;
    }

    PIDConstants updateGainsBasedOnTarget(double refrence) {
        PIDConstants constants = stateMachine.findAndSwitchStateByValue(refrence);
        controller.setGains(constants.kp, constants.ki, constants.kd, constants.kdd);
        return constants;
    }
    
    void setIntegralLimit(double integral_limit){
        controller.setIntegralLimit(integral_limit);
    }

    void updateGainsByOverride(double kp, double ki, double kd, double kdd) {
        controller.setGains(kp, ki, kd, kdd);
    }

    double computeControl(double reference, double measurement, double derivative = 0.0, double second_derivative = 0.0) {
        return controller.compute(reference, measurement, derivative, second_derivative);
    }
    double getIntegrator(){
        return controller.integral_error_;
    }

    void resetIntegral() {
        controller.resetIntegral();
    }

    void printCurrentPidConstants() {
        // this could be the same as the statemachine with override not.
        controller.printCurrentPidConstants();
    }

    void printStateMachine() {
        stateMachine.printStateMachine();
    }
    

};

