// data_importer_and_resampler.h

#ifndef DATA_IMPORTER_AND_RESAMPLER_H
#define DATA_IMPORTER_AND_RESAMPLER_H

#include <string>
#include <vector>
#include <Eigen/Dense>

class dataImporterAndResampler {
public:
    // Constructor with parameters to override defaults
    dataImporterAndResampler(
        double delta_T,
        bool import_all_data,
        size_t start_sample_index,
        size_t number_of_samples,
        const std::string& path_to_csv_file,
        const std::vector<int>& layer_numbers // New parameter
    );

    // Method to get all data
    std::vector<Eigen::VectorXd> getData() const;

    // Method to get data for specified columns
    std::vector<Eigen::VectorXd> getData(const std::vector<std::string>& column_names) const;

private:
    // Method to import and resample data
    void importAndResampleData();

    // Helper method to get column index by name
    size_t getColumnIndex(const std::string& column_name) const;

    // Member variables
    double delta_T_;
    bool import_all_data_;
    size_t start_sample_index_;
    size_t number_of_samples_;
    std::string path_to_csv_file_;

    double time_scale_;
    int data_start_line_;
    std::string time_column_name_;
    std::string layer_column_name_;       // New member variable

    std::vector<int> layer_numbers_;      // New member variable

    std::vector<std::string> column_names_;
    std::vector<Eigen::VectorXd> data_;
};

#endif // DATA_IMPORTER_AND_RESAMPLER_H
