/*
 * circular_buffer.h
 *
 *  Created on: Aug 3, 2020
 *      Author: marvin
 *
 *  threadsafe
 */


#pragma once
#include <array>
#include <mutex>
#include <stdexcept>
#include <vector>
#include <utility>
#include <iostream>
#include <cmath>

#include <algorithm>
#include <numeric>
#include <fftw3.h>
#include <complex> 

// terminal comand export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins

namespace tools{


    // Structure to hold FFT result
struct FFTResult {
    std::vector<double> frequencies;  // Frequency bins
    std::vector<double> magnitudes;   // Magnitude spectrum
    double total_power;               // Total power of the signal
};


template <typename T, unsigned int size>
class CircularBuffer
{
    mutable std::mutex mutex;
    unsigned int index = 0; // index of the last add
    std::array<T, size> buffer;
public:
    CircularBuffer(T value = T{}) { buffer.fill(value); }

    void add(T item)
    {
        std::lock_guard<std::mutex> lock(mutex);
        index = (index - 1 + size) % size;
        buffer[index] = std::move(item);
    }


    // returns the last n values in reverse order of arrival.
    // meaning the newest value first
    std::vector<T> get_snapshot(unsigned int n = size) const
    {
        if (n > size)
            throw std::runtime_error("Request exceeds buffer capacity.");

        std::vector<T> ret;
        ret.reserve(n);

        std::lock_guard<std::mutex> lock(mutex);
        for (unsigned int i = 0; i < n; i++)
            ret.push_back(buffer[(index + i + size) % size]);

        return ret;
    }

	// returns the last n values in reverse order of arrival.
	// meaning the newest value first

    // if stride > 1, it skips values in between.
	std::vector<T> get_snapshot_resampled(unsigned int n = size, unsigned int stride = 1) const
	{
		if (n > size)
		  throw std::runtime_error("Request exceeds buffer capacity.");

		if (stride <= 0)
		  throw std::runtime_error("Stride must be greater than zero.");

		std::vector<T> ret;
		ret.reserve(n);

		std::lock_guard<std::mutex> lock(mutex);
		for (unsigned int i = 0, count = 0; count < n && i < size; i += stride, count++)
		  ret.push_back(buffer[(index + i + size) % size]);

		return ret;
	}

    // Function to get the last n values, replacing outliers with the closest non-outlier values
    std::vector<T> get_snapshot_with_replaced_outliers(unsigned int n = size, double outlier_threshold = 1.0) const
    {
        if (n > size)
            throw std::runtime_error("Request exceeds buffer capacity.");

        auto snapshot = get_snapshot(n); // Get the last n values
        T mean = compute_mean(n);
        T variance = compute_variance(n);
        T stdev = std::sqrt(variance); // Standard deviation

        // Temporary vector to store non-outlier values for replacement purposes
        std::vector<T> replacements;

        // First pass: identify non-outliers and store them for potential replacement use
        for (const auto& value : snapshot) {
            if (std::abs(value - mean) <= (stdev * outlier_threshold)) {
                replacements.push_back(value);
            }
        }

        // If all values are outliers or no replacements available, just return the mean or any non-outlier value
        if (replacements.empty()) {
            replacements.push_back(mean); // Fallback to mean if no non-outliers found
            std::cout << "fallback to mean in buffer : " << mean << std::endl;  
        }

        // Second pass: replace outliers using the closest non-outlier value available
        std::vector<T> result;
        result.reserve(n);
        for (const auto& value : snapshot) {
            if (std::fabs(value - mean) > (stdev * outlier_threshold)) {
                //std::cout << "outlier detected : " << value << " / std x threshold " << stdev * outlier_threshold << std::endl;
                // Replace outlier with the last known non-outlier, if available
                result.push_back(replacements.back());
            } else {
                result.push_back(value);
                // Update the last known non-outlier value
                if (!replacements.empty()) {
                    replacements.push_back(value);
                }
                //std::cout << "no outlier: " << value << " std x threshold " << stdev * outlier_threshold << std::endl;

            }
        }

        return result;
    }

    void clear()
    {
    	//T item;// zero if initializer of T is zero
    	std::lock_guard<std::mutex> lock(mutex);
    	index = 0;
    	//buffer[size] = {};//todo does this work?
    	std::fill(std::begin(buffer), std::end(buffer), T{}); // set all to zero
    }

    // Function to compute the mean of the buffer
    T compute_mean() const
    {
        std::lock_guard<std::mutex> lock(mutex);
        T sum = std::accumulate(buffer.begin(), buffer.end(), T(0));
        return sum / size;
    }

    // Function to compute the variance of the buffer
    T compute_variance() const // this is Sample Variance!!
    {
        T mean = compute_mean();
        std::vector<T> diff(size);

        std::lock_guard<std::mutex> lock(mutex);
        std::transform(buffer.begin(), buffer.end(), diff.begin(), [mean](T x) { return x - mean; });
        T sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), T(0));

        return sq_sum / (size - 1); // Sample variance
    }
    // Function to compute the mean of the last N elements in the buffer
    T compute_mean(unsigned int N) const
    {
        if (N > size)
            throw std::runtime_error("Requested number of elements exceeds buffer capacity.");

        auto last_elements = get_snapshot(N);
        T sum = std::accumulate(last_elements.begin(), last_elements.end(), T(0));
        return sum / N;
    }

    // Function to compute the variance of the last N elements in the buffer
    T compute_variance(unsigned int N) const //Sample Variance!!
    {
        if (N > size)
            throw std::runtime_error("Requested number of elements exceeds buffer capacity.");

        T mean = compute_mean(N);
        auto last_elements = get_snapshot(N);
        std::vector<T> diff(N);

        std::transform(last_elements.begin(), last_elements.end(), diff.begin(), [mean](T x) { return x - mean; });
        T sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), T(0));

        return sq_sum / (N - 1); // Sample variance
    }

    // Function to compute the RMSE of the last N elements in the buffer
    T compute_rmse(unsigned int N) const
    {
        if (N > size)
            throw std::runtime_error("Requested number of elements exceeds buffer capacity.");

        T mean = compute_mean(N);
        auto last_elements = get_snapshot(N);
        std::vector<T> squared_diff(N);

        // Calculate squared difference from the mean for each element
        std::transform(last_elements.begin(), last_elements.end(), squared_diff.begin(),
                    [mean](T x) { return std::pow(x - mean, 2); });

        // Compute mean of squared differences
        T mean_of_squared_diff = std::accumulate(squared_diff.begin(), squared_diff.end(), T(0)) / N;

        // Return the square root of the mean of squared differences (RMSE)
        return std::sqrt(mean_of_squared_diff);
    }

    T compute_integral(unsigned int N, double timestep) const
    {
        if (N > size) throw std::runtime_error("Request exceeds buffer capacity.");

        std::lock_guard<std::mutex> lock(mutex);
        T sum = T{}; // Initialize sum of the last N values

        for (unsigned int i = 0; i < N; ++i) {
            sum += buffer[(index + i + size) % size]; // Summing the last N values in reverse order
        }

        return sum * timestep; // Multiply by the timestep to compute the integral
    }

    // Function to compute the magnitude spectrum and total power
    FFTResult compute_fft(unsigned int N, double sampling_frequency) const
    {
        if (N > size)
            throw std::runtime_error("Requested number of elements exceeds buffer capacity.");

        if (N == 0)
            throw std::runtime_error("Number of elements for FFT must be greater than zero.");

        // Retrieve the last N elements
        auto last_elements = get_snapshot(N);

        // Prepare input array for FFTW
        std::vector<double> in(N);
        for (unsigned int i = 0; i < N; ++i) {
            in[i] = static_cast<double>(last_elements[i]);
        }

        // Allocate output array (complex numbers)
        int N_out = N / 2 + 1; // For real-input FFT, output is N/2 + 1 complex numbers
        std::vector<std::complex<double>> out(N_out);

        // Create plan for real-to-complex FFT
        fftw_plan plan = fftw_plan_dft_r2c_1d(N, in.data(),
                                              reinterpret_cast<fftw_complex*>(out.data()),
                                              FFTW_ESTIMATE);

        if (!plan)
            throw std::runtime_error("FFTW plan creation failed.");

        // Execute the FFT
        fftw_execute(plan);

        // Destroy the plan
        fftw_destroy_plan(plan);

        // Compute the magnitude spectrum
        std::vector<double> magnitudes(N_out);
        for (int i = 0; i < N_out; ++i) {
            magnitudes[i] = std::abs(out[i]) * 2.0 / N; // Multiply by 2/N to get the correct amplitude
        }
        // Note: DC and Nyquist components should not be doubled
        magnitudes[0] /= 2.0;
        if (N % 2 == 0) { // If N is even, Nyquist frequency is included
            magnitudes[N_out - 1] /= 2.0;
        }

        // Compute frequency bins
        double freq_resolution = sampling_frequency / N;
        std::vector<double> frequencies(N_out);
        for (int i = 0; i < N_out; ++i) {
            frequencies[i] = i * freq_resolution;
        }

        // Compute total power (Parseval's theorem)
        double total_power = 0.0;
        for (int i = 0; i < N_out; ++i) {
            total_power += magnitudes[i] * magnitudes[i];
        }

        // Return the FFT result
        return FFTResult{frequencies, magnitudes, total_power};
    }



};


/**
 * @brief A wrapper class that stores a std::vector<double> by “unrolling”
 * each element into a separate CircularBuffer<double, size>.
 *
 * @tparam size Capacity of each internal CircularBuffer<double>.
 */
template <unsigned int size>
class CircularBufferVector
{
public:
    /**
     * @brief Default constructor: no buffers until you call reinit().
     */
    CircularBufferVector() = default;

    /**
     * @brief Construct with a given dimension and initial value.
     */
    explicit CircularBufferVector(size_t dim, double initial_value = 0.0)
    {
        reinit(dim, initial_value);
    }

    /**
     * @brief (Re)initialize the dimension. This clears any existing data
     *        and allocates `dim` internal CircularBuffer<double,size> objects.
     */
    void reinit(size_t dim, double initial_value = 0.0)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        buffers_.clear();
        buffers_.reserve(dim);

        // Create each buffer by unique_ptr
        for (size_t i = 0; i < dim; i++)
        {
            buffers_.push_back(
                std::make_unique<CircularBuffer<double, size>>(initial_value));
        }
    }

    /**
     * @brief Returns how many “channels” (elements per sample) are stored.
     */
    size_t dimension() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffers_.size();
    }

    /**
     * @brief Adds one vector sample, unrolling across the scalar buffers.
     */
    void add(const std::vector<double>& sample)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (sample.size() != buffers_.size()) {
            throw std::runtime_error(
                "Incoming sample dimension does not match buffer dimension");
        }
        // “Unroll” the sample into the multiple buffers
        for (size_t i = 0; i < sample.size(); i++) {
            buffers_[i]->add(sample[i]);
        }
    }

    /**
     * @brief Returns the last N samples, newest first.
     *        Each sample is a std::vector<double> of length dimension().
     */
    std::vector<std::vector<double>> get_snapshot(unsigned int N = size) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (N > size) {
            throw std::runtime_error(
                "Requested number of samples exceeds buffer capacity.");
        }
        if (buffers_.empty()) {
            return {}; // No dimension => return empty
        }

        // For each dimension i, retrieve the last N values
        std::vector<std::vector<double>> snapshots_per_dim(buffers_.size());
        for (size_t i = 0; i < buffers_.size(); i++) {
            snapshots_per_dim[i] = buffers_[i]->get_snapshot(N);
        }

        // Combine them => result[k] is the sample across all dims at index k
        std::vector<std::vector<double>> result;
        result.resize(N, std::vector<double>(buffers_.size(), 0.0));

        for (unsigned int k = 0; k < N; k++) {
            for (size_t dim = 0; dim < buffers_.size(); dim++) {
                result[k][dim] = snapshots_per_dim[dim][k];
            }
        }

        return result;
    }

    /**
     * @brief Compute the elementwise mean of the last N samples in each buffer.
     * @return A vector<double> of dimension() means.
     */
    std::vector<double> compute_mean(unsigned int N) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (N > size) {
            throw std::runtime_error(
                "Requested number of samples exceeds buffer capacity.");
        }
        std::vector<double> means(buffers_.size(), 0.0);
        for (size_t i = 0; i < buffers_.size(); i++) {
            means[i] = buffers_[i]->compute_mean(N);
        }
        return means;
    }

    /**
     * @brief Clears all underlying CircularBuffers.
     */
    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto &cb_ptr : buffers_) {
            cb_ptr->clear();
        }
    }

private:
    mutable std::mutex mutex_;

    // We store each channel as a unique_ptr to avoid copy-construction
    // and to keep the underlying CircularBuffer non-copyable/moveable.
    std::vector<std::unique_ptr<CircularBuffer<double, size>>> buffers_;
};


}