#include <gtest/gtest.h>
#include "data_handling/circular_buffer.h"
#include <thread>
#include <future>
#include <chrono>
#include <vector>



using namespace tools;

class CircularBufferTest : public ::testing::Test {
protected:
    CircularBuffer<double, 10> buffer;

    void SetUp() override {
        // Fill buffer with initial values for testing
        for (int i = 0; i < 5; ++i) {
            buffer.add(i);
        }
        for (int i = 0; i < 5; ++i) {
            buffer.add(i);
        }
    }
};

TEST_F(CircularBufferTest, AddAndSnapshot) {
    buffer.add(5);
    auto snapshot = buffer.get_snapshot(3);
    EXPECT_EQ(snapshot.size(), 3u);
    EXPECT_EQ(snapshot[0], 5);
    EXPECT_EQ(snapshot[1], 4);
    EXPECT_EQ(snapshot[2], 3);
}

TEST_F(CircularBufferTest, Clear) {
    buffer.clear();
    auto snapshot = buffer.get_snapshot();
    for (const auto& value : snapshot) {
        EXPECT_EQ(value, 0);
    }
}

TEST_F(CircularBufferTest, ComputeMean) {
    double mean = buffer.compute_mean();
    EXPECT_DOUBLE_EQ(mean, 2.0); // Mean of 0, 1, 2, 3, 4

}

TEST_F(CircularBufferTest, ComputeIntegral) {
    // Assuming a timestep of 1 for simplicity; adjust as needed
    double timestep = 1.0;
    unsigned int N = 5; // Number of elements to integrate over

    // The buffer was filled twice with values 0 to 4,
    // so the last 5 elements are [4, 3, 2, 1, 0].
    // The sum of these elements is 10.
    // With a timestep of 1, the expected integral is the sum itself, 10.
    double expectedIntegral = 10.0;

    // Compute the integral of the last N values
    double computedIntegral = buffer.compute_integral(N, timestep);

    EXPECT_EQ(computedIntegral, expectedIntegral);
}

TEST_F(CircularBufferTest, ComputeSampleVariance) {
    auto snapshot = buffer.get_snapshot();
    double variance = buffer.compute_variance();
    EXPECT_NEAR(variance, 2.22222222, 0.01); // Sample variance of 0, 1, 2, 3, 4, 0, 1, 2, 3, 4
    variance = buffer.compute_variance(5);
    EXPECT_NEAR(variance, 2.5, 0.01); // Sample variance of 0, 1, 2, 3, 4
}

TEST_F(CircularBufferTest, ComputeVarianceAndMeanWithRef) {

    buffer.add(4);
    buffer.add(4);

    auto mean = buffer.compute_mean(3);
    EXPECT_NEAR(mean, 4.0,0.001); // Mean of 4,4,4 with INT


    double variance = buffer.compute_variance(3);
    EXPECT_NEAR(variance, 0.0, 0.001); // Sample variance of 4,4,4
}




TEST_F(CircularBufferTest, ComputeRMSE) {
    // Adding values to ensure we have a known variance
    buffer.clear();
    for (int i = 1; i <= 5; ++i) {
        buffer.add(i);
    }
    double rmse = buffer.compute_rmse(5);
    //double expectedMean = 3.0; // Mean of 1, 2, 3, 4, 5
    double expectedRMSE = std::sqrt(((1-3)*(1-3) + (2-3)*(2-3) + (3-3)*(3-3) + (4-3)*(4-3) + (5-3)*(5-3)) / 5);
    EXPECT_NEAR(rmse, expectedRMSE, 0.001);
}

TEST_F(CircularBufferTest, SnapshotWithReplacedOutliers) {
    buffer.clear();
    buffer.add(1.0);
    buffer.add(1.0);
    buffer.add(1.0);
    buffer.add(1.0);
    buffer.add(1.0);
    buffer.add(1.0);
    buffer.add(2.0);
    buffer.add(3.0);
    buffer.add(100.0); // Outlier
    buffer.add(4.0);
    buffer.add(5.0);

    //std::cout << "mean " << buffer.compute_mean() << " var " << buffer.compute_variance() << std::endl;
    auto snapshot = buffer.get_snapshot_with_replaced_outliers(9, 1); // Using a threshold to identify the outlier

    // Expect the outlier (100) to be replaced. Since this test depends on your outlier replacement strategy,
    // you may need to adjust the expected values accordingly.

    EXPECT_EQ(snapshot[0], 5.0);
    EXPECT_EQ(snapshot[1], 4.0);
    EXPECT_NE(snapshot[2], 100.0); // Expect the outlier to be replaced
    EXPECT_TRUE(snapshot[2] == 4.0 ); // Assuming replacement with the closest non-outlier
    EXPECT_EQ(snapshot[3], 3.0);
    EXPECT_EQ(snapshot[4], 2.0);
}

TEST_F(CircularBufferTest, SnapshotWithReplacedOutliersMeanReplacement) {
    buffer.clear();
    buffer.add(1.0);
    buffer.add(1.0);
    buffer.add(1.0);
    buffer.add(10.0); // Outlier
    buffer.add(1.0);
    buffer.add(1.0);

    std::cout << "mean " << buffer.compute_mean() << " var " << buffer.compute_variance() << std::endl;
    auto snapshot = buffer.get_snapshot_with_replaced_outliers(6, 0.5); // Using a threshold to identify the outlier
    
    /**for (const auto& element : snapshot) {
        std::cout << " element " << element;
    }
    std::cout << std::endl;**/
    // Expect the outlier to be replaced. Since this test depends on your outlier replacement strategy,
    // you may need to adjust the expected values accordingly.
    EXPECT_EQ(snapshot[0], 1.0);
    EXPECT_EQ(snapshot[1], 1.0);
    EXPECT_NE(snapshot[2], 10.0); // Expect the outlier to be replaced
    EXPECT_TRUE(snapshot[2] == 1.0 ); // Assuming replacement with the closest non-outlier
    EXPECT_EQ(snapshot[3], 1.0);
    EXPECT_EQ(snapshot[4], 1.0);
}


TEST(CircularBufferAddTest, HandlesTenValues) {
    CircularBuffer<int, 10> buffer;

    // Add values 0 to 9
    for (int i = 0; i < 10; ++i) {
        buffer.add(i);
    }

    // Retrieve the snapshot
    auto snapshot = buffer.get_snapshot();

    // Expected reverse order
    std::array<int, 10> expected = {9, 8, 7, 6, 5, 4, 3, 2, 1, 0};

    // Verify the order of the values
    for (size_t i = 0; i < snapshot.size(); ++i) {
        EXPECT_EQ(snapshot[i], expected[i]);
    }

    }


/**
 * \brief An extended test fixture for more thorough tests
 */
class CircularBufferExtraTest : public ::testing::Test {
protected:
    CircularBuffer<double, 10> buffer;

    void SetUp() override {
        // Partially fill buffer
        for (int i = 0; i < 5; ++i) {
            buffer.add(static_cast<double>(i));
        }
    }
};

TEST_F(CircularBufferExtraTest, SnapshotResampledFull) {
    // Fill up to 10 items in the buffer
    for (int i = 5; i < 10; ++i) {
        buffer.add(static_cast<double>(i));
    }

    // Try a stride of 2
    auto snapshotStride2 = buffer.get_snapshot_resampled(10, 2);

    // Buffer was filled [0,1,2,3,4,5,6,7,8,9], but remember the newest is at index 0 in the snapshot
    // get_snapshot_resampled with stride=2 means we skip every other item
    // The newest item is 9, then skipping 8, next is 7, skipping 6, next is 5, skipping 4, etc.
    // So we expect [9, 7, 5, 3, 1]
    // But note we asked for 10 items; with stride=2, we won't get 10 unique samples if the buffer
    // can’t provide that many stepping by 2. We’ll get only as many as fit, up to n=10, but we run
    // out of the ring first.

    EXPECT_TRUE(snapshotStride2.size() <= 10u);
    // The first few elements should be 9, 7, 5, 3, 1 (descending by 2)
    std::vector<double> expectedResampled = {9, 7, 5, 3, 1};
    for (size_t i = 0; i < expectedResampled.size(); i++) {
        EXPECT_DOUBLE_EQ(snapshotStride2[i], expectedResampled[i]);
    }
}

TEST_F(CircularBufferExtraTest, SnapshotResampledThrowsOnInvalidStride) {
    // Stride of 0 should throw
    EXPECT_THROW(buffer.get_snapshot_resampled(5, 0), std::runtime_error);
    // Stride of negative is not allowed either (if you prefer to enforce >0).
    // This might be a design choice, so adapt accordingly.
}

TEST_F(CircularBufferExtraTest, MultiThreadedAddAndSnapshot) {
    // We'll spawn several threads that add values concurrently.
    // We just want to check that it doesn't crash and the final snapshot is valid.

    auto writer = [&](int startVal) {
        for (int i = 0; i < 100; i++) {
            buffer.add(startVal + i);
        }
    };

    // Launch multiple threads
    std::thread t1(writer, 0);
    std::thread t2(writer, 1000);
    std::thread t3(writer, 2000);

    t1.join();
    t2.join();
    t3.join();

    // Just check that we can retrieve a snapshot without data corruption
    auto snapshot = buffer.get_snapshot(10);
    // We don't assert the exact content because we wrote many times,
    // but let's check it has size 10 and no exception thrown.
    EXPECT_EQ(snapshot.size(), 10u);
}

TEST_F(CircularBufferExtraTest, ComputeFFTWithZeroElementsThrows) {
    // Request an FFT with 0 elements
    EXPECT_THROW(buffer.compute_fft(0, 100.0), std::runtime_error);
}

TEST_F(CircularBufferExtraTest, ComputeFFTWithMoreThanSizeThrows) {
    // Request more elements than size
    EXPECT_THROW(buffer.compute_fft(11, 100.0), std::runtime_error);
}

TEST_F(CircularBufferExtraTest, ComputeFFTBasicCheck) {
    // Fill the buffer with a simple ramp from 0..9
    buffer.clear();
    for (int i = 0; i < 10; ++i) {
        buffer.add(static_cast<double>(i));
    }

    // For a small test, we just verify it doesn’t crash and the result is sized appropriately
    auto fftRes = buffer.compute_fft(10, /* sampling_freq= */ 100.0);

    EXPECT_EQ(fftRes.frequencies.size(), 10/2 + 1); // 5+1=6
    EXPECT_EQ(fftRes.magnitudes.size(), 10/2 + 1);  // 6
    // We won't check the exact numeric result of the FFT here, just that it's computed.
    EXPECT_GE(fftRes.total_power, 0.0);
}



TEST(CircularBufferVectorTest, DefaultConstructor) {
    CircularBufferVector<10> vecBuffer;
    EXPECT_EQ(vecBuffer.dimension(), 0u);  // No buffers initially
}

TEST(CircularBufferVectorTest, Reinit) {
    CircularBufferVector<10> vecBuffer;
    vecBuffer.reinit(3, 42.0); // 3 “channels”, each pre-filled with 42.0
    EXPECT_EQ(vecBuffer.dimension(), 3u);

    // Immediately get snapshot to confirm it’s all 42.0
    auto snapshot = vecBuffer.get_snapshot(5);
    // Should have 5 “samples” (since N=5) if it was indeed pre-filled,
    // each sample is a vector<double> of size=3
    EXPECT_EQ(snapshot.size(), 5u);
    for (auto &samp : snapshot) {
        EXPECT_EQ(samp.size(), 3u);
        for (auto &val : samp) {
            EXPECT_DOUBLE_EQ(val, 42.0);
        }
    }
}

TEST(CircularBufferVectorTest, AddAndSnapshot) {
    CircularBufferVector<10> vecBuffer(3, 0.0); // 3 channels, filled with 0.0
    // Add some data
    vecBuffer.add({1.0, 2.0, 3.0});
    vecBuffer.add({4.0, 5.0, 6.0});
    vecBuffer.add({7.0, 8.0, 9.0});

    // Get the last 3 samples
    auto snapshot = vecBuffer.get_snapshot(3);
    ASSERT_EQ(snapshot.size(), 3u);

    // The newest sample is snapshot[0], next is snapshot[1], etc.
    EXPECT_EQ(snapshot[0][0], 7.0);
    EXPECT_EQ(snapshot[0][1], 8.0);
    EXPECT_EQ(snapshot[0][2], 9.0);

    EXPECT_EQ(snapshot[1][0], 4.0);
    EXPECT_EQ(snapshot[1][1], 5.0);
    EXPECT_EQ(snapshot[1][2], 6.0);

    EXPECT_EQ(snapshot[2][0], 1.0);
    EXPECT_EQ(snapshot[2][1], 2.0);
    EXPECT_EQ(snapshot[2][2], 3.0);
}

TEST(CircularBufferVectorTest, AddThrowsIfDimensionMismatch) {
    CircularBufferVector<10> vecBuffer(3);
    // Attempt to add a sample of dimension=2 where dimension=3 is expected:
    EXPECT_THROW(vecBuffer.add({1.0, 2.0}), std::runtime_error);
}

TEST(CircularBufferVectorTest, ComputeMean) {
    CircularBufferVector<10> vecBuffer(3, 0.0); // dimension=3
    // Fill each channel with some values
    vecBuffer.add({1.0, 2.0, 3.0});
    vecBuffer.add({1.0, 2.0, 3.0});
    vecBuffer.add({2.0, 4.0, 6.0});

    // Compute mean over last 3 samples
    auto means = vecBuffer.compute_mean(3);
    ASSERT_EQ(means.size(), 3u);
    // The last 3 samples in each channel are:
    //   ch0: 2,1,1 => actually in reverse: newest is 2. But the order doesn't matter for sum.
    //   ch1: 4,2,2
    //   ch2: 6,3,3
    // Summation in ch0: (2+1+1)=4 => mean=4/3=1.333..
    // Summation in ch1: (4+2+2)=8 => mean=8/3=2.666..
    // Summation in ch2: (6+3+3)=12 => mean=12/3=4.0
    EXPECT_NEAR(means[0], 1.3333, 1e-3);
    EXPECT_NEAR(means[1], 2.6666, 1e-3);
    EXPECT_NEAR(means[2], 4.0, 1e-3);
}

TEST(CircularBufferVectorTest, Clear) {
    CircularBufferVector<10> vecBuffer(3, 2.0);
    vecBuffer.add({1.0, 2.0, 3.0});
    vecBuffer.clear();
    // The next snapshot of size=5 => all zero if each buffer was cleared to T{}=0.0
    auto snapshot = vecBuffer.get_snapshot(5);
    for (auto &sample : snapshot) {
        for (auto &val : sample) {
            EXPECT_DOUBLE_EQ(val, 0.0);
        }
    }
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
