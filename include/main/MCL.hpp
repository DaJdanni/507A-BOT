#pragma once
#include "vex.h"

// Based off of FilterPy & Behnam's Implementation of an MCL model
// Uses C++ implementations of numpy/scipy [Eigen/Custom Code]

using Eigen::MatrixXd;
using Eigen::VectorXd;

static std::random_device rd;
static std::mt19937 gen(rd());
    
/**
 * @brief Normalize function from scipy
 * 
 * @param x
 * @param mean
 * @param std_dev
 */
double normal_pdf(double mean, double std_dev, double x);

/**
 * @brief Draw samples from a uniform distribution. Samples are uniformly distributed over the half-open interval [low, high) (includes low, but excludes high). In other words, any value within the given interval is equally likely to be drawn by uniform.
 * 
 * @param min Lower boundary of the output interval. All values generated will be greater than or equal to low. The default value is 0.
 * @param max Upper boundary of the output interval. All values generated will be less than or equal to high. The high limit may be included in the returned array of floats due to floating-point rounding in the equation low + (high-low) * random_sample(). The default value is 1.0.
 *
 * @return Returns a uniform real distribution containing the random numbers in your range
 */
std::uniform_real_distribution<>  uniform(double min, double max);

/**
 * @brief Create uniform particles for possible x, y, theta positions
 * 
 * @param xRanges [min, max] for possible X positions
 * @param yRanges [min, max] for possible Y positions
 * @param thetaRanges [min, max] for possible Theta Positions
 * 
 * @return MatrixXd of particles
 */
MatrixXd create_uniform_particles(std::vector<double> xRanges, std::vector<double> yRanges, std::vector<double> thetaRanges, int n);

/**
 * @brief Systematically resample weights by creating subdivisions and choosing positions with a consistent random offset
 * 
 * @param weights A VectorXd of weights
 * 
 * @return VectorXd of resampled weights
 */
VectorXd systematic_resample(VectorXd weights);

/**
 * @brief Calculate the weighted mean
 * 
 * @param positions A VectorXd of positions
 * @param weights A VectorXd of weights
 * 
 * @return double of weighted mean
 */
double calculate_weighted_mean(VectorXd positions, VectorXd weights);

/**
 * @brief Calculate the weighted variance
 * 
 * @param positions A VectorXd of positions
 * @param weights A VectorXd of weights
 * @param mean A VectorXd of the mean
 * 
 * @return double of weighted variance
 */
double calculate_weighted_variance(VectorXd positions, VectorXd weights, double mean);

/**
 * @brief Calculate the effective sample size for your MCL
 * 
 * @param weights A VectorXd that contains the weights of your particles
 * 
 * @return A number calculated by using 1 over the sum of the square of your weights
 */
double neff(VectorXd weights);

bool ccw(VectorXd a, VectorXd b, VectorXd c);
bool pointDoesIntersect(VectorXd segment1Start, VectorXd segment1End, VectorXd segment2Start, VectorXd segment2End);
VectorXd pointAtIntersect(VectorXd segment1Start, VectorXd segment1End, VectorXd segment2Start, VectorXd segment2End);
double distanceAtIntersect(VectorXd position, VectorXd position2);
double distanceToSegment(VectorXd point, VectorXd segmentStart, VectorXd segmentEnd);
bool pointIntersects(VectorXd point, VectorXd segmentStart, VectorXd segmentEnd);

class MCLOdometry {
    private:

    MatrixXd particles;
    VectorXd weights;
    MatrixXd landmarks;
    MatrixXd offsets;
    double mNoiseCovariance;
    double mOdomNoiseCovariance;
    double fVelocityStd;
    double aVelocityStd;
    double totalWeight;
    double maxWeight;

    public:

    /**
     * @brief Initialize the Monte Carlo Localization Odometry
     * 
     * @param initParticles The initial particles generated
     * @param initWeights The initial weights generated
     * @param landmarks The landmarks that the MCL will detect [ [xStart, yStart, xEnd, yEnd] ]
     * @param offsets The sensor offsets relative to the robot [ [sensor1XOffset, sensor1YOffset, thetaOffset], [sensor2XOffset, sensor2YOffset, thetaOffset] ]
     * @param mNoiseCovariance The scalar that represents the uncertanity in sensor measurements
     * @param fVelocityStd The scalar that represents the uncertanity in your forward velocity measurements
     * @param aVelocityStd The scalar that represents the uncertanity in your angular velocity measurements
     */
    MCLOdometry(MatrixXd initParticles, VectorXd initWeights, MatrixXd landmarks, MatrixXd offsets, double mNoiseCovariance, double mOdomNoiseCovariance, double fVelocityStd, double aVelocityStd) {
        this->particles = initParticles;
        this->weights = initWeights;
        this->landmarks = landmarks;
        this->offsets = offsets;
        this->mNoiseCovariance = mNoiseCovariance;
        this->mOdomNoiseCovariance = mOdomNoiseCovariance;
        this->fVelocityStd = fVelocityStd;
        this->aVelocityStd = aVelocityStd;

        std::cout << "MCL Odometry has been initialized." << std::endl;
      //  std::cout << "Particles: " << this->particles << std::endl;
    }

    /**
     * @brief Predict the new set of particles
     * 
     * @param controlInput The forward and angular velocity that control how the robot moves
     * @param dt The time between each loop in ms
     */
    void predict(std::vector<double> controlInput, float dt);

    /**
     * @brief Update the weights of the particles
     * 
     * @param measurements The measurements from the sensors
     */
    void update(std::vector<double> measurements); 

    /**
     * @brief Resample the particles systematically
     */
    void resample();

    /**
     * @brief Get back the estimated positions and their variances
     */
    std::vector<std::vector<double>> estimate();

    /**
     * @brief Returns the particles of the filter
     */
    MatrixXd getParticles();
};