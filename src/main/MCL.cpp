#include "vex.h"

constexpr double sanitizeAngle(double angle, bool inRadians = true) {
    if (inRadians == true) return fmod(fmod(angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    else return fmod(fmod(angle, 360) + 360, 360);
}

/**
 * @brief Normalize function from scipy
 *
 * @param mean
 * @param std_dev
 * @param x
 */
double normal_pdf(double mean, double std_dev, double x) {
    double exponent = -0.5 * pow((x - mean) / std_dev, 2);
    return (1.f / (std_dev * sqrt(2 * M_PI))) * exp(exponent);
}

/**
 * @brief Draw samples from a uniform distribution. Samples are uniformly distributed over the half-open interval [low, high) (includes low, but excludes high). In other words, any value within the given interval is equally likely to be drawn by uniform.
 *
 * @param min Lower boundary of the output interval. All values generated will be greater than or equal to low. The default value is 0.
 * @param max Upper boundary of the output interval. All values generated will be less than or equal to high. The high limit may be included in the returned array of floats due to floating-point rounding in the equation low + (high-low) * random_sample(). The default value is 1.0.
 */
std::uniform_real_distribution<> uniform(double min, double max) {
    std::uniform_real_distribution<> distributuion(min, max);

    return distributuion;
}

/**
 * @brief Create uniform particles
 *
 * @param xRanges [min, max] for possible X positions
 * @param yRanges [min, max] for possible Y positions
 * @param thetaRanges [min, max] for possible Theta Positions
 * @param n The amount of particles you wish to create
 *
 * @return MatrixXd of particles
 */
MatrixXd create_uniform_particles(std::vector<double> xRanges, std::vector<double> yRanges, std::vector<double> thetaRanges, int n) {
    std::random_device rdS;
    std::mt19937 genS(rdS());
    MatrixXd particles = MatrixXd(n, 3);

    std::uniform_real_distribution<> xDistributions = uniform(xRanges[0], xRanges[1]);
    std::uniform_real_distribution<> yDistributions = uniform(yRanges[0], yRanges[1]);
    std::uniform_real_distribution<> thetaDistributions = uniform(thetaRanges[0], thetaRanges[1]);

    for (int i = 0; i < n; ++i) {
        particles(i, 0) = xDistributions(genS);
        particles(i, 1) = yDistributions(genS);
        particles(i, 2) = thetaDistributions(genS);
    }

    return particles;
}

/**
 * @brief Systematically resample weights by creating subdivisions and choosing positions with a consistent random offset
 *
 * @param weights A VectorXd of weights
 *
 * @return VectorXd of resampled particles
 */
VectorXd systematic_resample(VectorXd weights) {
    int N = weights.size();

    VectorXd positions(N);
    // Initialize the positions vector
    for (int i = 0; i < N; ++i) {
        positions(i) = (uniform(0, 0.9999999999999999)(gen) + i) / N;
    }

    Eigen::VectorXd indexes(N);
    Eigen::VectorXd cumulative_sum(N);

    // Manually compute cumulative sum
    cumulative_sum(0) = weights(0);
    for (int i = 1; i < N; ++i) {
        cumulative_sum(i) = cumulative_sum(i - 1) + weights(i);
    }

    int i = 0, j = 0;
    while (i < N) {
        if (j >= N) {
            break;  // Prevent `j` from exceeding bounds
        }

        if (positions(i) < cumulative_sum(j)) {
            indexes(i) = j;
            i++;
        } else {
            j++;
        }
    }

    return indexes;
}


/**
 * @brief Calculate the weighted mean
 *
 * @param positions A VectorXd of positions
 * @param weights A VectorXd of weights
 *
 * @return double of weighted mean
 */
double calculate_weighted_mean(VectorXd positions, VectorXd weights) {
    double mean = (positions.array() * weights.array()).sum() / weights.sum();
    return mean;
}


/**
 * @brief Calculate the weighted variance
 *
 * @param positions A VectorXd of positions
 * @param weights A VectorXd of weights
 * @param mean A VectorXd of the mean
 *
 * @return double of weighted variance
 */
double calculate_weighted_variance(VectorXd positions, VectorXd weights, double mean) {
    VectorXd diff = positions.array() - mean;
    double var = (diff.array().square() * weights.array()).sum() / weights.sum();
    return var;
}
/**
 * @brief Calculate the effective sample size for your MCL
 *
 * @param weights A VectorXd that contains the weights of your particles
 *
 * @return A number calculated by using 1 over the sum of the square of your weights
 */
double neff(VectorXd weights) {
    return 1.f / weights.squaredNorm();
}

/**
 * @brief Predict the new set of particles
 *
 * @param controlInput The forward and angular velocity that control how the robot moves
 * @param dt The time between each loop in ms
 */
std::vector<double> previousControlInput = {0, 0, 0};
void MCLOdometry::predict(std::vector<double> controlInput, float dt) {
    // double currentRightPosition = controlInput[0];
    // double currentBackPosition = controlInput[1];
    // double currentIMU = controlInput[2];
    // double previousRightPosition = previousControlInput[0];
    // double previousBackPosition = previousControlInput[1];
    // double previousIMU = previousControlInput[2];

    std::normal_distribution<double> xNoise(0, this->fVelocityStd);
    std::normal_distribution<double> yNoise(0, this->fVelocityStd);
    std::normal_distribution<double> thetaNoise(0, this->aVelocityStd);

    double v = controlInput[0];
    double w = controlInput[1];

    std::normal_distribution<double> vNoise(0, this->fVelocityStd);
    std::normal_distribution<double> wNoise(0, this->aVelocityStd);

//     //std::cout << "hi" << std::endl;
 
//     // std::cout << "Vel: " << v << std::endl;

//     //std::cout << "Rows: " << n << std::endl;

// //    std::cout << "particles before: " << this->particles << std::endl;

    for (int i = 0; i < this->particles.rows(); ++i) {
    //     double x = this->particles(i, 0);
    //     double y = this->particles(i, 1);
    //     double theta = this->particles(i, 2);

    //     double noisyX = currentRightPosition + xNoise(gen);
    //     double noisyY = currentBackPosition + yNoise(gen);
    //     double noisyIMU = currentIMU + thetaNoise(gen);

    //     // // Get the changes:
    //     float rightDelta = currentRightPosition - previousRightPosition;
    //     float backDelta = currentBackPosition - previousBackPosition;
    //     float deltaIMU = currentIMU - previousIMU;
        
    //     // Calculate the average orientation:
    //     float averageTheta = previousIMU + deltaIMU / 2;

    //     // Calculate the local position of the robot:
    //     float localXPosition;
    //     float localYPosition;

    //    // std::cout << previousIMU << std::endl;

    //     if (deltaIMU == 0) { // prevent divide by 0
    //         localXPosition = backDelta;
    //         localYPosition = rightDelta;
    //     } else {
    //         localXPosition = 2 * sinf(deltaIMU / 2) * ((backDelta / deltaIMU) + -2.5);
    //         localYPosition = 2 * sinf(deltaIMU / 2) * ((rightDelta / deltaIMU) + -0.5);
    //     }
        
    //     // Calculate the global position of the robot:
    //     // currentCoordinates.x += localYPosition * sinf(averageTheta) - (localXPosition * cos(averageTheta));
    //     // currentCoordinates.y += localYPosition * cosf(averageTheta) + (localXPosition * sinf(averageTheta));
    //     x += localYPosition * sinf(averageTheta);
    //     y += localYPosition * cosf(averageTheta);
    //     x += localXPosition * -cosf(averageTheta);
    //     y += localXPosition * sinf(averageTheta);
    //     theta = currentIMU;
    //     theta = sanitizeAngle(theta);

    //     this->particles(i, 0) = x;
    //     this->particles(i, 1) = y;
    //     this->particles(i, 2) = theta;

        double x = this->particles(i, 0);
        double y = this->particles(i, 1);
        double theta = this->particles(i, 2);

        double noisy_v = v + vNoise(gen);
        double noisy_w = w + wNoise(gen);

        // std::cout << "Index: " << i << std::endl;
        // std::cout << "Noisy_V: " << noisy_v << " | Noisy_W: " << noisy_w << std::endl;

        // std::cout << "Before update: (" << x << ", " << y << ", " << theta << ")" << std::endl;

        x += noisy_v * sin(theta) * (10.f / 1000.f);
        y += noisy_v * cos(theta) * (10.f / 1000.f);
        theta += (noisy_w * (10.f / 1000.f));
        theta = sanitizeAngle(theta);

        //std::cout << "THETA: " << theta << std::endl;

        this->particles(i, 0) = x;
        this->particles(i, 1) = y;
        this->particles(i, 2) = theta;
    }

    previousControlInput = controlInput;

//     // // std::cout << "end func" << std::endl;

    //  std::cout << "new Particles: " << std::endl;
    //  std::cout << this->particles << std::endl;
}

bool pointIntersects(VectorXd point, double angle, VectorXd segmentStart, VectorXd segmentEnd) {
    Eigen::Vector2d direction(cos(angle), sin(angle));

    Eigen::Vector2d segmentDirection = segmentEnd - segmentStart;

    Eigen::Matrix2d A;
    A << -direction(0), segmentDirection(0),
        -direction(1), segmentDirection(1);

    Eigen::Vector2d B = segmentStart - point;

    Eigen::Vector2d solution = A.colPivHouseholderQr().solve(B);

    double t = solution(0);
    double s = solution(1);

    return (s >= 0 && s <= 1 && t >= 0);
}


bool ccw(VectorXd a, VectorXd b, VectorXd c) {
    return (c(1) - a(1)) * (b(0) - a(0)) > (b(1) - a(1)) * (c(0) - a(0));
}

bool pointDoesIntersect(VectorXd segment1Start, VectorXd segment1End, VectorXd segment2Start, VectorXd segment2End) {
    return ccw(segment1Start, segment2Start, segment2End) != ccw(segment1End, segment2Start, segment2End) && ccw(segment1Start, segment1End, segment2Start) != ccw(segment1Start, segment1End, segment2End);
}
//https://mathworld.wolfram.com/Line-LineIntersection.html
VectorXd pointAtIntersect(VectorXd segment1Start, VectorXd segment1End, VectorXd segment2Start, VectorXd segment2End) {
    segment1Start.conservativeResize(2);

    double x1 = segment1Start(0);
    double y1 = segment1Start(1);
    double x2 = segment1End(0);
    double y2 = segment1End(1);
    double x3 = segment2Start(0);
    double y3 = segment2Start(1);
    double x4 = segment2End(0);
    double y4 = segment2End(1);

    double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

    double px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / det;
    double py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / det;

    VectorXd point(2);
    point << px, py;

    return point;
}

double distanceAtIntersect(VectorXd position, VectorXd position2) {
    position.conservativeResize(2);
    double distance = (position2 - position).norm();

    return distance;
}

double distanceToSegment(VectorXd point, VectorXd segmentStart, VectorXd segmentEnd) {
    //https://stackoverflow.com/a/74819044 thank you whoever this is (JEESUS)
    double x1 = segmentStart(0);
    double y1 = segmentStart(1);
    double x2 = segmentEnd(0);
    double y2 = segmentEnd(1);
    double x = point(0);
    double y = point(1);
    double theta = point(2);

    double xp, yp;

    if (x2 == x1) {
        // Calculate intersection with a vertical line
        if (cos(theta) != 0) {
            double rm = sin(theta) / cos(theta);
            yp = y + rm * (x1 - x);
            xp = x1;
        }
        else {
            return false;  // No intersection if the line is vertical (theta = pi/2 or 3*pi/2)
        }
    }
    else if (cos(theta) == 0) {
        // Line is vertical
        double lm = (y2 - y1) / (x2 - x1);
        xp = (y1 - lm * x1 - y + sin(theta) * x) / (sin(theta) - lm);
        yp = sin(theta) * xp + y - sin(theta) * x;
    }
    else {
        // General case: non-vertical and non-horizontal line
        double rm = sin(theta) / cos(theta);
        double lm = (y2 - y1) / (x2 - x1);
        xp = (y1 - lm * x1 - y + rm * x) / (rm - lm);
        yp = rm * xp + y - rm * x;
    }

    return sqrt(pow(x - xp, 2) + pow(y - yp, 2));
}


bool pointIntersects(VectorXd point, VectorXd segmentStart, VectorXd segmentEnd) {
    //https://stackoverflow.com/a/74819044 thank you whoever this is (JEESUS)
    double x1 = segmentStart(0);
    double y1 = segmentStart(1);
    double x2 = segmentEnd(0);
    double y2 = segmentEnd(1);
    double x = point(0);
    double y = point(1);
    double theta = point(2);


    double theta1 = atan2(y1 - y, x1 - x);
    double theta2 = atan2(y2 - y, x2 - x);
    theta1 = remainder(theta1, M_PI);
    theta2 = remainder(theta2, M_PI);
    double dtheta = remainder(theta2 - theta1, M_PI);
    double ntheta = remainder(theta - theta1, M_PI);
    dtheta = atan2(sin(dtheta), cos(dtheta));
    ntheta = atan2(sin(ntheta), cos(ntheta));
    dtheta = remainder(dtheta, M_PI);
    ntheta = remainder(ntheta, M_PI);

    return (std::abs(ntheta) <= std::abs(dtheta));
}

bool circleIntersect(VectorXd centerSphere, double radius, VectorXd start, VectorXd end) {
    start.conservativeResize(2);
    // std::cout << "Checking intersection" << std::endl;
    //std::cout << "centerSphere: " << centerSphere.transpose() << std::endl;
    // std::cout << "radius: " << radius << std::endl;
    // std::cout << "start: " << start.transpose() << std::endl;
    // std::cout << "end: " << end.transpose() << std::endl;
    // quadratic formula for circle intersect:
    VectorXd d = end - start;
    VectorXd f = start - centerSphere;

    float a = d.dot(d);
    float b = 2 * f.dot(d);
    float c = f.dot(f) - radius * radius;
    float discriminant = b * b - 4 * a * c;

    // a possible intersection was found
    if (discriminant >= 0) {
        discriminant = sqrt(discriminant);

        float t1 = (-b - discriminant) / (2 * a);
        float t2 = (-b + discriminant) / (2 * a);

        //std::cout << "Intersection found!" << std::endl;

        return true;

        // priortize further down the path:
        // if (t2 >= 0 && t2 <= 1) return t2;
        // else if (t1 >= 0 && t1 <= 1) return t1;
    }

    // no intersection found:
   // printf("No intersection was found! \n");
    return false;
};

// bool pointIntersects(VectorXd point, VectorXd segmentStart, VectorXd segmentEnd) {
//     double x1 = segmentStart(0);
//     double y1 = segmentStart(1);
//     double x2 = segmentEnd(0);
//     double y2 = segmentEnd(1);
//     double x = point(0);
//     double y = point(1);
//     double theta = point(2);

//     // Calculate the angles from the sensor to the segment endpoints
//     double theta1 = atan2(y1 - y, x1 - x);  // angle to the start of the segment
//     double theta2 = atan2(y2 - y, x2 - x);  // angle to the end of the segment

//     // Normalize the angles to [-π, π]
//     theta1 = remainderf(theta1, M_2_PI);
//     theta2 = remainderf(theta2, M_2_PI);

//     // Compute the difference in angles
//     double dtheta = remainderf(theta2 - theta1, M_2_PI);
//     double ntheta = remainderf(theta - theta1, M_2_PI);

//     // Check if the sensor's angle is within the range of the segment's angle difference
//     // We will adjust the condition to check for the absolute angle difference
//     bool inRange = std::abs(ntheta) <= std::abs(dtheta);

//     // std::cout << "theta1: " << theta1 << ", theta2: " << theta2 << std::endl;
//     // std::cout << "ntheta: " << ntheta << ", dtheta: " << dtheta << std::endl;


//     return inRange;
// }


/**
 * @brief Update the weights of the particles
 *
 * @param measurements The measurements from the sensors
 */
constexpr double maxDistance = 78.74;
constexpr double pillarRadius = 4;
void MCLOdometry::update(std::vector<double> measurements) {

    this->totalWeight = 0.0;
    this->maxWeight = 0.0;

    for (int i = 0; i < this->particles.rows(); ++i) {

        // Get [x, y, theta] of the particle
        double xParticlePosition = this->particles(i, 0);
        double yParticlePosition = this->particles(i, 1);
        double theta = this->particles(i, 2);

        VectorXd positionVector(3);
        positionVector << xParticlePosition, yParticlePosition, theta;

        // Calculate the [x, y, theta] for each segment based off offsets and particle position
        VectorXd sensor1Segment(2);
        sensor1Segment(0) = xParticlePosition + maxDistance * sin(theta - M_PI_2);
        sensor1Segment(1) = yParticlePosition + maxDistance * cos(theta - M_PI_2);

        VectorXd sensor2Segment(2);
        sensor2Segment(0) = xParticlePosition + maxDistance * sin(theta + M_PI_2);
        sensor2Segment(1) = yParticlePosition + maxDistance * cos(theta + M_PI_2);

        VectorXd sensor3Segment(2);
        sensor3Segment(0) = xParticlePosition + maxDistance * sin(theta + M_PI);
        sensor3Segment(1) = yParticlePosition + maxDistance * cos(theta + M_PI);

        double measuredDistance1 = measurements.at(0);
        double measuredDistance2 = measurements.at(1);
        double measuredDistance3 = measurements.at(2);
       // double measuredPDistance1 = measurements.at(3);
       // double measuredPDistance2 = measurements.at(4);
       //double measuredPDistance3 = measurements.at(5);
        double measuredOdomX = measurements.at(3);
        double measuredOdomY = measurements.at(4);
        double measuredOdomTheta = measurements.at(5);

        // this->weights(i) *= normal_pdf(measuredOdomX, xParticlePosition, this->mOdomNoiseCovariance);
        // this->weights(i) *= normal_pdf(measuredOdomY, yParticlePosition, this->mOdomNoiseCovariance);
        // this->weights(i) *= normal_pdf(measuredOdomTheta, theta, this->mOdomNoiseCovariance);

        /*
            landmarks look like this:
            [
            1, 5, 2, 5 (xStart, yStart, xEnd, yEnd)
            ]
        */
        // Check if the sensors would intersect a landmark
        //std::cout << "landmark size: " << this->landmarks.transpose() << std::endl;
        for (int j = 0; j < 4; ++j) {
            //std::cout << "index of landmarks: " << j << std::endl;
            VectorXd landmarkStart(2);
            landmarkStart << landmarks(j, 0), landmarks(j, 1);

            VectorXd landmarkEnd(2);
            landmarkEnd << landmarks(j, 2), landmarks(j, 3);

            bool sensor1Intersection = pointDoesIntersect(positionVector, sensor1Segment, landmarkStart, landmarkEnd);
            bool sensor2Intersection = pointDoesIntersect(positionVector, sensor2Segment, landmarkStart, landmarkEnd);
            //bool sensor3Intersection = pointDoesIntersect(positionVector, sensor3Segment, landmarkStart, landmarkEnd);
            bool sensor3Intersection = false;

            // If either one intersects, calculate what the distance would be from the sensor to the landmark segment
            // then update the particle weights
            if (sensor1Intersection && measuredDistance1 != -999) {
            //    std::cout << "LEFT SENSOR DETECT LANDMARK" << std::endl;
                VectorXd intersectionPos = pointAtIntersect(positionVector, sensor1Segment, landmarkStart, landmarkEnd);
                double predictedDistance = distanceAtIntersect(positionVector, intersectionPos);
                //this->weights(i) *= exp(-(pow((measuredDistance1 - predictedDistance), 2)) / (pow(this->mNoiseCovariance, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(this->mNoiseCovariance, 2)));
                this->weights(i) *= normal_pdf(measuredDistance1, predictedDistance, this->mNoiseCovariance);
            }

            if (sensor2Intersection && measuredDistance2 != -999) {
              //  std::cout << "RIGHT SENSOR DETECT LANDMARK" << std::endl;
                VectorXd intersectionPos = pointAtIntersect(positionVector, sensor2Segment, landmarkStart, landmarkEnd);
                double predictedDistance = distanceAtIntersect(positionVector, intersectionPos);
                //this->weights(i) *= exp(-(pow((measuredDistance2 - predictedDistance), 2)) / (pow(this->mNoiseCovariance, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(this->mNoiseCovariance, 2)));
                this->weights(i) *= normal_pdf(measuredDistance2, predictedDistance, this->mNoiseCovariance);
            }

            if (sensor3Intersection && measuredDistance3 != -999) {
               // std::cout << "BACK SENSOR DETECT LANDMARK" << std::endl;
                VectorXd intersectionPos = pointAtIntersect(positionVector, sensor3Segment, landmarkStart, landmarkEnd);
                double predictedDistance = distanceAtIntersect(positionVector, intersectionPos);
                //this->weights(i) *= exp(-(pow((measuredDistance2 - predictedDistance), 2)) / (pow(this->mNoiseCovariance, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(this->mNoiseCovariance, 2)));
                this->weights(i) *= normal_pdf(measuredDistance3, predictedDistance, this->mNoiseCovariance);
            }
        }

        for (int j = 4; j < 7; ++j) {
            VectorXd pillar(2);
            pillar << landmarks(j, 0), landmarks(j, 1);

            //std::cout << landmarks.row(j) << std::endl;

            bool sensor1Intersection = circleIntersect(pillar, pillarRadius, positionVector, sensor1Segment);
            bool sensor2Intersection = circleIntersect(pillar, pillarRadius, positionVector, sensor2Segment);
            //bool sensor3Intersection = circleIntersect(pillar, pillarRadius, positionVector, sensor3Segment);
            bool sensor3Intersection = false;

            // If either one intersects, calculate what the distance would be from the sensor to the landmark segment
            // then update the particle weights
            if (sensor1Intersection) {
                //std::cout << "LEFT SENSOR DETECT PILLAR AT: " << pillar.transpose() << " | " << sensor1Segment.transpose() << std::endl;
                double predictedDistance = distanceAtIntersect(positionVector, pillar);
                //this->weights(i) *= exp(-(pow((measuredDistance1 - predictedDistance), 2)) / (pow(this->mNoiseCovariance, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(this->mNoiseCovariance, 2)));
                this->weights(i) *= normal_pdf(measuredDistance1, predictedDistance, this->mNoiseCovariance);
            }

            if (sensor2Intersection) {
                //std::cout << "RIGHT SENSOR DETECT PILLAR AT: " << pillar.transpose() <<  " | " << sensor2Segment.transpose() << std::endl;
                double predictedDistance = distanceAtIntersect(positionVector, pillar);
                //this->weights(i) *= exp(-(pow((measuredDistance2 - predictedDistance), 2)) / (pow(this->mNoiseCovariance, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(this->mNoiseCovariance, 2)));
                this->weights(i) *= normal_pdf(measuredDistance2, predictedDistance, this->mNoiseCovariance);
            }

            if (sensor3Intersection) {
                //std::cout << "BACK SENSOR DETECT PILLAR" << std::endl;
                double predictedDistance = distanceAtIntersect(positionVector, pillar);
                //this->weights(i) *= exp(-(pow((measuredDistance2 - predictedDistance), 2)) / (pow(this->mNoiseCovariance, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(this->mNoiseCovariance, 2)));
                this->weights(i) *= normal_pdf(measuredDistance3, predictedDistance, this->mNoiseCovariance);
            }
        }

        // Store the updated weight for the particle

        // Accumulate total weight for normalization
        this->totalWeight += this->weights(i);

        if (this->weights(i) > maxWeight) {
            this->maxWeight = this->weights(i);
        }

        //std::cout << "-------------------------------------------------------------" << std::endl;

        //wait(100, msec);
    }

    // Normalize all particle weights
    this->weights /= this->weights.sum();
}

double getRandomNumber() {

    std::random_device rdS;
    std::mt19937 genS(rdS());

    std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(genS);
}

/**
 * @brief Resample the particles systematically
 */
void MCLOdometry::resample() {

    //std::cout << "resampling" << std::endl;

    // int index = static_cast<int>(getRandomNumber() * this->particles.rows());
    // double beta = 0.0;

    // for (int i = 0; i < this->particles.rows(); ++i) {
    //     beta += getRandomNumber() * 2 * this->maxWeight;

    //    //std::cout << "index of: " << index << ", beta: " << beta << std::endl;

    //     // Make sure we don't go out of bounds
    //     while (beta > this->weights(index)) {
    //         beta -= this->weights(index);
    //         index = (index + 1) % this->particles.rows();

    //         // Debugging
    //         //std::cout << "Adjusted beta: " << beta << ", new index: " << index << std::endl;
    //     }

    //     // Assign the particle
    //     this->particles(i) = this->particles(index);
    // }

    VectorXd indicies = systematic_resample(this->weights);

    for (int i = 0; i < this->particles.rows(); ++i) {
       int index = static_cast<int>(indicies(i));
       if (index >= 0 && index < this->weights.size()) {
            this->particles.row(i) = this->particles.row(index);
       }
    }

    for (int i = 0; i < this->weights.size(); ++i) {
       int index = static_cast<int>(indicies(i));
       if (index >= 0 && index < this->weights.size()) {
            this->weights(i) = this->weights(index);
       }
    }

    //this->weights /= this->weights.sum();
   /* std::cout << "weights: " << std::endl;
    std::cout << this->weights.transpose() << std::endl;*/
}


/**
 * @brief Get back the estimated positions and their variances
 */
std::vector<std::vector<double>> MCLOdometry::estimate() {

    VectorXd xPositions = this->particles.col(0);
    VectorXd yPositions = this->particles.col(1);
    VectorXd thetaPositions = this->particles.col(2);

    double xMean = calculate_weighted_mean(xPositions, this->weights);
    double yMean = calculate_weighted_mean(yPositions, this->weights);
    double thetaMean = calculate_weighted_mean(thetaPositions, this->weights);

    double xVariances = calculate_weighted_variance(xPositions, this->weights, xMean);
    double yVariances = calculate_weighted_variance(yPositions, this->weights, yMean);
    double thetaVariances = calculate_weighted_variance(thetaPositions, this->weights, thetaMean);

    std::vector<double> means = { xMean, yMean, thetaMean };
    std::vector<double> variances = { xVariances, yVariances, thetaVariances };

    return { means, variances };
}

MatrixXd MCLOdometry::getParticles()
{
    return this->particles;
}