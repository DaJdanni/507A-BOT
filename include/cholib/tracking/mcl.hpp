#pragma once
#include "vex.h"
#include <cmath>
#include <random>

using namespace Eigen;

namespace choMCL {
    namespace util {
        constexpr inline double sanitizeAngle(double angle, bool inRadians = true) {
            if (inRadians) return fmod(fmodf(angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
            else return fmod(fmod(angle, 360) + 360, 360);
        }

        inline double normalizePDF(double mean, double stdDeviation, double x) {
            double exponent = -0.5 * pow((x - mean) / stdDeviation, 2);
            return (1.f / (stdDeviation * sqrt(2 * M_PI))) * exp(exponent);
        }

        inline std::uniform_real_distribution<> generateDistributions(double min, double max) {
            std::uniform_real_distribution<> distribution(min, max);
            return distribution;
        }

        inline double neff(VectorXd weights) {
            return 0.0;
        }
    };

    namespace geometry {
        inline bool ccw(VectorXd a, VectorXd b, VectorXd c) {
            return (c(1) - a(1)) * (b(0) - a(0)) > (b(1) - a(1)) * (c(0) - a(0));
        }

        inline bool segmentsIntersect(VectorXd segment1Start, VectorXd segment1End, VectorXd segment2Start, VectorXd segment2End) {
            return ccw(segment1Start, segment2Start, segment2End) != ccw(segment1End, segment2Start, segment2End) &&
                   ccw(segment1Start, segment1End, segment2Start) != ccw(segment1Start, segment1End, segment2End);
        }

        inline bool circleIntersect(VectorXd centerSphere, double radius, VectorXd start, VectorXd end) {
            start.conservativeResize(2);
            VectorXd d = end - start;
            VectorXd f = start - centerSphere;
        
            float a = d.dot(d);
            float b = 2 * f.dot(d);
            float c = f.dot(f) - radius * radius;
            float discriminant = b * b - 4 * a * c;
        
            if (discriminant >= 0) {
                discriminant = sqrt(discriminant);
        
                float t1 = (-b - discriminant) / (2 * a);
                float t2 = (-b + discriminant) / (2 * a);

                return true;
            }

            return false;
        }

        inline double distanceAtIntersect(VectorXd position, VectorXd position2) {
            position.conservativeResize(2);
            double distance = (position2 - position).norm();
            return distance;
        }

        inline double distanceToSegment(VectorXd point, VectorXd segmentStart, VectorXd segmentEnd) {
            return 0.0;
        }

        inline VectorXd pointAtIntersect(VectorXd segment1Start, VectorXd segment1End, VectorXd segment2Start, VectorXd segment2End) {
            VectorXd point(2);
            return point;
        }
    };

    class MCLFilter {
        MatrixXd particles;
    };
}
