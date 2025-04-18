#pragma once
#include "vex.h"

namespace choMCL {
    namespace util {
        constexpr double sanitizeAngle(double angle, bool inRadians = true) {
            if (inRadians == true) return fmod(fmodf(angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
            else return fmod(fmod(angle, 360) + 360, 360);
        }

        double normalizePDF(double mean, double stdDeviation, double x) {
            double exponent = -0.5 * pow((x - mean) / stdDeviation, 2);
            return (1.f / (stdDeviation * sqrt(2 * M_PI))) * exp(exponent);
        }

        std::uniform_real_distribution<> generateDistributions(double min, double max) {
            std::uniform_real_distribution<> distribution(min, max);
            return distribution;
        }
        double neff(VectorXd weights);
    }

    namespace geometry {
        bool ccw(VectorXd a, VectorXd b, VectorXd c) {
            return (c(1) - a(1)) * (b(0) - a(0)) > (b(1) - a(1)) * (c(0) - a(0));
        }
        bool segmentsIntersect(VectorXd segment1Start, VectorXd segment1End, VectorXd segment2Start, VectorXd segment2End) {
            return ccw(segment1Start, segment2Start, segment2End) != ccw(segment1End, segment2Start, segment2End) && ccw(segment1Start, segment1End, segment2Start) != ccw(segment1Start, segment1End, segment2End);
        }
        bool circleIntersect(VectorXd centerSphere, double radius, VectorXd start, VectorXd end) {
            start.conservativeResize(2);
            VectorXd d = end - start;
            VectorXd f = start - centerSphere;
        
            float a = d.dot(d);
            float b = 2 * f.dot(d);
            float c = f.dot(f) - radius * radius;
            float discriminant = b * b - 4 * a * c;
        
            // a possible intersection was found:
            if (discriminant >= 0) {
                discriminant = sqrt(discriminant);
        
                float t1 = (-b - discriminant) / (2 * a);
                float t2 = (-b + discriminant) / (2 * a);

                return true;
            }
        
            // no intersection found:
            return false;
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
    }

    class MCLFilter {
        MatrixXd particles;
    }
}