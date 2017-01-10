/**
 * ICP localization 
 * Skeleton code for teaching 
 * A3M33MKR
 * Czech Technical University 
 * Faculty of Electrical Engineering
 * Intelligent and Mobile Robotics group
 *
 * Authors: Zdeněk Kasl, Karel Košnar kosnar@labe.felk.cvut.cz
 *
 * Licence: MIT (see LICENSE file)
 **/

#include <cmath>
#include <cassert>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include "gui/gui.h"
#include "dataLoader/laserDataLoader.h"

using namespace imr;
using namespace gui;
using namespace laserDataLoader;


double toRadians(const double alpha);

// Function performing the iterative closest point method - you must to implement it.
RobotPosition
icp(Measurement reference, Measurement actual, RobotPosition previousPosition, int iteration, Gui gui1);

// Function convert RobotPosition to Point structure (for drawing).
Point robotPosition2point(const RobotPosition &rp);

// Convert the laser scan into vector of points in Cartesian coordinate system using the odometry from the Measurement structure.
void calculateRawPoints(RawPoints &rp, const Measurement &m);

// Convert the laser scan into vector of points in Cartesian coordinate system using given robot position p.
void calculateRawPoints(RawPoints &rp, const Measurement &m, const RobotPosition &p);

// Convert point in polar coordinate system  into Cartesian coordinate system (possibly moved by Robot Position).
void polar2cartesian(Point &p, double alpha, double r, const RobotPosition &p0);

Point getClosestReference(Point &currPoint, const RawPoints &refPoints);

double getDistancePow(Point &p1, Point &p2);

void help(char **argv) {
    std::cout << "\nUsage of the program " << argv[0] << ":\n"
              << "Parameter [-h or -H] displays this message.\n"
              << "Parameter [-f or -F] specifies path to data."
              << "Parameter [-m or -M] specifies number of measurements taken,\n"
              << "   default number of measurements is 2.\n"
              << std::endl;
}

int main(int argc, char** argv)
{
    int nMeasurements = 2;
    int nStart = 0;
    char *dataFile;

    // argument count must be greater than three
    // >>> at least source file must be specified
    if (argc < 3)
    {
        help(argv);
        return EXIT_FAILURE;
    }

    // Parse all console parameters
    for (int i = 0; i < argc; i++)
    {
        if (argv[i][0] == '-')
        {
            switch (argv[i][1])
            {
                //> HELP
                case 'H' :
                case 'h' :
                    help(argv);
                    break;

                    //> Source file
                case 'F' :
                case 'f' :
                    assert(i + 1 < argc);
                    dataFile = argv[i + 1];
                    break;

                    //> Number of Measurements
                case 'M' :
                case 'm' :
                    assert(i + 1 < argc);
                    assert(atoi(argv[i + 1]) > 1);
                    nMeasurements = atoi(argv[i + 1]);
                    break;
                case 'S' :
                case 's' :
                    assert(i + 1 < argc);
                    assert(atoi(argv[i + 1]) > 1);
                    nStart = atoi(argv[i + 1]);
                    break;

                default :
                    std::cout << "Parameter \033[1;31m" << argv[i] << "\033[0m is not valid!\n"
                              << "Use parameter -h or -H for help." << std::endl;
                    break;
            }
        }
    }
    // All parameters parsed.

    std::cout << "Number of measuremetns taken: " << nMeasurements << "\n"
              << "Source file: " << dataFile
              << std::endl;

    // Load data.
    LaserDataLoader loader(dataFile, nMeasurements, "FLASER");

    // RawPoints (defined in gui/gui.h) is a vector of Points.
    RawPoints initial;
    RawPoints tentative;

    // Measurement (defined in dataLoader/laserDataLoader.h) is a structure
    // holding the range data and odometry position.
    Measurement previous;

    // RobotPosition (defined in dataLoader/laserDataLoader.h).
    RobotPosition previousPosition;

    // Initialize previous measurement.
    previous = loader[nStart];
    // Initialize reference scan.
    calculateRawPoints(initial, loader[nStart]);
    // Initialize previous position .
    previousPosition = loader[nStart].position;
    // Initialize the next scan.
    calculateRawPoints(tentative, loader[1]);
    // Initial robot position.
    Gui gui(initial, tentative, robotPosition2point(loader[nStart].position));

    // Comment the line below in order to let the program continue right away.
//    gui.startInteractor();

    for (int i = nStart; i < nMeasurements; i++)
    {
        // Compute the points of non-corrected measurement.
        RawPoints tentative;
        Measurement actual = loader[i];
        calculateRawPoints(tentative, actual);
        // Vizualize tentative non-corrected points (red).
        gui.setTentativePoints(tentative);
//        usleep(1000000);
        // Calculate the transform and transform data.
        RobotPosition correctedPosition = icp(previous, actual, previousPosition, 300, gui);
        // Correct the actual measurement according to the computed ICP-corrected position.
        RawPoints initial;
        calculateRawPoints(initial, actual, correctedPosition);

        // Clear map (to show only actually given scan).
//        gui.clearMapPoints();
        // Add the new measurement to the map.
        gui.setPointsToMap(initial, robotPosition2point(actual.position), robotPosition2point(correctedPosition));

        //> Comment line below in order to let the program
        //> continue right away.
//        if (i % 20 == 0)
//            gui.startInteractor();
        previous = actual;
        previousPosition = correctedPosition;
    }
    gui.clearTentativePoints();
    gui.startInteractor();

    return EXIT_SUCCESS;
}


/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//                               implement ICP here
/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


RobotPosition icp(Measurement reference, Measurement actual, RobotPosition previousPosition, int iterations, Gui gui) {
    RobotPosition newPos;
    newPos.phi = previousPosition.phi + actual.position.phi - reference.position.phi;
    newPos.x = previousPosition.x + actual.position.x - reference.position.x;
    newPos.y = previousPosition.y + actual.position.y - reference.position.y;
    RawPoints refPoints;
    RawPoints tempPoints;
    calculateRawPoints(refPoints, reference, previousPosition);
    calculateRawPoints(tempPoints, reference, newPos);
    double errLast = DBL_MAX;
    int iteration;
    for (iteration = 0; iteration < iterations; iteration++) {
        double distCoef = 1;
        double treshold = 1.2;
        RawPoints currPoints;
        calculateRawPoints(currPoints, actual, newPos);
        gui.setTentativePoints(currPoints);

//        gui.clearMapPoints();
//        usleep(10000);

        unsigned long size = currPoints.size();

        Point closestPairs[size];
        double xAvg = 0;
        double yAvg = 0;
        double xPrimeAvg = 0;
        double yPrimeAvg = 0;
        int size_ = 0;
        double err = 0;

        for (int i = 0; i < size; i++) {
            Point currPoint = currPoints[i];
            Point refPoint = getClosestReference(currPoint, refPoints);
            closestPairs[i] = refPoint;
            Point p(newPos.x,newPos.y);
            double distance = getDistancePow(p, currPoint);
            if (((currPoint.x - refPoint.x) * (currPoint.x - refPoint.x) + (currPoint.y - refPoint.y) * (currPoint.y - refPoint.y))* (1 - distCoef/distance) *100 > treshold) {
                continue;
            }
            err+=(currPoint.x - refPoint.x)*(currPoint.x - refPoint.x) + (currPoint.y - refPoint.y)*(currPoint.y - refPoint.y);
            xAvg += currPoint.x;
            yAvg += currPoint.y;
            xPrimeAvg += refPoint.x;
            yPrimeAvg += refPoint.y;
            size_ ++;
        }
//        std::cout << "error: "<< err*100 << " in " << points << " points" << endl;

        xAvg /= size_;
        yAvg /= size_;
        xPrimeAvg /= size_;
        yPrimeAvg /= size_;

        double sum1 = 0;
        double sum2 = 0;
        double sum3 = 0;
        double sum4 = 0;
        for (int i = 0; i < size; i++) {
            Point currPoint = currPoints[i];
            Point refPoint = closestPairs[i];
            double x = currPoint.x;
            double y = currPoint.y;
            double xPrime = refPoint.x;
            double yPrime = refPoint.y;
            Point p(newPos.x,newPos.y);
            double distance = getDistancePow(p, currPoint);
            if (((currPoint.x - refPoint.x) * (currPoint.x - refPoint.x) + (currPoint.y - refPoint.y) * (currPoint.y - refPoint.y)) * (1 - distCoef/distance) *100 > treshold) {
                continue;
            }
            sum1 += (x - xAvg) * (yPrime - yPrimeAvg);
            sum2 += (y - yAvg) * (xPrime - xPrimeAvg);
            sum3 += (x - xAvg) * (xPrime - xPrimeAvg);
            sum4 += (y - yAvg) * (yPrime - yPrimeAvg);
        }

        double omega = atan2(sum1 - sum2, sum3 + sum4);
        double tX = xPrimeAvg - (xAvg * cos(omega) - yAvg * sin(omega));
        double tY = yPrimeAvg - (xAvg * sin(omega) + yAvg * cos(omega));

        newPos.phi += omega;
        double newX = newPos.x*cos(omega) - newPos.y * sin(omega) + tX;
        double newY = newPos.x*sin(omega) + newPos.y * cos(omega) + tY;
        newPos.x = newX;
        newPos.y = newY;
//        newPos.x += cos(omega) * tX - sin(omega) * tY;
//        newPos.y += sin(omega) * tX + cos(omega) * tY;
//        newPos.x += tX;
//        newPos.y += tY;

        if (fabs(errLast - err) < 0.0000000001) {
            break;
        }
        errLast = err;
        if (iteration+1 == iterations) {
            std::cout << "error: " << err << endl;
            std::cout << "sum1: " << sum1 << endl;
            std::cout << "sum2: " << sum2 << endl;
            std::cout << "sum3: " << sum3 << endl;
            std::cout << "sum4: " << sum4 << endl;
            std::cout << "tx: " << tX << endl;
            std::cout << "ty: " << tY << endl;
            std::cout << "omega: " << omega << endl;
            std::cout << "size: " << size_ << endl;
        }
    }
    std::cout << "iterations: " << iteration << endl;

    return newPos;
}

/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

Point robotPosition2point(const RobotPosition &rp) {
    return Point(rp.x, rp.y);
}

void calculateRawPoints(RawPoints &rp, const Measurement &m) {
    calculateRawPoints(rp, m, m.position);
}

void calculateRawPoints(RawPoints &rp, const Measurement &m, const RobotPosition &pos) {
    const double laserResolution = 0.5; // deg
    const double laserShift = -90;
    for (unsigned int j = 0; j < m.scan.size(); j++) {
        if (m.scan[j] > 8.0)
            continue;
        Point p;
        polar2cartesian(p, toRadians(j * laserResolution + laserShift), m.scan[j], pos);
        rp.push_back(p);
    }
}

void polar2cartesian(Point &p, double alpha, double r, const RobotPosition &p0) {
    p.x = p0.x + r * cos(alpha + p0.phi);
    p.y = p0.y + r * sin(alpha + p0.phi);
}

double toRadians(const double alpha) {
    return (alpha * M_PI) / 180.0;
}

Point getClosestReference(Point &currPoint, const RawPoints &refPoints) {
    Point closestPoint;
    double minDistancePow = DBL_MAX;
    int temp = 0;
    for (int i = 0; i < refPoints.size() - 1; i++) {
        Point currClosestPoint;
        Point refPoint1 = refPoints[i];
        Point refPoint2 = refPoints[i + 1];
        double currDistancePow;
        double cPow = getDistancePow(refPoint1, refPoint2);
        double c = sqrt(cPow);
        double dPow = getDistancePow(currPoint, refPoint1);
        double ePow = getDistancePow(currPoint, refPoint2);
        double f = (cPow + dPow - ePow) / 2 * c;
        currDistancePow = getDistancePow(currPoint, refPoint1);
        if (currDistancePow < minDistancePow) {
            minDistancePow = currDistancePow;
            closestPoint = refPoint1;
        }
        currDistancePow = getDistancePow(currPoint, refPoint2);
        if (currDistancePow < minDistancePow) {
            minDistancePow = currDistancePow;
            closestPoint = refPoint2;
        }
        if (f < 0) {
            continue;
        }
        if (f > c) {
            continue;
        }
//            std::cout << "c: "<< c << endl;
        if (c > 0.02) {
            temp++;
            continue;
        }

        currClosestPoint.x = refPoint1.x + (refPoint2.x - refPoint1.x) * f / c;
        currClosestPoint.y = refPoint1.y + (refPoint2.y - refPoint1.y) * f / c;
        currDistancePow = getDistancePow(currPoint, currClosestPoint);
        if (currDistancePow < minDistancePow) {
            minDistancePow = currDistancePow;
            closestPoint = currPoint;
        }
    }
//    std::cout << temp << endl;

    return closestPoint;
}

double getDistancePow(Point &point1, Point &point2) {
    double d1 = point1.x - point2.x;
    double d2 = point1.y - point2.y;
    return d1 * d1 + d2 * d2;
}
