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

#include<cmath>
#include<cassert>
#include<cstdlib>
#include<fstream>
#include<iostream>

#include "gui/gui.h"
#include "dataLoader/laserDataLoader.h"

using namespace imr;
using namespace gui;

using namespace laserDataLoader;

double toRadians(const double alpha);

//function performing the iterative closest point method - you must to implement it
RobotPosition icp(const Measurement reference, const Measurement actual, RobotPosition previousPosition);

//function convert RobotPosition to Point structure (for drawing)
Point robotPosition2point(const RobotPosition &rp);

//convert the laser scan into vector of points in Cartesian coordinate system using the odometry from the Measurement structure
void calculateRawPoints(RawPoints &rp, Measurement m);

//convert the laser scan into vector of points in Cartesian coordinate system using given robot position p
void calculateRawPoints(RawPoints &rp, Measurement m, RobotPosition p);

//convert point in polar coordinate system  into Cartesian coordinate system (possibly moved by Robot Position)
void polar2cartesian(Point &p, const double &alpha, const double &r, const RobotPosition &p0);

//cos from degrees
double cosDeg(double deg);
//sin from degrees
double sinDeg(double deg);
//euclidian distance
double euclidianDist(Point p1, Point p2);
//angle between two vector
double angleBetweenVectors(const Vector v1, const Vector v2);
//normalizate vector
void normalizateVector(Vector &v);
//multiply vector
void multiplyVector(Vector &v, double k);

void help(char** argv)
{
    std::cout << "\nUsage of the program " << argv[0]+2 << ":\n"
              << "Parameter [-h or -H] displays this message.\n"
              << "Parameter [-f or -F] specifies path to data."
              << "Parameter [-m or -M] specifies number of measurements taken,\n"
              << "   default number of measurements is 2.\n"
              << std::endl;
}

int main(int argc, char** argv)
{
    int nMeasurements = 2;
    char *dataFile;
    // argument count must be greater than three
    // >>> at least source file must be specified
    if (argc < 3) {
        help(argv);
        return EXIT_FAILURE;
    }

    // Parse all console parameters
    for (int i=0; i<argc; i++) {
        if (argv[i][0]=='-') {
            switch(argv[i][1]) {
                //> HELP
                case 'H' : case 'h' :
                    help(argv);
                    break;

                    //> Source file
                case 'F' : case 'f' :
                    assert(i+1 < argc);
                    dataFile = argv[i+1];
                    break;

                    //> Number of Measurements
                case 'M' : case 'm' :
                    assert(i+1 < argc);
                    assert(atoi(argv[i+1])>1);
                    nMeasurements = atoi(argv[i+1]);
                    break;

                default :
                    std::cout << "Parameter \033[1;31m" << argv[i] << "\033[0m is not valid!\n"
                              << "Use parameter -h or -H for help." << std::endl;
                    break;
            }
        }
    }
    // All parameters parsed

    std::cout << "Number of measuremetns taken: " << nMeasurements << "\n"
              << "Source file: " << dataFile
              << std::endl;

    // Load data
    LaserDataLoader loader(dataFile, nMeasurements, "FLASER");


    //RawPoints (defined in gui/gui.h) is a vector of Points
    RawPoints initial, tentative;
    //Measurement (defined in dataLoader/laserDataLoader.h) is a structure holding the range data and odometry position

    Measurement actual, previous;
    //RobotPosition (defined in dataLoader/laserDataLoader.h)
    RobotPosition correctedPosition, previousPosition;
    //initialize previous measurement
    previous = loader[0];
    // initialize reference scan
    calculateRawPoints(initial, loader[0]);
    // initialize previous position
    previousPosition = loader[0].position;
    // initialize the next scan
    calculateRawPoints(tentative, loader[1]);
    // initial robot position
    Gui gui(initial,tentative,robotPosition2point(loader[0].position));

    //> comment line below in order to let the program
    //> continue right away
    //gui.startInteractor();

    for(int i=1; i<nMeasurements; i++) {

        tentative.clear();
        /// compute points of non-corrected measurement
        calculateRawPoints(tentative, loader[i]);
        /// vizualize tentative non-corrected points (red)
        gui.setTentativePoints(tentative);

        actual = loader[i];
        /// calculate transform and transform data
        correctedPosition = icp(previous,actual, previousPosition);
        ///clear points
        initial.clear();
        /// correct actual measurement according the computed ICP-corrected position
        calculateRawPoints(initial, actual, correctedPosition);

        //> clear map (to show only actually given scan)
        //> gui.clearMapPoints();

        /// add new measurement to map
        gui.setPointsToMap(initial,robotPosition2point(loader[i].position),
                           robotPosition2point(correctedPosition));

        //> comment line below in order to let the program
        //> continue right away
        if (i%20 == 0) gui.startInteractor();
    }
    gui.clearTentativePoints();
    gui.startInteractor();

    return EXIT_SUCCESS;
}


/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//                               implement ICP here
/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


RobotPosition icp(const Measurement reference, const Measurement actual, RobotPosition previousPosition)
{

    Point closest_pairs[actual.scan.size()];  // x,y

    RawPoints actual_points, reference_points;

    calculateRawPoints(actual_points,actual);
    calculateRawPoints(reference_points,reference);

    for (int i = 0; i < actual_points.size()-1; ++i) {
        double closest_dist = VTK_DOUBLE_MAX;
        double dist;

        for (int j = 0; j < reference_points.size()-2; ++j) {
            Vector *v1 = new Vector(reference_points[j+1].x - reference_points[j].x,reference_points[j+1].y - reference_points[j].y );
            Vector *v2 = new Vector(actual_points[i].x - reference_points[j].x,actual_points[i].y - reference_points[j].y );
            double angle_alpha = angleBetweenVectors(*v1,*v2);
            Vector *v3 = new Vector(reference_points[j].x - reference_points[j+1].x,reference_points[j].y - reference_points[j+1].y );
            Vector *v4 = new Vector(actual_points[i].x - reference_points[j+1].x,actual_points[i].y - reference_points[j+1].y );
            double angle_beta = angleBetweenVectors(*v3,*v4);


            if (angle_alpha >= 1.5708) {
                dist = euclidianDist(reference_points[j],actual_points[i]);
                if(dist < closest_dist){
                    closest_dist = dist;
                    closest_pairs[i] = reference_points[j];
                    continue;
                }
            }
            if(angle_beta >= 1.5708) {
                dist  = euclidianDist(reference_points[j+1],actual_points[i]);
                if(dist < closest_dist){
                    closest_dist = dist;
                    closest_pairs[i] = reference_points[j+1];
                    continue;
                }
            }

            dist = sin(angle_alpha)*euclidianDist(reference_points[j],actual_points[i]);
            if(dist < closest_dist){
                closest_dist = dist;
                double AC = euclidianDist(reference_points[j],actual_points[i]);
                double va = sqrt((AC*AC)-(dist*dist));
                normalizateVector(*v1);
                multiplyVector(*v1,va);

                closest_pairs[i] = Point(reference_points[j].x + v1->x, reference_points[j].y + v1->y);

            }


        }
    }

    double x_avrg = 0;
    double y_avrg = 0;
    double x_avrg_prime = 0;
    double y_avrg_prime = 0;

    for (int i = 0; i < actual_points.size() - 1; ++i) {
        x_avrg += actual_points[i].x;
        y_avrg += actual_points[i].y;
    }
    for (int i = 0; i < actual.scan.size() - 1; ++i) {
        x_avrg_prime += closest_pairs[i].x;
        y_avrg_prime += closest_pairs[i].y;
    }
    x_avrg /= actual_points.size();
    y_avrg /= actual_points.size();
    x_avrg_prime /= actual.scan.size();
    y_avrg_prime /= actual.scan.size();

    double sum1 = 0;
    double sum2 = 0;
    double sum3 = 0;
    double sum4 = 0;

    for (int k = 0; k < actual.scan.size() - 1; ++k) {
        sum1 += (actual_points[k].x-x_avrg)*(closest_pairs[k].y-y_avrg_prime);
        sum2 += (actual_points[k].y-y_avrg)*(closest_pairs[k].x-x_avrg_prime);
        sum3 += (actual_points[k].x-x_avrg)*(closest_pairs[k].x-x_avrg_prime);
        sum4 += (actual_points[k].y-y_avrg)*(closest_pairs[k].y-y_avrg_prime);
    }

    double w_avrg = atan2(sum1-sum2,sum3+sum4);

    double Tx = x_avrg_prime-(x_avrg*cos(w_avrg)-y_avrg*sin(w_avrg));
    double Ty = y_avrg_prime-(x_avrg*sin(w_avrg)+y_avrg*cos(w_avrg));




    RobotPosition ret = actual.position;
    double w = w_avrg;

    double x_new = ret.x*cos(w) - ret.y*sin(w) + Tx;
    double y_new = ret.y*cos(w) + ret.x*sin(w) + Ty;

    ret.x = x_new;
    ret.y = y_new;
    ret.phi = w;

    return ret;
}

/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

Point robotPosition2point(const RobotPosition &rp)
{
    return Point(rp.x,rp.y);
}

void calculateRawPoints(RawPoints &rp, Measurement m)
{
    calculateRawPoints(rp,m,m.position);
}

void calculateRawPoints(RawPoints &rp, Measurement m, RobotPosition pos)
{
    const double laserResolution = 0.5; // deg
    const double laserShift = -90.0;
    for (int j=0; j<m.scan.size(); j++) {
        if (m.scan[j] > 50.0) continue;
        Point p;
        polar2cartesian(p,toRadians(j*laserResolution+laserShift),m.scan[j],pos);
        rp.push_back(p);
    }
}

void polar2cartesian(Point &p, const double &alpha, const double &r, const RobotPosition &p0)
{
    p.x = p0.x+r*cos(alpha+p0.phi);
    p.y = p0.y+r*sin(alpha+p0.phi);
}

double toRadians(const double alpha)
{
    return (alpha*M_PI)/180.0;
}

double cosDeg(double deg)
{
    return cos(toRadians(deg));
}
double sinDeg(double deg)
{
    return sin(toRadians(deg));
}
double euclidianDist(Point p1, Point p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.x-p2.y));
}

double angleBetweenVectors(const Vector v1, const Vector v2)
{
    double dot = v1.x*v2.x + v1.y*v2.y;     // dot product
    double det = v1.x*v2.y - v1.y*v2.x;     // determinant
    double angle = atan2(det, dot);   // atan2(y, x) or atan2(sin, cos)

    if(angle<0) angle*=-1;
    return angle;
}

void normalizateVector(Vector &v)
{
    double norm = sqrt((v.x*v.x)+(v.y*v.y));
    v.x /=norm;
    v.y /=norm;
}

void multiplyVector(Vector &v, double k)
{
    v.x *= k;
    v.y *= k;
}