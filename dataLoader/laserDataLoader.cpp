#include "laserDataLoader.h"

#include <fstream>
#include <sstream>
#include <cstdlib> // For EXIT_FAILURE.
#include <iostream>

using namespace imr::laserDataLoader;

LaserDataLoader::LaserDataLoader(const char *_dataFile, const int nMeasurements, const std::string & _keyWord)
{
    int measurement_count = 0;

    std::ifstream ifs(_dataFile);
    if (ifs.fail())
    {
        std::cerr << "File " << _dataFile << " cannot be opened." << std::endl;
        exit(EXIT_FAILURE);
    }

    while (ifs && measurement_count < nMeasurements)
    {
        // Read line from file input stream
        std::string line;
        std::getline(ifs, line);

        // Construct string stream
        std::stringstream iss_line(line);
        std::string keyword;
        if ((iss_line >> keyword).fail())
        {
            continue;
        }
        if (keyword.compare(_keyWord) == 0)
        {
            std::cout << "Loading measurement " << ++measurement_count;
            
            std::string temp;

            unsigned int nmeas;
            if ((iss_line >> nmeas).fail())
            {
                std::cerr << "Error parsing type in input\n";
                exit(EXIT_FAILURE);
            }

            std::cout << " containing " << nmeas << " measurements." << std::endl;

            Measurement new_measurement;
            // Read all laser measurements
            for (unsigned int i = 0; i < nmeas; i++)
            {
                double measurement;
                if ((iss_line >> measurement).fail())
                {
                    std::cerr << "Error parsing measurement data in input\n";
                    exit(EXIT_FAILURE);
                }
                new_measurement.scan.push_back(measurement);
            }

            // Read position (x,y,theta)
            if ((iss_line >> new_measurement.position.x).fail())
            {
                std::cerr << "Error parsing position x in input\n";
                exit(EXIT_FAILURE);
            }
            if ((iss_line >> new_measurement.position.y).fail())
            {
                std::cerr << "Error parsing position y in input\n";
                exit(EXIT_FAILURE);
            }
            if ((iss_line >> new_measurement.position.phi).fail())
            {
                std::cerr << "Error parsing position phi in input\n";
                exit(EXIT_FAILURE);
            }

	    measurements.push_back(new_measurement);
        }
    }

    std::cout << "Data loaded ..." << std::endl;
}
