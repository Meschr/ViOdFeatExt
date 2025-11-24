#include "CaptureDevice.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <iomanip>


double integral (double time1, double time2, double accel){
    return (time2 - time1)* 1e-6 * accel;
}

int main() {
    std::ifstream f("/home/robinazv/ViOdFeatExt/LogData/2025-11-18_1/2025-11-18_12-27-10_angularMovementRoomMoon90Deg/data.csv");
    std::string line;

    std::vector<double> timestamps, ax, ay, az; //accel
    std::vector<double> px, py, pz;  //positions
    
    std::getline(f, line);

    //for (int i = 0; i < 100 && std::getline(f, line); i++) {
    while (std::getline(f, line)) {
        std::stringstream ss(line);
        std::string item;
        int col = 0;

        double t = 0, x = 0, y = 0, z = 0;

        while (std::getline(ss, item, ',')) {
            if (col == 0) t = std::stod(item);
            if (col == 2) x = std::stod(item);
            if (col == 3) y = std::stod(item);
            if (col == 4) z = std::stod(item);
            col++;
        }

        timestamps.push_back(t);
        ax.push_back(x);
        ay.push_back(y);
        az.push_back(z);
    }

    for (int i = 1; i < timestamps.size(); i++) {
        double dt = timestamps[i] - timestamps[i-1];

        double vx = integral(timestamps[i-1], timestamps[i], ax[i-1]);
        double vy = integral(timestamps[i-1], timestamps[i], ay[i-1]);
        double vz = integral(timestamps[i-1], timestamps[i], az[i-1]);

        if (i == 1) {
            px.push_back(vx);
            py.push_back(vy);
            pz.push_back(vz);
        } else {
            px.push_back(px.back() + vx);
            py.push_back(py.back() + vy);
            pz.push_back(pz.back() + vz);
        }
    }

    std::ofstream out("../tests/output.csv");
    out << std::fixed << std::setprecision(10); 
    for (int i = 0; i < px.size(); i++)
        out << px[i] << "," << py[i] << "," << pz[i] << "\n";
    }
