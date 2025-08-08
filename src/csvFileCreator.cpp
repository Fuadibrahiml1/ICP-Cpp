//
// Created by aze_pc0218 on 7/29/25.
//
#include "csvFileCreator.hpp"
#include <fstream>
#include <pcl/io/pcd_io.h>


void CreateCSV(const std::string& filename) {
    std::ofstream fout(filename, std::ios::out | std::ios::app);
    if (!fout.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    else {
        std::cerr << "Successfully opened file: " << filename << std::endl;
    }
    fout.close();
}