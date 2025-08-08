//
// Created by aze_pc0218 on 7/29/25.
//


#ifndef CSV_FILE_CREATOR_HPP
#define CSV_FILE_CREATOR_HPP

#include <string>
#include <pcl/io/pcd_io.h>

void CreateCSV(const std::string& filename);
void ConvertPCDToCSV(const std::string& pcd_file, const std::string& csv_file);

#endif
