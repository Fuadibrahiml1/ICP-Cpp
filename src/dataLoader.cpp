#include "../include/dataLoader.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include "../include/csvFileCreator.hpp"


void CreateCSV() {
	std::fstream fout("csvFile.csv", std::ios::out | std::ios::app);
	fout.close();
}

void ConvertPCDToCSV(const std::string& pcd_file, const std::string& csv_file) {
	std::ifstream pcd(pcd_file);
	std::ofstream csv(csv_file, std::ios::out | std::ios::app);

	if (!pcd.is_open() || !csv.is_open()) {
		std::cerr << "Failed to open input or output file.\n";
		return;
	}

	std::string line;
	bool data_section = false;

	while (std::getline(pcd, line)) {
		if (!data_section) {
			if (line.find("DATA ascii") != std::string::npos)
				data_section = true;
			continue;
		}
		std::istringstream iss(line);
		float x, y, z;
		if (iss >> x >> y >> z) {
			csv << x << "," << y << "," << z << "\n";
		}
	}

	pcd.close();
	csv.close();
}