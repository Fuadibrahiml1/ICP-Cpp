/*#include "../include/dataLogger.hpp"
#include <fstream>
#include <vector>

// Make sure you include the correct DataLoader header here
#include "../include/dataLoader.hpp"  // Add this line to correctly access DataLoader::Point

void Logger::saveCSV(const std::string& filePath, const std::vector<DataLoader::point>& points) {
    std::ofstream csvFile(filePath);

    if (!csvFile.is_open()) {
        std::cerr << "Failed to open the file for writing: " << filePath << std::endl;
        return;
    }

    // Write the CSV header
    csvFile << "x,y,z" << std::endl;

    // Write point cloud data
    for (const auto& point : points) {
        csvFile << point.x << "," << point.y << "," << point.z << std::endl;
    }

    csvFile.close();
    std::cout << "Data saved successfully to " << filePath << std::endl;
}*/