// flower_pot_locations.h
#ifndef GET_FLOWER_LOCATIONS_H
#define GET_FLOWER_LOCATIONS_H

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <tinyxml2.h>

std::vector<std::pair<float, float>> getFlowerPotLocations(const std::string& filePath);

std::vector<std::pair<float, float>> getFlowerPotLocations(const std::string& filePath) {
    std::vector<std::pair<float, float>> flowerPotLocations;
    
    // Get the root document
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(filePath.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
        std::cerr << "Error parsing the XML file." << std::endl;
        return flowerPotLocations;
    }

    // Start from the root
    tinyxml2::XMLElement* sdf = doc.RootElement();
    if (!sdf) {
        std::cerr << "Cannot find sdf element." << std::endl;
        return flowerPotLocations;
    }

    // Get the first child "world"
    tinyxml2::XMLElement* world = sdf->FirstChildElement("world");
    if (!world) {
        std::cerr << "Cannot find world element." << std::endl;
        return flowerPotLocations;
    }

    // Get the second child "state"
    tinyxml2::XMLElement* state = world->FirstChildElement("state");
    if (!state) {
        std::cerr << "Cannot find state element." << std::endl;
        return flowerPotLocations;
    }


    // Get all "model" elements inside it
    tinyxml2::XMLElement* model = state->FirstChildElement("model");
    while (model) {
        const char* name = model->Attribute("name");
        if (name && std::string(name).find("Cole_Hardware") != std::string::npos) {
            tinyxml2::XMLElement* pose = model->FirstChildElement("pose");
            if (pose) {
                std::string poseStr = pose->GetText();

                //flowerPotLocations.emplace_back(poseStr.at(0), poseStr.at(1));
                
                std::istringstream iss(poseStr);
                float x, y, z, roll, pitch, yaw;
                if (iss >> x >> y >> z >> roll >> pitch >> yaw) {
                    flowerPotLocations.emplace_back(x, y);
                }
            }
        }
        model = model->NextSiblingElement("model");
    }

    if (flowerPotLocations.empty()) {
        std::cout << "No flower pots found in the file." << std::endl;
    }

    return flowerPotLocations;
}

#endif