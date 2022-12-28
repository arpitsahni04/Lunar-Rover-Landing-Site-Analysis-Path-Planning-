#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>



struct RGB_Map {
	Eigen::MatrixXd visR, visG, visB;
};

struct GridPoint {
	int x;
	int y;
};

struct Coord {
	double lat;
	double lon;
};

class Map {
public:
	Map(){};
	// default constructor for the class. Initializes member variables only.
	int numRows;
	Eigen::MatrixXd LOSpercent;
	Eigen::MatrixXd slopeMap;
	RGB_Map img;
	int totalmaxLOSpoints;
	Map(Eigen::MatrixXd LOSpercent) { 
		this->LOSpercent = LOSpercent;  
		numRows = LOSpercent.rows();
	}

	double getLOSpercent(int i, int j) { return LOSpercent(i, j); }

	double getTotalLOSpercent() {
		for (int i = 0; i < numRows; i++) {
			for (int j = 0; j < numRows; j++) {
				if (getLOSpercent(i, j) == 100)
					totalmaxLOSpoints += 1;
			}
		}

	}

	GridPoint landSite;
	std::vector<GridPoint> vantagePoints;
	std::vector<GridPoint> final_path_astar;
	double distToLander;
	double percentVantagePoints= vantagePoints.size()*100/ 36;
	double lengthVantagePath = 0;

};