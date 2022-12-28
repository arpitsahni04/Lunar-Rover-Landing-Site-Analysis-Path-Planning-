#pragma once
#include "Map.h"
#include <vector>


struct MyMaps {
	// for wrapping map and score into 1 object
	Map AMap;
	double score;
	MyMaps(Map Map, double s) : AMap(Map), score(s) {
	}
};


struct less_than_key
{
	// lambda funtion for for sorting 
	inline bool operator() (const MyMaps& Map1, const MyMaps& Map2)
	{
		return (Map1.score < Map2.score);
	}
};

class AutomaticsiteEvaluation {

public:
	double w1, w2, w3, w4; // weights for computing hueristics
	std::vector<MyMaps> AllMaps;
	// Takes in the weights and vector of Map objects and inits a MyMaps Vector 
	// where each element is a Map and a score
	AutomaticsiteEvaluation(std::vector<Map > Maps, double k1, double k2, double k3,
		double k4) {
		// packs the score and Map in a vector of type MyMaps
		for (int i = 0; i < Maps.size(); i++) {

			double score_i = ComputeScore(Maps[i]);

			AllMaps.push_back(MyMaps(Maps[i], score_i));
		}
		// weights for the various parameters used to evaluate landing site
		w1 = k1;
		w2 = k2;
		w3 = k3;
		w4 = k4;
	};

	//  Sorts to return the Top site Candidates
	void Evaluatelandingsite() {

		// sorts AllMaps vector based on there score
		std::sort(AllMaps.begin(), AllMaps.end(), less_than_key());


	};

	// Computes Evalution Hueristic
	double ComputeScore(Map AMap) {


		// To compute the closest vantagepoint to Lander based on Eucledian distance

			//*min_element(disttoAllVantage.begin(),disttoAllVantage.end());

		// calculate a weighted score 
		double score_temp = AMap.totalmaxLOSpoints * w1 + AMap.distToLander * w2 +
			AMap.percentVantagePoints * w3 + AMap.lengthVantagePath * w4;

		return  score_temp;
	};

};

	