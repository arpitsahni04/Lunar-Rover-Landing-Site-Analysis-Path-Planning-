#include <list>
#include <algorithm>
#include <iostream>
#include "Map.h"

const float PI = 3.141529;
class point {
public:
	point(int a = 0, int b = 0) { x = a; y = b; }
	bool operator ==(const point& o) { return o.x == x && o.y == y; }
	point operator +(const point& o) { return point(o.x + x, o.y + y); }
	int x, y;
};


class node {
public:
	bool operator == (const node& o) { return pos == o.pos; }
	bool operator == (const point& o) { return pos == o; }
	bool operator < (const node& o) { return dist + cost < o.dist + o.cost; }
	point pos, parent;
	double cost, dist = -1;
};


class aStar {
public:
	aStar() {
		neighbours[0] = point(-1, -1); neighbours[1] = point(1, -1);
		neighbours[2] = point(-1, 1); neighbours[3] = point(1, 1);
		neighbours[4] = point(0, -1); neighbours[5] = point(-1, 0);
		neighbours[6] = point(0, 1); neighbours[7] = point(1, 0);
	}

	double calcDist(point& p, Eigen::MatrixXd slopeMap) {
		// need a better heuristic
		int x = end.x - p.x;
		int y = end.y - p.y;
		double temp_distance_variable = abs((double)(x * x + y * y)) * cos(slopeMap(x, y) * PI / 180.0) * cos(slopeMap(x, y) * PI / 180.0);
		return(temp_distance_variable* temp_distance_variable);
	}

	bool isValid(point& p, Eigen::MatrixXd slopeMap) {
		//return (p.x > 1 && p.y > 1 && p.x < theMap.numRows-1&& p.y < theMap.numRows-1 &&
		return(theMap.LOSpercent(p.x, p.y) > 1);//50 && slopeMap(p.x, p.y) < 20);
		//return true;
	}

	bool existPoint(point& p, double cost) {
		std::list<node>::iterator i;
		i = std::find(closed.begin(), closed.end(), p);
		if (i != closed.end()) {
			if ((*i).cost + (*i).dist < cost) return true;
			else { closed.erase(i); return false; }
		}
		i = std::find(open.begin(), open.end(), p);
		if (i != open.end()) {
			if ((*i).cost + (*i).dist < cost) return true;
			else { open.erase(i); return false; }
		}
		return false;
	}

	bool fillOpen(node& n, Eigen::MatrixXd slopeMap) {
		double LOScost,nc, dist;
		point neighbour;

		for (int x = 0; x < 8; x++) {
			// one can make diagonals have different cost
			LOScost = (1-theMap.LOSpercent(n.pos.x, n.pos.y)/100);
			neighbour = n.pos + neighbours[x];
			if (neighbour == end) return true;

			if (isValid(neighbour, slopeMap) && (theMap.LOSpercent(neighbour.x, neighbour.y) > 50 ? 1 : 0) !=0){
				nc = LOScost + n.cost;
				dist = calcDist(neighbour,slopeMap);
				if (!existPoint(neighbour, nc + dist)) {
					node node_map;
					node_map.cost = nc; node_map.dist = dist;
					node_map.pos = neighbour;
					node_map.parent = n.pos;
					open.push_back(node_map);
				}
			}
		}
		return false;
	}

	bool search(point& start_point, point& end_point, Map& mp, Eigen::MatrixXd slopeMap, RGB_Map img) {
		std::cout << "Entred_search" << std::endl;
		node n; end = end_point; start = start_point; theMap = mp;
		n.cost = 0; n.pos = start_point; n.parent = 0; n.dist = 0;
		n.dist = calcDist(start_point,slopeMap);
		open.push_back(n);
		while (!open.empty()) {
			//std::cout << "Entred_OPEN" << std::endl;
			node n = open.front();
			open.pop_front();
			//std::cout << open.size() << std::endl;
			closed.push_back(n);
			/*if (closed.size() % 100 == 0) {
				std::stringstream fileName;
				fileName << "A_Star_site_" << end_point.x << "_" << end_point.y << ".png";
				Utils utility;
				utility.SavePNG(fileName.str(), img);
				std::cout << "SAVED IMAGE!" << std::endl;
			}*/

			std::cout << closed.size() << std::endl;
			if (fillOpen(n, slopeMap)) 
				return true;
		}
		return false;
	}

	int path(std::list<point>& path) {
		path.push_front(end);
		double cost = 1 + closed.back().cost;
		path.push_front(closed.back().pos);
		point parent = closed.back().parent;

		for (std::list<node>::reverse_iterator i = closed.rbegin(); i != closed.rend(); i++) {
			if (i->pos == parent && !((*i).pos == start)) {
				path.push_front((*i).pos);
				parent = i->parent;
			}
		}
		path.push_front(start);
		return cost;
	}

	//map map;
	Map theMap;


	//point end, start;
	point end, start;

	//point neighbours[8];
	point neighbours[8];

	std::list<node> open;
	std::list<node> closed;
};

std::list<point> path_manager(Map& inputMap, point start_point, point end_point,Eigen::MatrixXd slopeMap, RGB_Map img) {
	aStar as;
	as.theMap = inputMap;
	as.start = start_point;
	as.end=end_point;

	if (as.search(start_point, end_point, as.theMap, slopeMap,img) ){
		std::list<point> path;
		double cost = as.path(path);
		std::cout << "\nPath cost " << cost << ": ";
		return path;
	}
	std::list<point>empty_path = { -1,-1};
	return empty_path;
}
