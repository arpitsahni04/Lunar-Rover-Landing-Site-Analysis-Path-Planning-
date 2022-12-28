#include <list>
#include <algorithm>
#include <iostream>
#include "Map.h"




class a_Star_algo{
public:
	int wallCost = 100;
	int locRow, locCol;
	static const int MAX_MAP_SIZE = 2000; // may need to be changed later on
	bool visited[MAX_MAP_SIZE][MAX_MAP_SIZE];
	GridPoint parent[MAX_MAP_SIZE][MAX_MAP_SIZE] = { -1,-1 };
	std::vector<GridPoint>final_path;

	void clearShortestPath()
	{
		for (int i = 1; i < MAX_MAP_SIZE; i++)
			for (int j = 1; j < MAX_MAP_SIZE; j++) {
				visited[i][j] = false;
				parent[i][j] = { -1,-1 };
			}
	}

	double heuristic(int aRow, int aCol, GridPoint end_point, const Eigen::MatrixXd& slopeMap) {
		// Manhattan Distance
		double mh_dist = (abs(end_point.x - aRow) + abs(end_point.y - aCol)) * cos(PI / 180 * slopeMap(aRow, aCol));
		return abs(mh_dist * mh_dist);
	}

	bool isNavigable(int row, int col, const Eigen::MatrixXd& LOSpercent)
	{
		// note order of conditions 
		// (I don't check map[row][col] until I am sure that row and col are valid)
		return (0 < row && row <= MAX_MAP_SIZE
			&& 0 < col && col <= MAX_MAP_SIZE && LOSpercent(row, col) >= 75
			/*&& !map[row][col]*/);
	}

	bool isObstacle(int row, int col, const Eigen::MatrixXd& slopeMap) {
		if (slopeMap(row, col) > 15)
			return true;
		else
			return false;
	};

	std::vector<GridPoint> findShortestPathAstar(const Eigen::MatrixXd slopeMap,
		const Eigen::MatrixXd LOSpercent, GridPoint start_point, GridPoint end_point)
	{
		initialise_astar(start_point.x,start_point.y);
		if (locRow > 0) {
			using namespace std;
			// parent array is a class member

			//bool visited[Maze::MAX_MAP_SIZE][Maze::MAX_MAP_SIZE]; // now a class variable so I can paint it
			GridPoint endCell = {};//myMaze->getEnd();
			GridPoint currCell;
			//queue<Cell> processQueue;

			struct Node { int row, col; double dist; };
			struct compareDistance {
				// return "true" if "n1" should be farther from root than "n2"
				bool operator()(Node const& n1, Node const& n2) {
					return n1.dist > n2.dist;
				}
			};
			priority_queue<Node, vector<Node>, compareDistance > processPriority;
			Node currNode;

			// for cycling through adjacent cells
			int rowAdjust[] = { -1, 0, 1, 0, 1, 1, -1, -1 };
			int colAdjust[] = { 0, 1, 0, -1, 1, -1, 1, -1 };
			int testRow, testCol;

			clearShortestPath();

			//Mark the root/start vertex as visited. // Not used in A* algorithm
			visited[locRow][locCol] = true;

			//Create a way to store the g_scores of all nodes as they are discovered 
			int gScore[MAX_MAP_SIZE][MAX_MAP_SIZE];

			//Create a way to check if a certain cell is already in the queue (heaps are not good for this)
			bool isInQueue[MAX_MAP_SIZE][MAX_MAP_SIZE];

			// Temporary variable for g_scores (used to determine if neighbor cell should be processed)
			int tentativeGscore;

			int nodesProcessed = 0; // not part of algorithm, but I'm interested to know.

			for (int i = 1; i < MAX_MAP_SIZE; i++)
				for (int j = 1; j < MAX_MAP_SIZE; j++) {
					isInQueue[i][j] = false;
					gScore[i][j] = INT_MAX;
				}

			//	Add the root vertex to the process queue
			processPriority.push({ locRow, locCol, heuristic(locRow, locCol, end_point, slopeMap) });
			isInQueue[locRow][locCol] = true;
			gScore[locRow][locCol] = 0;

			bool foundTheEnd = false;

			//	While the end vertex is yet to be discovered and there are vertices to process :
			//while (processQueue.size() > 0 && !foundTheEnd) {
			while (processPriority.size() > 0 && !foundTheEnd) {
				//Get the current vertex at the head of the queue (removing it from the queue)
				currNode = processPriority.top();
				processPriority.pop();  // remove from queue
				currCell = { currNode.row, currNode.col };
				isInQueue[currCell.x][currCell.y] = false; // keep track of Cells not in queue

				nodesProcessed++;

				//	If current vertex is the target vertex, we are done
				if (currCell.x == endCell.x && currCell.y == endCell.y)
					foundTheEnd = true;
				else {
					//	Else, for each of the adjacent vertices that have not been visited :

					for (int i = 0; i < 8; i++) { //neighbours -8
						testRow = currCell.x + rowAdjust[i];
						testCol = currCell.y + colAdjust[i];

						// check whether it is a valid neighbor
						if (/*!visited[testRow][testCol] && */isNavigable(testRow, testCol, LOSpercent)) {

							// calculate tentative g_score of neighbor
							if (isObstacle(testRow, testCol, slopeMap))
								tentativeGscore = gScore[currCell.x][currCell.y] + (wallCost - LOSpercent(testRow, testCol)); // if a wall going there is more costly

							else
								tentativeGscore = gScore[currCell.x][currCell.y] + 1;

							// process the neighbor if its g_score can be improved
							if (tentativeGscore < gScore[testRow][testCol]) {
								// store the better g_score (i.e., the path through currCell to neighbor is better)
								gScore[testRow][testCol] = tentativeGscore;

								//Set neighbor's parent to current cell
								parent[testRow][testCol] = currCell;

								//	Mark it as visited (just for show, not part of algorithm)
								visited[testRow][testCol] = true;

								//	Add neighbor to the queue, but only if it's not there already
								if (!isInQueue[testRow][testCol]) {
									processPriority.push({ testRow, testCol, (tentativeGscore
										+ heuristic(testRow, testCol, end_point, slopeMap)) });
									isInQueue[testRow][testCol] = true;
								}
							}
						}
					}
				}
			}
			if (parent[endCell.x][endCell.y].x == -1)
				cout << "Could not find a path " << endl;
		}

		for (int i = 0; i < MAX_MAP_SIZE; i++) {
			for (int j = 0; j < MAX_MAP_SIZE; j++) {
				if (visited[i][j] == 1) {
					final_path.push_back({ i,j });
				}
			}
		}
		return final_path;
	}


	void initialise_astar(int landingx, int landingy)
	{
		// mark everything as not traveled and not visited and not "parented"
		for (int i = 1; i < MAX_MAP_SIZE; i++) {
			for (int j = 1; j < MAX_MAP_SIZE; j++) {
				visited[i][j] = false;     // could call clearShortestPath() here, but since I
				parent[i][j] = { -1,-1 };  // already have a loop running  . . .
			}
		}
		locRow = landingx;
		locCol = landingy;
	}


};
