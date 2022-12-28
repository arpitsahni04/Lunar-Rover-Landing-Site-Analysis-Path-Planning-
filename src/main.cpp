#include "happly.h"
#include <embree3/rtcore.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
//#include <fmt/format.h>
//#include <fmt/ostream.h>
#include <filesystem>
#include <vector>
#include <string>
#include <random>
#include <numeric>
#include <memory>
#include <cassert>
#include <iostream>
#include <sstream>
#include <string>
#include <queue>
#include <local_to_latlon.h>
#include <Map.h>
#include "A_star.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "a_Star_algo.h"
#include "AutomateEvalutate.h"

//constants from RFP
const double CIRCLE_RAD = 50.0; //radius around landing site (m)
const double MAX_SLOPE_DEG = 10.0; //maximum slope allowed within landing site area
//const float PI = 3.141529;

const float MAP_CELL_PITCH = 1.00; // meters
const float ROVER_HEIGHT = 1.00; // meters
const float LANDER_HEIGHT = 3.00; // meters

const double CIRCLE_STEP_SIZE = 20.0; //step size inside a landing circle (m)

const double SEARCH_RAD = 400.0; //radius around crater where program will search for landing sites
GridPoint PIT_CENTER = { -1,-1 }; //pit center (unitless, in grid)

std::vector<Map> MapList; //list of maps as result of running gridSearch

struct TerrainMesh {
	Eigen::MatrixXd      vertices;
	Eigen::MatrixXd vertexNormals;
	Eigen::MatrixXi   faceIndices;
};



struct Color {
	int R, G, B;
};

TerrainMesh loadTerrainMesh(const std::string& plyfile) {
	TerrainMesh tmesh;

	happly::PLYData plyIn(plyfile);
	std::vector<std::array<double, 3>> vertices = plyIn.getVertexPositions();
	std::vector<std::vector<size_t>> faceIndices = plyIn.getFaceIndices<size_t>();
	std::vector<float> nx = plyIn.getElement("vertex").getProperty<float>("nx");
	std::vector<float> ny = plyIn.getElement("vertex").getProperty<float>("ny");
	std::vector<float> nz = plyIn.getElement("vertex").getProperty<float>("nz");

	assert(vertices.size() == nx.size());
	assert(vertices.size() == ny.size());
	assert(vertices.size() == nz.size());

	tmesh.vertices.resize(3, vertices.size());
	tmesh.vertexNormals.resize(3, nx.size());
	tmesh.faceIndices.resize(3, faceIndices.size());

	for (int i = 0; i < vertices.size(); ++i) {
		tmesh.vertices(0, i) = vertices[i][0];
		tmesh.vertices(1, i) = vertices[i][1];
		tmesh.vertices(2, i) = vertices[i][2];

		tmesh.vertexNormals(0, i) = nx[i];
		tmesh.vertexNormals(1, i) = ny[i];
		tmesh.vertexNormals(2, i) = nz[i];
	}

	for (int i = 0; i < faceIndices.size(); ++i) {
		tmesh.faceIndices(0, i) = faceIndices[i][0];
		tmesh.faceIndices(1, i) = faceIndices[i][1];
		tmesh.faceIndices(2, i) = faceIndices[i][2];
	}
	return tmesh;
}

void savePointCloud(const std::string& plyfile, const Eigen::MatrixXd& cloud) {
	happly::PLYData plyOut;
	std::vector<std::array<double, 3>> meshVertexPositions(cloud.cols());
	for (int i = 0; i < cloud.cols(); ++i) {
		meshVertexPositions[i][0] = cloud(0, i);
		meshVertexPositions[i][1] = cloud(1, i);
		meshVertexPositions[i][2] = cloud(2, i);
	}
	plyOut.addVertexPositions(meshVertexPositions);
	plyOut.write(plyfile, happly::DataFormat::Binary);
}

void savePNG(const std::string& filename, RGB_Map& img) {
	int rows = img.visR.rows();
	int cols = img.visR.cols();

	unsigned char* data = (unsigned char*)malloc(3 * rows * cols * sizeof(unsigned char));

	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			data[3 * i * cols + 3 * j + 0] = img.visR(i, j);
			data[3 * i * cols + 3 * j + 1] = img.visG(i, j);
			data[3 * i * cols + 3 * j + 2] = img.visB(i, j);
		}
	}
	int ret = stbi_write_png(filename.c_str(), cols, rows, 3, data, 3 * sizeof(data[0]) * cols);
	free(data);
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd>
buildTerrainMaps(const TerrainMesh& tmesh, double pitch) {
	using namespace Eigen;

	VectorXd aobb_min = tmesh.vertices.rowwise().minCoeff();
	VectorXd aobb_max = tmesh.vertices.rowwise().maxCoeff();
	VectorXi map_dims = ((aobb_max - aobb_min) / pitch).cast<int>();

	double Z_HEIGHT = aobb_max[2] + 10.0;

	int numVerts = tmesh.vertices.cols();
	int numFaces = tmesh.faceIndices.cols();

	RTCDevice device = rtcNewDevice(NULL);
	RTCScene  scene = rtcNewScene(device);
	RTCGeometry geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
	float* vb = (float*)rtcSetNewGeometryBuffer(geometry, RTC_BUFFER_TYPE_VERTEX,
		0, RTC_FORMAT_FLOAT3,
		3 * sizeof(float), numVerts);
	for (int i = 0; i < numVerts; ++i) {
		vb[3 * i + 0] = (float)tmesh.vertices(0, i);
		vb[3 * i + 1] = (float)tmesh.vertices(1, i);
		vb[3 * i + 2] = (float)tmesh.vertices(2, i);
	}
	unsigned* ib = (unsigned*)rtcSetNewGeometryBuffer(geometry, RTC_BUFFER_TYPE_INDEX,
		0, RTC_FORMAT_UINT3,
		3 * sizeof(unsigned), numFaces);
	for (int i = 0; i < numFaces; ++i) {
		ib[3 * i + 0] = tmesh.faceIndices(0, i);
		ib[3 * i + 1] = tmesh.faceIndices(1, i);
		ib[3 * i + 2] = tmesh.faceIndices(2, i);
	}

	rtcCommitGeometry(geometry);
	rtcAttachGeometry(scene, geometry);
	rtcReleaseGeometry(geometry);
	rtcCommitScene(scene);

	MatrixXd  elevationMap(map_dims[1], map_dims[0]);
	MatrixXd      slopeMap(map_dims[1], map_dims[0]);
	elevationMap.fill(Z_HEIGHT);
	slopeMap.fill(0.0);

	for (int i = 0; i < elevationMap.rows(); ++i) {
		//fmt::print("Building Terrain Maps: {}/{}\n", i, elevationMap.rows());
		std::cout << "Building Terrain Maps:" << "current row in map: " << i << "total num rows: " << elevationMap.rows() << std::endl;
#pragma omp parallel for
		for (int j = 0; j < elevationMap.cols(); ++j) {
			RTCRayHit rayhit;
			rayhit.ray.org_x = aobb_min[0] + j * pitch;
			rayhit.ray.org_y = aobb_min[1] + (elevationMap.rows() - i) * pitch;
			rayhit.ray.org_z = Z_HEIGHT;
			rayhit.ray.dir_x = 0.0f;
			rayhit.ray.dir_y = 0.0f;
			rayhit.ray.dir_z = -1.0f;
			rayhit.ray.tnear = 0.0f;
			rayhit.ray.tfar = std::numeric_limits<float>::infinity();
			rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

			RTCIntersectContext context;
			rtcInitIntersectContext(&context);
			rtcIntersect1(scene, &context, &rayhit);

			if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
				elevationMap(i, j) = Z_HEIGHT - rayhit.ray.tfar;
				float nx = rayhit.hit.Ng_x;
				float ny = rayhit.hit.Ng_y;
				float nz = rayhit.hit.Ng_z;
				float nn = std::sqrt(nx * nx + ny * ny + nz * nz);
				slopeMap(i, j) = std::acos(nz / nn) * 180 / PI; //remmeber
			}
		}
	}

	rtcReleaseScene(scene);
	rtcReleaseDevice(device);

	return std::make_tuple(elevationMap, slopeMap);
}

void addSlopeColormap(const Eigen::MatrixXd& slopeMap, RGB_Map& img)
{
	double sum_x = 0, sum_y = 0, count = 0;
	// Use the slope map to color code the result
			// image in shades of red/orange/yellow.
	for (int i = 0; i < slopeMap.rows(); ++i) {
		for (int j = 0; j < slopeMap.cols(); ++j) {
			double s = slopeMap(i, j);
			if (s <= 5) {
				img.visR(i, j) = 63.0;
				img.visG(i, j) = 11.0;
				img.visB(i, j) = 27.0;
			}
			if (5 <= s && s < 10) {
				img.visR(i, j) = 122.0;
				img.visG(i, j) = 22.0;
				img.visB(i, j) = 49.0;
			}
			if (10 <= s && s < 15) {
				img.visR(i, j) = 207.0;
				img.visG(i, j) = 66.0;
				img.visB(i, j) = 60.0;
			}
			if (15 <= s && s < 20) {
				img.visR(i, j) = 252.0;
				img.visG(i, j) = 125.0;
				img.visB(i, j) = 73.0;
			}
			if (20 <= s) {
				img.visR(i, j) = 255.0;
				img.visG(i, j) = 212.0;
				img.visB(i, j) = 98.0;

				sum_x = sum_x + i;
				sum_y = sum_y + j;
				count++;
			}
		}
	}
	PIT_CENTER.x = (int)(sum_x / count);
	PIT_CENTER.y = (int)(sum_y / count);
	std::cout << "PIT COORD: " << PIT_CENTER.x << ", " << PIT_CENTER.y << std::endl;
}

void drawPoint(int li, int lj, double radius, RGB_Map& img, Color clr = { 255, 255, 255 })
{
	if (255 < clr.R || clr.R < 0); clr.R = 255;
	if (255 < clr.G || clr.G < 0); clr.G = 255;
	if (255 < clr.B || clr.B < 0); clr.B = 255;

	// Loop over a square centered on the landing site.
	for (int i = -radius / MAP_CELL_PITCH; i <= radius / MAP_CELL_PITCH; ++i) {
		for (int j = -radius / MAP_CELL_PITCH; j <= radius / MAP_CELL_PITCH; ++j) {
			// Skip pixels that are out of bounds.
			if (li + i >= img.visR.rows() ||
				li + i < 0 || lj + j < 0 ||
				lj + j >= img.visR.cols()) {
				continue;
			}

			// Draw a black circle around the landing site.
			if (std::sqrt(i * i + j * j) < radius / MAP_CELL_PITCH) {
				img.visR(li + i, lj + j) = clr.R;
				img.visG(li + i, lj + j) = clr.G;
				img.visB(li + i, lj + j) = clr.B;
			}

		}
	}
}

void addVantagePoints(const Eigen::MatrixXd slopeMap, const Eigen::MatrixXd LOSpercent, Map& map_obj, RGB_Map& img)
{
	int x, y;
	int final_x = PIT_CENTER.x, final_y = PIT_CENTER.y;
	int vantage_count = 0;
	//std::cout << slopeMap(final_x, final_y)<<std::endl;
	for (int a = 0; a < 360; a += 10)
	{
		bool found_new_point = false;
		// y-mx-c=0
		x = PIT_CENTER.x;
		y = PIT_CENTER.y;
		int radial_dist = 1;
		bool pointFound = false;
		while (!pointFound && radial_dist < 300) {
			//if (radial_dist > 300) { pointFound = true; }
			x = PIT_CENTER.x + (int)(cos(a * PI / 180) * radial_dist);
			y = PIT_CENTER.y + (int)(sin(a * PI / 180) * radial_dist);

			if (slopeMap(x, y) <= 15.0 && LOSpercent(x, y) >= 75) {
				final_x = x;
				final_y = y;
				found_new_point = true;
				pointFound = true;
			}
			radial_dist++;
		}
		if (found_new_point) {
			vantage_count++;
		}

		drawPoint(final_x, final_y, 5.0, img, { 12, 255, 255 });
		map_obj.vantagePoints.push_back({ final_x,final_y });
	}
	std::cout << "Vantage_Points Percentage:" << (double)vantage_count * 100 / 36 << std::endl;
}

void drawLandingSite(const Eigen::MatrixXd& slopeMap, RGB_Map& img, int li, int lj)
{
	// Loop over a square centered on the landing site.
	for (int i = -CIRCLE_RAD / MAP_CELL_PITCH; i <= CIRCLE_RAD / MAP_CELL_PITCH; ++i) {
		for (int j = -CIRCLE_RAD / MAP_CELL_PITCH; j <= CIRCLE_RAD / MAP_CELL_PITCH; ++j) {
			// Skip pixels that are out of bounds.
			if (li + i >= img.visR.rows() ||
				li + i < 0 || lj + j < 0 ||
				lj + j >= img.visR.cols()) {
				continue;
			}

			// Draw a blue circle around the landing site.
			if (std::sqrt(i * i + j * j) < CIRCLE_RAD / MAP_CELL_PITCH) {
				double alpha = 0.6;
				img.visR(li + i, lj + j) = std::clamp((1 - alpha) * img.visR(li + i, lj + j) + alpha * 0, 0.0, 255.0);
				img.visG(li + i, lj + j) = std::clamp((1 - alpha) * img.visG(li + i, lj + j) + alpha * 0, 0.0, 255.0);
				img.visB(li + i, lj + j) = std::clamp((1 - alpha) * img.visR(li + i, lj + j) + alpha * 255, 0.0, 255.0);
			}

			// Draw a white dot at the center of the landing site.
			if (std::sqrt(i * i + j * j) < 4.0 / MAP_CELL_PITCH) {
				img.visR(li + i, lj + j) = 255.0;
				img.visG(li + i, lj + j) = 255.0;
				img.visB(li + i, lj + j) = 255.0;
			}
		}
	}
}

void addLOScolormap(const Eigen::MatrixXd& LOSpercent, RGB_Map& img)
{
	double alpha = 0.4;
	// Use the slope map to color code the result
	// image in shades of green at the rover sites
	for (int i = 0; i < LOSpercent.rows(); ++i) {
		for (int j = 0; j < LOSpercent.cols(); ++j) {
			double p = LOSpercent(i, j);
			if (0 < p && p < 25) {
				img.visR(i, j) = std::clamp((1 - alpha) * img.visR(i, j) + alpha * 0, 0.0, 255.0);
				img.visG(i, j) = std::clamp((1 - alpha) * img.visG(i, j) + alpha * 51, 0.0, 255.0);
				img.visB(i, j) = std::clamp((1 - alpha) * img.visR(i, j) + alpha * 0, 0.0, 255.0);
			}
			if (25 < p && p < 50) {
				img.visR(i, j) = std::clamp((1 - alpha) * img.visR(i, j) + alpha * 0, 0.0, 255.0);
				img.visG(i, j) = std::clamp((1 - alpha) * img.visG(i, j) + alpha * 102, 0.0, 255.0);
				img.visB(i, j) = std::clamp((1 - alpha) * img.visR(i, j) + alpha * 0, 0.0, 255.0);
			}
			if (50 < p && p < 75) {
				img.visR(i, j) = std::clamp((1 - alpha) * img.visR(i, j) + alpha * 0, 0.0, 255.0);
				img.visG(i, j) = std::clamp((1 - alpha) * img.visG(i, j) + alpha * 153, 0.0, 255.0);
				img.visB(i, j) = std::clamp((1 - alpha) * img.visR(i, j) + alpha * 0, 0.0, 255.0);
			}
			if (75 < p && p < 100) {
				img.visR(i, j) = std::clamp((1 - alpha) * img.visR(i, j) + alpha * 0, 0.0, 255.0);
				img.visG(i, j) = std::clamp((1 - alpha) * img.visG(i, j) + alpha * 204, 0.0, 255.0);
				img.visB(i, j) = std::clamp((1 - alpha) * img.visR(i, j) + alpha * 0, 0.0, 255.0);
			}
			if (p == 100) {
				img.visR(i, j) = std::clamp((1 - alpha) * img.visR(i, j) + alpha * 0, 0.0, 255.0);
				img.visG(i, j) = std::clamp((1 - alpha) * img.visG(i, j) + alpha * 255, 0.0, 255.0);
				img.visB(i, j) = std::clamp((1 - alpha) * img.visR(i, j) + alpha * 0, 0.0, 255.0);
			}
		}
	}
}

void getLOSpercent(Eigen::MatrixXd& LOSpercent,
	const TerrainMesh& tmesh,
	const Eigen::MatrixXd& elevationMap,
	const Eigen::MatrixXd& slopeMap,
	int li, int lj) {
	using namespace Eigen;

	double pitch = MAP_CELL_PITCH;

	//MatrixXd LOSpercent(elevationMap.rows(), elevationMap.cols());

	VectorXd aobb_min = tmesh.vertices.rowwise().minCoeff();
	VectorXd aobb_max = tmesh.vertices.rowwise().maxCoeff();
	VectorXi map_dims = ((aobb_max - aobb_min) / pitch).cast<int>();

	int numVerts = tmesh.vertices.cols();
	int numFaces = tmesh.faceIndices.cols();

	RTCDevice device = rtcNewDevice(NULL);
	RTCScene  scene = rtcNewScene(device);
	RTCGeometry geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
	float* vb = (float*)rtcSetNewGeometryBuffer(geometry, RTC_BUFFER_TYPE_VERTEX,
		0, RTC_FORMAT_FLOAT3,
		3 * sizeof(float), numVerts);
	for (int i = 0; i < numVerts; ++i) {
		vb[3 * i + 0] = (float)tmesh.vertices(0, i);
		vb[3 * i + 1] = (float)tmesh.vertices(1, i);
		vb[3 * i + 2] = (float)tmesh.vertices(2, i);
	}
	unsigned* ib = (unsigned*)rtcSetNewGeometryBuffer(geometry, RTC_BUFFER_TYPE_INDEX,
		0, RTC_FORMAT_UINT3,
		3 * sizeof(unsigned), numFaces);
	for (int i = 0; i < numFaces; ++i) {
		ib[3 * i + 0] = tmesh.faceIndices(0, i);
		ib[3 * i + 1] = tmesh.faceIndices(1, i);
		ib[3 * i + 2] = tmesh.faceIndices(2, i);
	}

	rtcCommitGeometry(geometry);
	rtcAttachGeometry(scene, geometry);
	rtcReleaseGeometry(geometry);
	rtcCommitScene(scene);

	// Compute lander position in 3D from the (li, lj) map coordinates.
	Vector3d lander_pos;
	lander_pos[0] = aobb_min[0] + lj * pitch;
	lander_pos[1] = aobb_min[1] + (elevationMap.rows() - li) * pitch;
	lander_pos[2] = elevationMap(li, lj) + LANDER_HEIGHT;

	//fmt::print("Evaluating Site: {} {}\n", li, lj);
	std::cout << "    Evaulating Position:" << li << ", " << lj << std::endl;

	// Loop over the entire map. At each site on the map, place a rover.
	// Shoot a ray from the lander antenna to the rover antenna.
	// If the ray is not obstructed, blend green into the map color.
	for (int ri = 0; ri < elevationMap.rows(); ++ri) {
		for (int rj = 0; rj < elevationMap.cols(); ++rj) {
			// Compute rover position in 3D from (ri, rj) map coordinates.
			Vector3d rover_pos;
			rover_pos[0] = aobb_min[0] + rj * pitch;
			rover_pos[1] = aobb_min[1] + (elevationMap.rows() - ri) * pitch;
			rover_pos[2] = elevationMap(ri, rj) + ROVER_HEIGHT;

			// Compute the normalized direction of
			// a ray from the lander to the rover.
			Vector3d ray_dir = (rover_pos - lander_pos);
			double lander_to_rover_dist = ray_dir.norm();
			ray_dir /= lander_to_rover_dist;

			// Construct the inputs for the embree raytracing function.
			RTCRayHit rayhit;
			rayhit.ray.org_x = lander_pos[0];
			rayhit.ray.org_y = lander_pos[1];
			rayhit.ray.org_z = lander_pos[2];
			rayhit.ray.dir_x = ray_dir[0];
			rayhit.ray.dir_y = ray_dir[1];
			rayhit.ray.dir_z = ray_dir[2];
			rayhit.ray.tnear = 0.0f;
			rayhit.ray.tfar = std::numeric_limits<float>::infinity();
			rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

			// Ask embree to raytrace for you.
			RTCIntersectContext context;
			rtcInitIntersectContext(&context);
			rtcIntersect1(scene, &context, &rayhit);

			// found_hit is true if the ray hit the mesh somewhere.
			bool found_hit = rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID;

			// This is the distance from the lander to the hit.
			double hitdist = rayhit.ray.tfar;

			// If no hit was found, line-of-sight comms is possible.
			// If a hit was found, but the hit is farther away from
			// the lander than the rover, line-of-sight is still possible.
			if (!found_hit ||
				(found_hit && hitdist > lander_to_rover_dist))
			{
				LOSpercent(ri, rj) += 1.0;
			}
		}
	}
}

double evaluate_distance(int aRow, int aCol, GridPoint end_point) {
	// Manhattan Distance
	double mh_dist = (abs(end_point.x - aRow) + abs(end_point.y - aCol));
	return abs(mh_dist * mh_dist);
}


void gridSearchTargeted(const TerrainMesh& tmesh,
	const Eigen::MatrixXd& elevationMap,
	const Eigen::MatrixXd& slopeMap,
	GridPoint landPoint) {
	using namespace Eigen;

	MatrixXd LOSpercent(elevationMap.rows(), elevationMap.cols());

	MatrixXd R(elevationMap.rows(), elevationMap.cols());
	MatrixXd G(elevationMap.rows(), elevationMap.cols());
	MatrixXd B(elevationMap.rows(), elevationMap.cols());
	RGB_Map img = { R,G,B };

	// Use the slope map to color code the result
			// image in shades of red/orange/yellow.
	addSlopeColormap(slopeMap, img);

	int li = landPoint.x;
	int lj = landPoint.y;

	// Evaluate slopes inside the circle centered on this landing site.
			// If any slope is too steep, skip this landing site and move on to the next one.
	double max_slope = 0;
	int count = 0;
	for (int i = -CIRCLE_RAD / MAP_CELL_PITCH; i <= CIRCLE_RAD / MAP_CELL_PITCH; ++i) {
		for (int j = -CIRCLE_RAD / MAP_CELL_PITCH; j <= CIRCLE_RAD / MAP_CELL_PITCH; ++j) {
			if (std::sqrt(i * i + j * j) > CIRCLE_RAD / MAP_CELL_PITCH) { continue; }
			if (li + i < 0 || li + i >= slopeMap.rows() ||
				lj + j < 0 || lj + j >= slopeMap.cols()) {
				continue;
			}
			max_slope = std::max(max_slope, slopeMap(li + i, lj + j));
			count++;
		}
	}
	if (max_slope > MAX_SLOPE_DEG)
		std::cout << "THERE WAS A SLOPE OVER THE MAX ALLOWED SLOPE FOUND IN LANDING CIRCLE" << std::endl;

	double lat, lon;
	grid_to_latlon(li, lj, elevationMap.rows(), lat, lon);
	std::cout << "Evaulating Site (lat, lon): " << lat << ", " << lon << std::endl;
	int numLandCirclePnts = 0;
	int radiusGrid = (int)(CIRCLE_RAD / MAP_CELL_PITCH);
	int gridStep = (int)(CIRCLE_STEP_SIZE / MAP_CELL_PITCH);

	// Loop over points inside landing circle,
	// determining percent of line of site
	for (int ci = -radiusGrid; ci <= radiusGrid; ci += gridStep) {
		//#pragma omp parallel for // This line tells OpenMP to parallelize this loop.
		for (int cj = -radiusGrid; cj <= radiusGrid; cj += gridStep) {
			// Skip pixels that are out of bounds.
			if (li + ci >= img.visR.rows() ||
				li + ci < 0 || lj + cj < 0 ||
				lj + cj >= img.visR.cols()) {
				continue;
			}

			// Only process points that are inside landing circle
			if (std::sqrt(ci * ci + cj * cj) < radiusGrid) {

				numLandCirclePnts++;

				// Draw a white dot at each point inside landing circle that was checked
				img.visR(li + ci, lj + cj) = 255.0;
				img.visG(li + ci, lj + cj) = 255.0;
				img.visB(li + ci, lj + cj) = 255.0;

				getLOSpercent(LOSpercent, tmesh, elevationMap, slopeMap, li + ci, lj + cj);
			}
		}
	}

	//average the LOS values over the number of points within landing circle to get the percentage 
	LOSpercent = LOSpercent / numLandCirclePnts * 100;

	//color map different shade of green according to percentage of rover points 
	// that are within LOS of the landing circle sampled points 
	addLOScolormap(LOSpercent, img);

	//create new map object with the calculated LOSpercent map
	::Map newMap(LOSpercent);
	newMap.landSite = landPoint;

	// We can overlay position and radius around the landing site onto the map.
	drawLandingSite(slopeMap, img, li, lj);
	drawPoint(PIT_CENTER.x, PIT_CENTER.y, 10.0, img, { 0, 0, 0 }); //draw point to mark center of pitt
	addVantagePoints(slopeMap, LOSpercent, newMap, img);

	double distance_to_lander;
	double min_distance = INFINITY;
	for (int i = 0; i < newMap.vantagePoints.size(); i++) {

		std::vector<GridPoint>final_path;
		std::vector<GridPoint> A_star_path;
		GridPoint start_point = { li, lj };
		GridPoint end_point = { newMap.vantagePoints.at(i).x, newMap.vantagePoints.at(i).y };
		//std::cout << "end_point->" << end_point.x << "-" << end_point.y << std::endl;


		distance_to_lander = evaluate_distance(li, lj, end_point);
		if (distance_to_lander < min_distance) {
			min_distance = distance_to_lander;
		}

		// A_Star.h  ---> does work but takes too much time
		//std::cout << "initialise_list-->" << i << std::endl;
		//std::list<point>path = path_manager(newMap, point(newMap.landSite.x, newMap.landSite.y),
		//	point(end_point.x, end_point.y), slopeMap, img);
		//std::list<point>::iterator it;
		//std::cout << path.size() << std::endl;
		//if (path.size() < 2) {
		//	for (it = path.begin(); it != path.end(); ++it) {
		//		drawPoint(it->x, it->y, 4.0, img);
		//	}
		//}



		// NEEDS FIXING THE A STAR DOES NOT WORK SO FOR NOW JUST USING THE DISTANCE
		/*a_Star_algo algo_util;
		final_path=algo_util.findShortestPathAstar(slopeMap, LOSpercent, start_point ,end_point );
		for (int i = 0; i<final_path.size(); i++) {
			drawPoint(final_path.at(i).x, final_path.at(i).y, 4.0, img);
		
		}*/

	}

	newMap.distToLander = min_distance;
	newMap.numRows = slopeMap.rows();
	newMap.img = img;

	//add new map to map list
	MapList.push_back(newMap);

	//save png of map
	std::stringstream fileName;
	fileName << "site_" << lat << "_" << lon << ".png";
	savePNG(fileName.str(), img);
	std::cout << "SAVED IMAGE!" << std::endl;
}

void gridSearchTargeted(const TerrainMesh& tmesh,
	const Eigen::MatrixXd& elevationMap,
	const Eigen::MatrixXd& slopeMap,
	Coord landSite) {

	double x, y;
	latlon_to_local(landSite.lat, landSite.lon, x, y);
	int li = elevationMap.rows() - (int)(y / MAP_CELL_PITCH);
	int lj = (int)(x / MAP_CELL_PITCH);
	GridPoint landPoint = { li, lj };

	gridSearchTargeted(tmesh, elevationMap, slopeMap, landPoint);
}

void gridSearch(const TerrainMesh& tmesh,
	const Eigen::MatrixXd& elevationMap,
	const Eigen::MatrixXd& slopeMap) {
	using namespace Eigen;

	double STEP_SIZE = CIRCLE_RAD * 1;
	int siteStepSize = (int)(STEP_SIZE / MAP_CELL_PITCH);
	int searchRadGrid = (int)(SEARCH_RAD / MAP_CELL_PITCH);

	int radiusGrid = (int)(CIRCLE_RAD / MAP_CELL_PITCH);
	int gridStep = (int)(CIRCLE_STEP_SIZE / MAP_CELL_PITCH);

	// Loop over a grid of possible landing sites on the map.
	for (int li = PIT_CENTER.x - searchRadGrid; li < PIT_CENTER.x + searchRadGrid; li += siteStepSize) {
#pragma omp parallel for // This line tells OpenMP to parallelize this loop.
		for (int lj = PIT_CENTER.y - searchRadGrid; lj < PIT_CENTER.y + searchRadGrid; lj += siteStepSize) {

			//skip all points outside the search radius
			if (std::sqrt((li - PIT_CENTER.x) * (li - PIT_CENTER.x)
				+ (lj - PIT_CENTER.y) * (lj - PIT_CENTER.y))
		> searchRadGrid) {
				continue;
			}

			GridPoint landSite = { li, lj };
			gridSearchTargeted(tmesh, elevationMap, slopeMap, landSite);
		}
	}
}

int main(int argc, char* argv[]) {
	//using namespace Eigen;

	// 1. Read .ply mesh file.
	auto terrainFile = (argc > 1) ? argv[1] : "C:/Users/mukul/source/repos/FanFeast/Engineering_Computation/meshes/lacus_mortis.ply";
	auto tmesh = loadTerrainMesh(terrainFile);

	// 2. Construct elevation and slope maps.
	auto [elevationMap, slopeMap] = buildTerrainMaps(tmesh, MAP_CELL_PITCH);

	// 3. Evaluate landing sites.
	// 
	// TO RUN ON MULTIPLE  LANDING SITES THROUGHOUT THE MAP
	// 
	//gridSearch(tmesh, elevationMap, slopeMap);

	//  TO RUN ON MULTIPLE TARGATED LANDING SITES
	std::cout << MapList.size() << std::endl;

	Coord landSite1 = { 44.961, 25.628 };
	gridSearchTargeted(tmesh, elevationMap, slopeMap, landSite1);
	Coord landSite2 = { 44.967, 25.629 };
	gridSearchTargeted(tmesh, elevationMap, slopeMap, landSite2);

	Coord landSite3 = { 44.956, 25.608 };
	gridSearchTargeted(tmesh, elevationMap, slopeMap, landSite3);

	// Evaluate
	//AutomaticsiteEvaluation Evaluator(MapList, 2, 2, 2, 2);
	//Evaluator.Evaluatelandingsite();

	//Map BestMap = *Evaluator.AllMaps[0].AMap;

	//double BestLandLat,BestLandLon;
	//grid_to_latlon(BestMap.landSite.x,BestMap.landSite.y, 
	//				BestMap.numRows,BestLandLat, BestLandLon);
	//Coord landsiteCoord = { BestLandLat,BestLandLon };



	//// Save File
	//std::stringstream fileName;
	//fileName << "Bestsite_" << BestLandLat << "_" << BestLandLon << ".png";
	//savePNG(fileName.str(), BestMap.img);
	//std::cout << "SAVED IMAGE!" << std::endl;
	


	// Evaluate
	AutomaticsiteEvaluation Evaluator(MapList, 2, 2, 2, 2);
	Evaluator.Evaluatelandingsite();

	Map BestMap = Evaluator.AllMaps.back().AMap;

	//double BestLandLat,BestLandLon;
	//grid_to_latlon(BestMap.landSite.x,BestMap.landSite.y, 
	//				BestMap.numRows,BestLandLat, BestLandLon);
	//Coord landsiteCoord = { BestLandLat,BestLandLon };



	// Save File
	std::stringstream fileName;
	fileName << "Bestsite.png";
	savePNG(fileName.str(), BestMap.img);
	std::cout << "SAVED BEST MAP IMAGE!" << std::endl;
	return 0;
}

