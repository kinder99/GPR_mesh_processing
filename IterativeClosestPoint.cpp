#include <iostream>
#include <algorithm>
#include "IterativeClosestPoint.h"
#include <Eigen/Dense>
#include <Eigen/LU>
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>


void IterativeClosestPoint::setClouds(PointCloud *pointCloud1, PointCloud *pointCloud2)
{
	cloud1 = pointCloud1;
	cloud2 = pointCloud2;
	knn.setPoints(&(cloud1->getPoints()));
}

// This method should mark the border points in cloud 1. It also changes their color (for example to red).
// You will need to add an attribute to this class that stores this property for all points in cloud 1. 
// maxDeltaAlpha is supposed to be between 90 and 180 degrees apparently
// compute border points and store them locally, plus change colors of border points using PointCloud::getColors()
void IterativeClosestPoint::markBorderPoints()
{
	// TODO
	std::cout << "marking border points" << std::endl;
	std::vector<size_t> neighbors;
	std::vector<float> dists;
	std::vector<glm::vec3>& points = (cloud1->getPoints());
	//initialize values of border vector
	for (int i = 0; i < points.size(); i++){
		isBorder.push_back(false);
	}
	int counter = 0;
	for(glm::vec3 point : points){
		neighbors.clear();
		dists.clear();
		//get nearest neighbors of current point	
		knn.getKNearestNeighbors(point,10,neighbors,dists);

		//PCA
		glm::vec3 sum = glm::vec3(0,0,0);
		for (int i = 0; i<neighbors.size(); i++){
			sum += points.at(neighbors.at(i));
		}
		auto const count = static_cast<float>(neighbors.size());
		glm::vec3 p_centroid = sum/count; //compute centroid of current point subset
		Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero();
		for (int i = 0; i<neighbors.size(); i++){ //for all neighbors of current point
			glm::vec3 p = points.at(neighbors.at(i));
			p = p - p_centroid; //translate all points by the centroid coordinates 
			Eigen::Vector3f p_eigen = Eigen::Vector3f(p.x,p.y,p.z);
			covariance_matrix += p_eigen * p_eigen.transpose(); //compute covariance matrix
		}
		//compute spectral decomposition
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covariance_matrix);
		Eigen::Matrix3f eigenvecs = eigensolver.eigenvectors();

		//transform the neighbors to the PCA computed local frame, then discard z
		std::vector<float> neighbor_angles;
		for (int i = 1; i < neighbors.size(); i++){
			glm::vec3 current_neighbor = points.at(neighbors.at(i));
			Eigen::Vector3f substract = Eigen::Vector3f(current_neighbor.x - point.x, current_neighbor.y - point.y, current_neighbor.z - point.z);
			current_neighbor.x = eigenvecs.col(2).dot(substract);
			current_neighbor.y = eigenvecs.col(1).dot(substract);
			current_neighbor.z = 0;//discard Z component to project on xy plane
			float angle = atan2(current_neighbor.y, current_neighbor.x) * (180/M_PI); //get angle value in degrees
			neighbor_angles.push_back(angle);
		}

		neighbors.erase(neighbors.begin() + 0); //erase current point from neighbors

		//sort angles then look for maxDeltaAlpha (the largest)
		std::sort(neighbor_angles.begin(), neighbor_angles.end());
		float maxDeltaAlpha = 0;
		float difference = 360 - (neighbor_angles.back() - neighbor_angles.front());
		difference > maxDeltaAlpha ? maxDeltaAlpha = difference : maxDeltaAlpha = maxDeltaAlpha;
		for(int i = 0; i < neighbor_angles.size()-1; i++){
			difference = neighbor_angles.at(i+1) - neighbor_angles.at(i);
			difference > maxDeltaAlpha ? maxDeltaAlpha = difference : maxDeltaAlpha = maxDeltaAlpha; // accumulate maxDeltaAlpha difference value
		}
		maxDeltaAlpha > 90 ? isBorder.at(counter) = true : isBorder.at(counter) = false; //check value of maxDeltaAlpha angle to set border point status
		counter++;
	}
	for(int i = 0; i < points.size(); i++){
		if(isBorder.at(i) == true){
			(cloud1->getColors()).at(i) = glm::vec4(1,0,0,1); //set color as red if point is a border point
		}
	}
}


// This method should compute the closest point in cloud 1 for all non border points in cloud 2. 
// This correspondence will be useful to compute the ICP step matrix that will get cloud 2 closer to cloud 1.
// Store the correspondence in this class as the following method is going to need it.
// As it is evident in its signature this method also returns the correspondence. The application draws this if available.
// store correspondances into vector of int, if border point assign value -1
vector<int> *IterativeClosestPoint::computeCorrespondence()
{
	// TODO
	
	return NULL;
}


// This method should compute the rotation and translation of an ICP step from the correspondence
// information between clouds 1 and 2. Both should be encoded in the returned 4x4 matrix.
// To do this use the SVD algorithm in Eigen. (JacobiSVD)
// can ignore step 5 from instructions
// return homogeneous matrix, should be autonomous ???
glm::mat4 IterativeClosestPoint::computeICPStep()
{
	// TODO
	markBorderPoints();
	return glm::mat4(1.f);
}


// This method should perform the whole ICP algorithm with as many steps as needed.
// It should stop when maxSteps are performed, when the Frobenius norm of the transformation matrix of
// a step is smaller than a small threshold, or when the correspondence does not change from the 
// previous step.
// use second method for the stopping condition
vector<int> *IterativeClosestPoint::computeFullICP(unsigned int maxSteps)
{
	// TODO
	
	return NULL;
}





