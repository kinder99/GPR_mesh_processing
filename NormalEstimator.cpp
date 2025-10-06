#include "NormalEstimator.h"
#include "NearestNeighbors.h"

// This method has to compute a normal per point in the 'points' vector and put it in the 
// 'normals' vector. The 'normals' vector already has the same size as the 'points' vector. 
// There is no need to push_back new elements, only to overwrite them ('normals[i] = ...;')
// In order to compute the normals you should use PCA. The provided 'NearestNeighbors' class
// wraps the nanoflann library that computes K-nearest neighbors effciently. 

void NormalEstimator::computePointCloudNormals(const vector<glm::vec3> &points, vector<glm::vec3> &normals)
{
	// TODO
	NearestNeighbors nn = NearestNeighbors();
	nn.setPoints(&points);

	//compute centroid of point cloud
	auto const count = static_cast<float>(points.size());
	
	//problem using reduce despite using c++17 and clang 15
	//glm::vec3 p_centroid = std::reduce(points.begin(), points.end())/count;
	
	glm::vec3 sum;
	std::vector<size_t> neighbors;
	std::vector<float> dists;
	int c = 0;
	for(glm::vec3 p : points){
		c++;
		nn.getKNearestNeighbors(p,8,neighbors,dists); //get neighbors of current point
		for (int i = 0; i<neighbors.size(); i++){
			sum += points.at(neighbors.at(i));
		}
		//std::cout << sum.x << ","<< sum.y << "," << sum.z << std::endl;
		glm::vec3 p_centroid = sum/count;
		//std::cout << "centroid : " << p_centroid.x << ","<< p_centroid.y << "," << p_centroid.z << std::endl;
		//std::cout << "centroid ok" << std::endl;

		//translate all points by the centroid coordinates 
		p = p - p_centroid;
		//std::cout << "translation ok" << std::endl;

		//compute covariance matrix
		Eigen::Matrix3f covariance_matrix;
		Eigen::Vector3f p_eigen = Eigen::Vector3f(p.x,p.y,p.z);
		covariance_matrix += p_eigen * p_eigen.transpose();
		//std::cout << covariance_matrix << std::endl;

		//compute spectral decomposition
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covariance_matrix);
		//std::cout << "spectral decomposition ok" << std::endl;

		//eigenvalues are sorted from lowest to highest here
		Eigen::Matrix3f eigenvecs = eigensolver.eigenvectors();
		Eigen::Vector3f eigenvals = eigensolver.eigenvalues();
		eigenvecs.rowwise().reverse(); //reverse sorting of eigen vectors to get descending order

		//define normals
		glm::vec3 temp = glm::vec3(eigenvecs.col(0)[0], eigenvecs.col(0)[1], eigenvecs.col(0)[2]);
		normals[c] = temp;
		std::cout << c << std::endl;
	}
	
}

bool NormalEstimator::comp(float a, float b){
	return a<b;
}