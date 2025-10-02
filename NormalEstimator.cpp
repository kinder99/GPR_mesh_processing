#include "NormalEstimator.h"
#include "NearestNeighbors.h"
#include <Eigen>
#include <numeric>


// This method has to compute a normal per point in the 'points' vector and put it in the 
// 'normals' vector. The 'normals' vector already has the same size as the 'points' vector. 
// There is no need to push_back new elements, only to overwrite them ('normals[i] = ...;')
// In order to compute the normals you should use PCA. The provided 'NearestNeighbors' class
// wraps the nanoflann library that computes K-nearest neighbors effciently. 

void NormalEstimator::computePointCloudNormals(const vector<glm::vec3> &points, vector<glm::vec3> &normals)
{
	// TODO

	//compute centroid of point cloud
	auto const count = static_cast<float>(points.size());
	glm::vec3 p_centroid = std::reduce(points.begin(), points.end())/count;

	//translate all points by the centroid coordinates 
	for (auto p : points){
		p = p - p_centroid;
	}

	//compute covariance matrix
	Eigen::Matrix3f covariance_matrix;
	for(float n = 0; n<count; n++){
		Eigen::Vector3f p_eigen = Eigen::Vector3f(points[n].x,points[n].y,points[n].z);
		covariance_matrix += p_eigen * p_eigen.transpose(); 
	}

	//compute spectral decomposition
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covariance_matrix);
	Eigen::Matrix3f eigenvecs = eigensolver.eigenvectors();

	Eigen::MatrixXd sorted_eigenvecs = sortMatrix(eigenvecs);
}

Eigen::MatrixXd sortMatrix(const Eigen::MatrixXd &original_matrix) {

  Eigen::MatrixXd sorted_matrix(original_matrix.rows(), original_matrix.cols());
  Eigen::VectorXd sorted_cols = original_matrix.row(0);
  Eigen::VectorXd::Index min_col;

  for (int i = 0; i < original_matrix.cols(); i++) {
    sorted_cols.minCoeff(&min_col);
    sorted_matrix.col(i) = original_matrix.col(min_col);
    sorted_cols(min_col) = std::numeric_limits<double>::max();
  }

   return sorted_matrix;
}