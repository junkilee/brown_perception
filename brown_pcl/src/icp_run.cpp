#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "bt_pc_segmentation/icpPointToPlane.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>

using namespace std;

void pca() {
  int npts = 10000;
  cv::Mat X3; X3.create(npts,3,CV_64F);
  // ... you put your 3D points in X3

  cv::PCA pca(X3,cv::Mat(),CV_PCA_DATA_AS_ROW);

  // collect mean, eigvec
  cv::Mat meanX,eigvecX;
  meanX = pca.mean;
  eigvecX = pca.eigenvectors;
}

int main (int argc, char** argv) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  cout << argv[1] << endl;

  string filename(argv[1]);

  cout << "filename : " << argv[1] << endl;

  if (pcl::io::loadOBJFile<pcl::PointXYZ> (filename, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  // define a 3 dim problem with 10000 model points
  // and 10000 template points:
  int32_t dim = 3;
  int32_t num = 10000;

  // allocate model and template memory
  double* M = (double*)calloc(3*num,sizeof(double));
  double* T = (double*)calloc(3*num,sizeof(double));

  // set model and template points
  cout << endl << "Creating model with 10000 points ..." << endl;
  cout << "Creating template by shifting model by (1,1,1) ..." << endl;
  int32_t k=0;
  for (double x=-2; x<2; x+=0.04) {
    for (double y=-2; y<2; y+=0.04) {
      double z=5*x*exp(-x*x-y*y);
      M[k*3+0] = x;
      M[k*3+1] = y;
      M[k*3+2] = z;
      T[k*3+0] = x-1;
      T[k*3+1] = y-1;
      T[k*3+2] = z-1;
      k++;
    }
  }

  // start with identity as initial transformation
  // in practice you might want to use some kind of prediction here
  Matrix R = Matrix::eye(3);
  Matrix t(3,1);

  // run point-to-plane ICP (-1 = no outlier threshold)
  cout << endl << "Running ICP (point-to-plane, no outliers)" << endl;
  IcpPointToPlane icp(M,num,dim);
  icp.fit(T,num,R,t,-1);

  // results
  cout << endl << "Transformation results:" << endl;
  cout << "R:" << endl << R << endl << endl;
  cout << "t:" << endl << t << endl << endl;

  // free memory
  free(M);
  free(T);

  // success
  return 0;
}