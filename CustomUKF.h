#ifndef _CUSTUM_UKF_H_
#define _CUSTUM_UKF_H_

#include "Parameter.h"
// Standard 
#include <iostream>
#include <fstream>
#include <sstream>
#include <conio.h>
#include <vector>
#include <random>
// Eigen
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen\src\Core\EigenBase.h>

// PCL
#include <pcl/io/pcd_io.h>

#include <pcl\visualization\cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl\console\time.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/make_shared.hpp>
//#include <boost\shared_ptr.hpp>
#include <pcl/registration/icp_nl.h>

using namespace std;

struct POSE_DATA {
	unsigned int frame;
	Eigen::Matrix4f mat;
	POSE_DATA() {
		mat.setIdentity();
	}
};

class CustomUKF{
private:

	bool firstRun;
	bool DefaultLoop;
	int NumberofState;
	int NumberofMeasurement;
	int kappa;

	double ALPHA, BETA;

	Eigen::Matrix3d Q, R, P;
	Eigen::Vector3d x;

	// for ICP
	vector<pcl::PointCloud<pcl::PointXYZI>> TargetFrames;
	Eigen::Matrix4f BackStepPose;	// Last transformation of each map building step
	Eigen::Matrix4f GlobalTrans;	// Global transformation matrix
	Eigen::Matrix4f tempLastPose;
	Eigen::Matrix4f TransformPose;
	vector<POSE_DATA> GPS_TFM;				// GPS transformation matrix vector
	double **GPS_Table;						// GPS pose table
	int DataSize;			// Total data size
	double INIT_LAT;  // 위도의 초기값. 맵의 원점
	double INIT_LON;	// 고도의 초기값. 맵의 원점
	double INIT_ALT;	// 위도의 초기값. 맵의 원점

	ofstream POSE_LOG_GroundTruth;	
	ofstream POSE_LOG_Measurement;
	ofstream POSE_LOG_UKF;
	ofstream POSE_LOG_UKFwithICP;
	ofstream DeadReckoning;

public:
	CustomUKF(double **table, int datasize) : GPS_Table(table), DataSize(datasize), firstRun(true), DefaultLoop(true)
	{
		cout << "Custom UKF module is getting started" << endl;
		NumberofState = 3;
		NumberofMeasurement = 3;
		
		kappa = 3 - NumberofState;
		POSE_LOG_GroundTruth.open("../data/POSE_LOG_GroundTruth.txt", ios_base::out | ios_base::trunc);
		POSE_LOG_Measurement.open("../data/RT_POSE_LOG.txt", ios_base::out | ios_base::trunc);
		POSE_LOG_UKF.open("../data/UKF_POSE_LOG.txt", ios_base::out | ios_base::trunc);
		POSE_LOG_UKFwithICP.open("../data/POSE_LOG_UKFwithICP.txt", ios_base::out | ios_base::trunc);
		DeadReckoning.open("../data/DR_POSE_LOG.txt", ios_base::out | ios_base::trunc);
		
	}
	~CustomUKF() {}

	void UnscentedTransform(Eigen::MatrixXd fXi, Eigen::VectorXd W, Eigen::MatrixXd Q, Eigen::VectorXd &xp, Eigen::MatrixXd &Pp);	 // Nonlinear fuction의 평균과 분산을 구한다.
	void SigmaPoints(Eigen::VectorXd xm, Eigen::MatrixXd P, int kappa, Eigen::MatrixXd &Xi, Eigen::VectorXd &W);
	void UKFmodule(int loop, double &posX, double &posY, double &heading);
	void UKFwithICPmodule(int loop, double &posX, double &posY, double &heading, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map);
	void UKFwithICP();
		
	void fx(int loop, Eigen::VectorXd x, Eigen::VectorXd &xp);
	void hx(Eigen::VectorXd x, Eigen::VectorXd &hp);

	void SimpleDR(int loop, double &posX, double &posY, double &heading);
	// For ICP

	void globalAlign(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source, Eigen::VectorXd x, int loop); // Initial align for Exterior map building step	
	void ICPWithNormals(pcl::PointCloud<pcl::PointXYZI>::Ptr sourceCloudPtr, int loop, Eigen::Matrix4f &LocalMat);

	void calcGPS();
	void getTransformations(Eigen::VectorXd x, int loop);
	void getInverseTransformation(Eigen::Matrix4f matrix, double &x, double &y, double &z, double &roll, double &pitch, double &yaw);
	inline string PathSetting(string PATH, int num);
};


class MyPointRepresentation : public pcl::PointRepresentation < pcl::PointXYZINormal >
{
	using pcl::PointRepresentation<pcl::PointXYZINormal>::nr_dimensions_;
public:
	MyPointRepresentation()
	{
		// Define the number of dimensions
		nr_dimensions_ = 5;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray(const pcl::PointXYZINormal &p, float * out) const
	{
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.intensity;
		out[4] = p.curvature;
	}
};
#endif _CUSTUM_UKF_H_