#include "CustomUKF.h"
void
CustomUKF::UKFwithICP() {
	const int Nsamples = END_FRAME - START_FRAME + 1;
	
	// AWGN
	std::default_random_engine generator;
	std::normal_distribution<double> distribution(0.0, 0.5);
	/*for (int k = 0; k < this->DataSize; k++) {
		double noise = distribution(generator);
		
		this->POSE_LOG_GroundTruth << this->GPS_Table[k][1] << "	" << this->GPS_Table[k][2]<< "	" << this->GPS_Table[k][3] << endl;
		this->GPS_Table[k][1] = this->GPS_Table[k][1] + noise;
		this->GPS_Table[k][2] = this->GPS_Table[k][2] + noise;
		
		this->POSE_LOG_Measurement << this->GPS_Table[k][1] << "	" << this->GPS_Table[k][2] << "	" << this->GPS_Table[k][3] << endl;
	}*/
	
	//pcl::visualization::CloudViewer Viewer("UKF with ICP");
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZI>);
	// UKF with ICP
	//for (int k = 0; k < Nsamples; k = k + STEP) {
	//	int k_ = k + START_FRAME - 1;
	//	double posX(0), posY(0), heading(0);
	//	CustomUKF::UKFwithICPmodule(k_, posX, posY, heading, cloud_map);
	//	//this->POSE_LOG_UKF << posX << "	" << posY << "	" << heading << endl;
	//	Viewer.showCloud(cloud_map);
	//}
	
	// UKF
	for (int k = 0; k < Nsamples; k = k + STEP) {
		int k_ = k + START_FRAME;
		double posX(0), posY(0), heading(0);
		CustomUKF::UKFmodule(k_, posX, posY, heading);
		this->POSE_LOG_UKF << k_ << "	" << -posX << "	" << posY << "	" << this->GPS_Table[k_][7] << "	" << this->GPS_Table[k_][3] << "	" << this->GPS_Table[k_][8] << "	" << this->GPS_Table[k_][9] << endl;
		//Viewer.showCloud(cloud_map);
	}

	//this->firstRun = true;
	//// Simple Dead Reckoning
	//double posX(0), posY(0), heading(0);
	//for (int k = 0; k < Nsamples; k = k + STEP) {
	//	int k_ = k + START_FRAME - 1;		
	//	CustomUKF::SimpleDR(k_, posX, posY, heading);
	//	this->DeadReckoning << k_ << "	" << -posY << "	" << -posX << "	" << this->GPS_Table[k_][7] << "	" << this->GPS_Table[k_][3] << "	" << this->GPS_Table[k_][8] << "	" << this->GPS_Table[k_][9] << endl;
	//}

}

void 
CustomUKF::SimpleDR(int loop, double &posX, double &posY, double &heading) {
	pcl::console::TicToc time;
	time.tic();
	if (this->firstRun == true) {		
		x << this->GPS_Table[loop][1], this->GPS_Table[loop][2], this->GPS_Table[loop][3];
		//posX = x(0); posY = x(1); heading = x(2);
		firstRun = false;
	}
	else if (this->firstRun == false) {
		x(0) = posX; x(1) = posY; x(2) = heading;
	}
	Eigen::VectorXd xp(NumberofState);
	fx(loop, x, xp);
	posX = xp(0); posY = xp(1); heading = xp(2);
}

void 
CustomUKF::UKFmodule(int loop, double &posX, double &posY, double &heading) {
	pcl::console::TicToc time;
	time.tic();
	if (this->firstRun == true) {
	
		R << 0.05, 0, 0,
			0, 0.05, 0,
			0, 0, 0.08;
		
		Q << 0.35, 0, 0,
			0, 0.35, 0,
			0, 0, 0.35;
		
		P << 1, 0, 0,
			0, 1, 0,
			0, 0, 1;

		x << this->GPS_Table[loop][1], this->GPS_Table[loop][2], this->GPS_Table[loop][3];
		firstRun = false;

	}

	Eigen::MatrixXd Xi(NumberofState, 2 * NumberofState + 1);
	Eigen::VectorXd W(2 * NumberofState + 1);
	Xi = Eigen::MatrixXd::Zero(NumberofState, 2 * NumberofState + 1);
	W = Eigen::VectorXd::Zero(2 * NumberofState + 1);
	// 시그마포인트 뽑기
	
	CustomUKF::SigmaPoints(x, P, kappa, Xi, W);
	//cout << "Xi = " << endl;
	//cout << Xi << endl;
	
	// 상태공간 예측하기
	Eigen::MatrixXd fXi(NumberofState, 2 * NumberofState + 1);
	fXi = Eigen::MatrixXd::Zero(NumberofState, 2 * NumberofState + 1);
	for (int k = 0; k < 2 * NumberofState + 1; k++) {
		
		Eigen::VectorXd xp_(NumberofState);		
		Eigen::VectorXd Xi_temp(NumberofState);
		Xi_temp(0) = Xi(0, k);	Xi_temp(1) = Xi(1, k);	Xi_temp(2) = Xi(2, k);
		CustomUKF::fx(loop, Xi_temp, xp_);
		fXi(0, k) = xp_(0);	fXi(1, k) = xp_(1);	fXi(2, k) = xp_(2);
	}
	//cout << "fXi = " << endl;
	//cout << fXi << endl;
	
	Eigen::VectorXd xp(NumberofState);
	Eigen::MatrixXd Pp(NumberofState, NumberofState);
	CustomUKF::UnscentedTransform(fXi, W, this->Q, xp, Pp);
	/*cout << "xp = " << endl;
	cout << xp << endl;
	cout << "Pp = " << endl;
	cout << Pp << endl;
	*/
	// 측정값 예측하기
	Eigen::MatrixXd hXi(NumberofMeasurement, 2 * NumberofState + 1);
	hXi = Eigen::MatrixXd::Zero(NumberofMeasurement, 2 * NumberofState + 1);
	for (int k = 0; k < 2 * NumberofState + 1; k++) {		
		Eigen::VectorXd yp(NumberofMeasurement);
		Eigen::VectorXd Xi_temp(NumberofState);
		Xi_temp(0) = Xi(0, k);	Xi_temp(1) = Xi(1, k);	Xi_temp(2) = Xi(2, k);
		CustomUKF::hx(Xi_temp, yp);
		hXi(0, k) = yp(0);	hXi(1, k) = yp(1);	hXi(2, k) = yp(2);
	}
	
	Eigen::VectorXd zp(NumberofMeasurement);
	Eigen::MatrixXd Pz(NumberofMeasurement, NumberofMeasurement);
	CustomUKF::UnscentedTransform(hXi, W, this->R, zp, Pz);
	
	// 칼만 게인 구하기
	Eigen::MatrixXd Pxz(NumberofState, NumberofMeasurement);
	Pxz = Eigen::MatrixXd::Zero(NumberofState, NumberofMeasurement);
	for (int k = 0; k < 2 * NumberofState + 1; k++) {
		Eigen::VectorXd tempfXi(NumberofState);
		tempfXi(0) = fXi(0, k); tempfXi(1) = fXi(1, k); tempfXi(2) = fXi(2, k);
		Eigen::VectorXd temphXi(NumberofMeasurement);
		temphXi(0) = hXi(0, k); temphXi(1) = hXi(1, k); temphXi(2) = hXi(2, k);
		
		Pxz = Pxz + W(k)*(tempfXi - xp)*(temphXi.transpose() - zp.transpose());
	}
	//cout << "Pxz = " << endl;
	//cout << Pxz << endl;
	
	Eigen::MatrixXd K(NumberofState, NumberofMeasurement);
	K = Pxz * Pz.inverse();
	
	Eigen::VectorXd z(NumberofMeasurement);
	z = Eigen::VectorXd::Zero(NumberofMeasurement);
	// TODO
	// z에 측정값 넣는 코드 추가
	z(0) = this->GPS_Table[loop][1];
	z(1) = this->GPS_Table[loop][2];
	z(2) = this->GPS_Table[loop][3];
	this->x = xp + K*(z - zp);
	this->P = Pp - K* Pz*K.transpose();

	posX = x(0);
	posY = x(1);
	heading = x(2);

	cout << "UKF + ICP 진행 중..." << loop << "/" << END_FRAME << "\t Computation time = " << time.toc() / 1000 << "초" << endl;
}


void 
CustomUKF::UnscentedTransform(Eigen::MatrixXd fXi, Eigen::VectorXd W, Eigen::MatrixXd Q, Eigen::VectorXd &xp, Eigen::MatrixXd &Pp) {
	Eigen::VectorXd xm(NumberofState);
	xm = Eigen::VectorXd::Zero(NumberofState);
	for (int k = 0; k < 2 * NumberofState + 1; k++) {
		Eigen::VectorXd tempXi(NumberofState);
		tempXi(0) = fXi(0, k); tempXi(1) = fXi(1, k); tempXi(2) = fXi(2, k);
		//cout << "W(k) : " << endl;
		//cout << W(k) << endl;
		xm = xm + W(k)*tempXi;
		//cout << "xm : " << endl;
		//cout << xm << endl;
	}
	xp = xm;
	Eigen::MatrixXd xcov(NumberofState, NumberofState);
	xcov = Eigen::MatrixXd::Zero(NumberofState, NumberofState);
	for (int k = 0; k < 2 * NumberofState + 1; k++) {
		Eigen::VectorXd tempXi(NumberofState);
		tempXi(0) = fXi(0, k); tempXi(1) = fXi(1, k); tempXi(2) = fXi(2, k);
		xcov = xcov + W(k)*(tempXi - xm)*(tempXi.transpose() - xm.transpose());
		//cout << "xcov : " << endl;
		//cout << xcov << endl;
	}

	Pp = xcov + Q;
}


void 
CustomUKF::SigmaPoints(Eigen::VectorXd xm, Eigen::MatrixXd P, int kappa, Eigen::MatrixXd &Xi, Eigen::VectorXd &W) {
	Xi(0, 0) = xm(0); Xi(1, 0) = xm(1);	Xi(2, 0) = xm(2);
	W(0) = kappa / (double)(NumberofState + kappa);

	Eigen::MatrixXd U(3, 3);
	Eigen::FullPivLU<Eigen::MatrixXd> lu(P);
	U = lu.matrixLU().triangularView<Eigen::Upper>();
	//cout << "upper" << U<<endl;
	for (int k = 0; k < NumberofState; k++) {
		Xi(0, k + 1) = xm(0) + U(k, 0);
		Xi(1, k + 1) = xm(1) + U(k, 1);
		Xi(2, k + 1) = xm(2) + U(k, 2);

		float tempw = (double)1 / (double)(2 * (NumberofState + kappa));
		//cout << "NumberofState : " << NumberofState << endl;
		//cout << "tempw : " << tempw << endl;
		W(k + 1) = tempw;
	}
	
	for (int k = 0; k < NumberofState; k++) {
		Xi(0, NumberofState + k + 1) = xm(0) - U(k, 0);
		Xi(1, NumberofState + k + 1) = xm(1) - U(k, 1);
		Xi(2, NumberofState + k + 1) = xm(2) - U(k, 2);
		float tempw = (double)1 / (double)(2 * (NumberofState + kappa));
		W(NumberofState + k + 1) = tempw;
	}
}

void
CustomUKF::fx(int loop, Eigen::VectorXd x, Eigen::VectorXd &xp) {
	xp(0) = x(0) + dt*GPS_Table[loop][4] * cos(x(2));
	xp(1) = x(1) + dt*GPS_Table[loop][5] * sin(x(2));
	xp(2) = x(2) + dt*GPS_Table[loop][6];
}

void 
CustomUKF::hx(Eigen::VectorXd x, Eigen::VectorXd &yp) {
	yp(0) = x(0);
	yp(1) = x(1);
	yp(2) = x(2);
}



void
CustomUKF::globalAlign(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source, Eigen::VectorXd x, int loop) {
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transform(new pcl::PointCloud<pcl::PointXYZI>);
	
	if (this->DefaultLoop == true) {	 // 맨 처음 야외구간
		
		CustomUKF::getTransformations(x, loop);
		pcl::transformPointCloud(*cloud_source, *cloud_transform, this->TransformPose);
		cloud_source->clear();
		for each(auto it in *cloud_transform) {
			cloud_map->push_back(it);
			//cloud_source->push_back(it);
		}


		pcl::PointCloud<pcl::PointXYZI> temp;
		pcl::copyPointCloud(*cloud_transform, temp);
		this->TargetFrames.push_back(temp);
		this->BackStepPose = this->TransformPose;

		double tempx(0), tempy(0), tempz(0), tempyaw(0), temproll(0), temppitch(0);
		CustomUKF::getInverseTransformation(this->TransformPose, tempx, tempy, tempz, temproll, temppitch, tempyaw);
		this->POSE_LOG_UKFwithICP << tempx << "	" << tempy <<  "	" << tempyaw << endl;
		Eigen::VectorXd Tempx(3);
		Tempx << tempx, tempy, tempyaw;
		this->x = Tempx;
	}

	else if (this->DefaultLoop == false) {

		
		CustomUKF::getTransformations(x, loop);
		pcl::transformPointCloud(*cloud_source, *cloud_transform, this->TransformPose);
		
	    CustomUKF::ICPWithNormals(cloud_transform, loop, this->TransformPose);
		//_getch();
		double tempx(0), tempy(0), tempz(0), tempyaw(0), temproll(0), temppitch(0);
		CustomUKF::getInverseTransformation(this->TransformPose, tempx, tempy, tempz, temproll, temppitch, tempyaw);
		tempyaw = this->x(2);
		Eigen::VectorXd Tempx(3);
		Tempx << tempx, tempy, tempyaw;
	//	cout << "this->x = " << endl;
		//cout << this->x << endl;

		this->x = ALPHA*this->x + BETA*Tempx;
		//cout << "함" << endl;
		//this->POSE_LOG_UKFwithICP << loop << "	" << tempx << "	" << tempy << "	" << tempz << "	" << tempyaw << "	" << temproll << "	" << temppitch << endl;
		this->POSE_LOG_UKFwithICP << this->x.transpose() << endl;
		
		
		/********** 타겟 프레임 업데이트 **********/
		CustomUKF::getTransformations(this->x, loop);
		cloud_transform->clear();
		//this->BackStepPose = this->TransformPose;
		pcl::transformPointCloud(*cloud_source, *cloud_transform, this->TransformPose);
		
		cloud_map->clear();
		pcl::PointCloud<pcl::PointXYZI> temp;
		pcl::copyPointCloud(*cloud_transform, temp);
		this->TargetFrames.push_back(temp);
		int NumofFrames = this->TargetFrames.size();
		if (NumofFrames > 2){
			this->TargetFrames.erase(this->TargetFrames.begin());
		}
		for (int i = 0; i < this->TargetFrames.size(); i++) {
			for each(auto points in TargetFrames[i]){
				cloud_map->push_back(points);
			}
		}
		
		
		/*cout << "x : " << endl;
		cout << x << endl;
		cout << "this->x" << endl;
		cout << this->x << endl;*/
		//_getch();
	}
	cloud_transform->clear();
}


void
CustomUKF::ICPWithNormals(pcl::PointCloud<pcl::PointXYZI>::Ptr sourceCloudPtr, int loop, Eigen::Matrix4f &LocalMat){

	pcl::PointCloud<pcl::PointXYZI>::Ptr temptarget(new pcl::PointCloud<pcl::PointXYZI>);

	for (int i = 0; i < this->TargetFrames.size(); i++) {
		for each(auto points in TargetFrames[i]){
			temptarget->push_back(points);
		}
	}
	cout << "this->TargetFrames.size() : " << this->TargetFrames.size() << endl;
	cout << "temptarget.size() : " << temptarget->size() << endl;
	cout << "sourceCloudPtr.size() : " << sourceCloudPtr->size() << endl;
	//_getch();


	Eigen::Matrix4f mat;
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr points_with_normals_src(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr points_with_normals_tgt(new pcl::PointCloud<pcl::PointXYZINormal>);

	pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> norm_est;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(KDTREE_SEARCH_NORM);
	//cout << "sourceCloudPtr->size()" << sourceCloudPtr->size() << endl;
	norm_est.setInputCloud(sourceCloudPtr);
	norm_est.compute(*points_with_normals_src);
	pcl::copyPointCloud(*sourceCloudPtr, *points_with_normals_src);

	norm_est.setInputCloud(temptarget);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*temptarget, *points_with_normals_tgt);

	// Instantiate our custom point representation (defined above) ...
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[5] = { 1.0, 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);

	// Align
	pcl::IterativeClosestPointNonLinear<pcl::PointXYZINormal, pcl::PointXYZINormal> reg;
	// Set the point representation
	reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));

	int cnt = 0;
	double epsilon = ICP_EPS_FROM;
	unsigned int iteration = ICP_ITR_FROM;
	for (double distance = ICP_DIST_FROM; distance >= ICP_DIST_TO; distance -= ICP_DIST_UNIT){
		reg.setInputTarget(points_with_normals_tgt);
		reg.setInputSource(points_with_normals_src);
		reg.setMaximumIterations(iteration);
		reg.setTransformationEpsilon(epsilon);
		reg.setMaxCorrespondenceDistance(distance);
		reg.align(*points_with_normals_src);
		if (!reg.hasConverged()) {
			cout << "매칭 FAIL! Coresponding distance 늘리자" << endl;

			cnt++;
			distance = ICP_DIST_FROM + 0.5 * cnt;
			iteration = ICP_ITR_FROM;
			epsilon = ICP_EPS_FROM;
		}

		LocalMat = reg.getFinalTransformation() * LocalMat;

		epsilon -= ICP_EPS_UNIT;
		iteration += ICP_ITR_UNIT;
	}

	cout << "스캔매칭 결과 mat : \n" << LocalMat << endl;
	/********** 맵 쌓기 **********/

	this->tempLastPose = LocalMat;

	cout << "GPS Data Processing...end!" << endl;
	cout << endl;
}

void
CustomUKF::calcGPS(){
	
	for (int i = 0; i < this->DataSize; i++) {
		long double lat, lngd, alt;
		long double radius;
		long double x, y, z;

		lat = this->GPS_Table[i][1];
		lngd = this->GPS_Table[i][2];
		alt = this->GPS_Table[i][3];

		long double roll = drad * this->GPS_Table[i][6];
		long double pitch = drad * this->GPS_Table[i][5];
		long double yaw = -drad * this->GPS_Table[i][4];

		radius = (EARTH_RADIUS_MAJOR * EARTH_RADIUS_MAJOR) / std::sqrt(EARTH_RADIUS_MAJOR * EARTH_RADIUS_MAJOR * std::cos(lat*drad) * std::cos(lat*drad) + EARTH_RADIUS_MAJOR * EARTH_RADIUS_MAJOR * std::sin(lat*drad) * std::sin(lat*drad)) + alt;

		this->GPS_Table[i][1] = radius * (lngd - this->INIT_LON)*drad *cos(this->INIT_LAT*drad);  // dx 데이터
		this->GPS_Table[i][2] = radius * ((lat - this->INIT_LAT)*drad);					  // dy 데이터
		this->GPS_Table[i][3] = alt - this->INIT_ALT;										  // dz 데이터
		//this->GPS_Table[3][i] = 0;										  // dz 데이터
		this->GPS_Table[i][4] = yaw;
		this->GPS_Table[i][5] = pitch;
		this->GPS_Table[i][6] = roll;		
	}	
}

void
CustomUKF::getTransformations(Eigen::VectorXd x, int loop) {
	//cout << "글로벌 좌표계 변환 행렬 구하기" << endl;

	//for (int i = 0; i < this->DataSize; i++) {
		Eigen::Matrix4f matrix;
		double cos_yaw = std::cos(x(2));
		double sin_yaw = std::sin(x(2));
		/*double cos_pitch = std::cos(this->GPS_Table[i][5]);
		double sin_pitch = std::sin(this->GPS_Table[i][5]);
		double cos_roll = std::cos(this->GPS_Table[i][6]);
		double sin_roll = std::sin(this->GPS_Table[i][6]);
		*/
		double cos_pitch = std::cos(0);
		double sin_pitch = std::sin(0);
		double cos_roll = std::cos(0);
		double sin_roll = std::sin(0);

		double comb_a = cos_yaw * sin_pitch;
		double comb_b = sin_yaw * sin_pitch;
		matrix(0, 0) = cos_yaw * cos_pitch;
		matrix(0, 1) = comb_a * sin_roll - sin_yaw * cos_roll;
		matrix(0, 2) = comb_a * cos_roll + sin_yaw * sin_roll;
		matrix(1, 0) = sin_yaw * cos_pitch;
		matrix(1, 1) = comb_b * sin_roll + cos_yaw * cos_roll;
		matrix(1, 2) = comb_b * cos_roll - cos_yaw * sin_roll;
		matrix(2, 0) = -sin_pitch;
		matrix(2, 1) = cos_pitch * sin_roll;
		matrix(2, 2) = cos_pitch * cos_roll;
		matrix(3, 0) = matrix(3, 1) = matrix(3, 2) = 0;
		matrix(3, 3) = 1;

		matrix(0, 3) = x(0);
		matrix(1, 3) = x(1);
		//matrix(2, 3) = this->GPS_Table[i][3];
		matrix(2, 3) = 0;
		/*POSE_DATA tempPose;
		tempPose.frame = loop;
		tempPose.mat = matrix;
		this->GPS_TFM.push_back(tempPose);*/
		this->TransformPose = matrix;
	//}

}

void
CustomUKF::getInverseTransformation(Eigen::Matrix4f matrix, double &x, double &y, double &z, double &roll, double &pitch, double &yaw){
	pitch = -std::asin(matrix(2, 0));
	double cos_pitch = std::cos(yaw);
	roll = std::atan2(matrix(2, 1) / cos_pitch, matrix(2, 2) / cos_pitch);
	yaw = std::atan2(matrix(1, 0) / cos_pitch, matrix(0, 0) / cos_pitch);
	x = matrix(0, 3);
	y = matrix(1, 3);
	z = matrix(2, 3);
}
inline string
CustomUKF::PathSetting(string PATH, int num) {
	string path = PATH;
	string number;
	int num1(num);
	stringstream sst;
	sst << num1;
	sst >> number;
	string path1 = path + number + "_lidar.pcd";
	return path1;
}


void
CustomUKF::UKFwithICPmodule(int loop, double &posX, double &posY, double &heading, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map) {
	cout << "UKF start?" << endl;
	pcl::console::TicToc time;
	time.tic();
	if (this->firstRun == true) {
		/*this->x(NumberofState);
		this->P(NumberofState, NumberofState);
		this->Q(NumberofState, NumberofState);
		this->R(NumberofMeasurement, NumberofMeasurement);*/
		ALPHA = 0.4; // UKF
		BETA = 0.6;  // ICP
		R << 0.65, 0, 0,
			0, 0.65, 0,
			0, 0, 0.08;

		Q << 2.55, 0, 0,
			0, 2.55, 0,
			0, 0, 0.05;

		P << 1, 0, 0,
			0, 1, 0,
			0, 0, 1;

		x << this->GPS_Table[loop][1], this->GPS_Table[loop][2], this->GPS_Table[loop][3];
		firstRun = false;

	}

	Eigen::MatrixXd Xi(NumberofState, 2 * NumberofState + 1);
	Eigen::VectorXd W(2 * NumberofState + 1);
	Xi = Eigen::MatrixXd::Zero(NumberofState, 2 * NumberofState + 1);
	W = Eigen::VectorXd::Zero(2 * NumberofState + 1);
	// 시그마포인트 뽑기

	CustomUKF::SigmaPoints(x, P, kappa, Xi, W);
	//cout << "Xi = " << endl;
	//cout << Xi << endl;

	// 상태공간 예측하기
	Eigen::MatrixXd fXi(NumberofState, 2 * NumberofState + 1);
	fXi = Eigen::MatrixXd::Zero(NumberofState, 2 * NumberofState + 1);
	for (int k = 0; k < 2 * NumberofState + 1; k++) {
		Eigen::VectorXd xp_(NumberofState);
		Eigen::VectorXd Xi_temp(NumberofState);
		Xi_temp(0) = Xi(0, k);	Xi_temp(1) = Xi(1, k);	Xi_temp(2) = Xi(2, k);
		CustomUKF::fx(loop, Xi_temp, xp_);
		fXi(0, k) = xp_(0);	fXi(1, k) = xp_(1);	fXi(2, k) = xp_(2);
	}
	//cout << "fXi = " << endl;
	//cout << fXi << endl;

	Eigen::VectorXd xp(NumberofState);
	Eigen::MatrixXd Pp(NumberofState, NumberofState);
	CustomUKF::UnscentedTransform(fXi, W, this->Q, xp, Pp);
	/*cout << "xp = " << endl;
	cout << xp << endl;
	cout << "Pp = " << endl;
	cout << Pp << endl;
	*/
	// 측정값 예측하기
	Eigen::MatrixXd hXi(NumberofMeasurement, 2 * NumberofState + 1);
	hXi = Eigen::MatrixXd::Zero(NumberofMeasurement, 2 * NumberofState + 1);
	for (int k = 0; k < 2 * NumberofState + 1; k++) {
		Eigen::VectorXd yp(NumberofMeasurement);
		Eigen::VectorXd Xi_temp(NumberofState);
		Xi_temp(0) = Xi(0, k);	Xi_temp(1) = Xi(1, k);	Xi_temp(2) = Xi(2, k);
		CustomUKF::hx(Xi_temp, yp);
		hXi(0, k) = yp(0);	hXi(1, k) = yp(1);	hXi(2, k) = yp(2);
	}

	Eigen::VectorXd zp(NumberofMeasurement);
	Eigen::MatrixXd Pz(NumberofMeasurement, NumberofMeasurement);
	CustomUKF::UnscentedTransform(hXi, W, this->R, zp, Pz);

	// 칼만 게인 구하기
	Eigen::MatrixXd Pxz(NumberofState, NumberofMeasurement);
	Pxz = Eigen::MatrixXd::Zero(NumberofState, NumberofMeasurement);
	for (int k = 0; k < 2 * NumberofState + 1; k++) {
		Eigen::VectorXd tempfXi(NumberofState);
		tempfXi(0) = fXi(0, k); tempfXi(1) = fXi(1, k); tempfXi(2) = fXi(2, k);
		Eigen::VectorXd temphXi(NumberofMeasurement);
		temphXi(0) = hXi(0, k); temphXi(1) = hXi(1, k); temphXi(2) = hXi(2, k);

		Pxz = Pxz + W(k)*(tempfXi - xp)*(temphXi.transpose() - zp.transpose());
	}
	//cout << "Pxz = " << endl;
	//cout << Pxz << endl;

	Eigen::MatrixXd K(NumberofState, NumberofMeasurement);
	K = Pxz * Pz.inverse();

	Eigen::VectorXd z(NumberofMeasurement);
	z = Eigen::VectorXd::Zero(NumberofMeasurement);
	// TODO
	// z에 측정값 넣는 코드 추가
	z(0) = this->GPS_Table[loop][1];
	z(1) = this->GPS_Table[loop][2];
	z(2) = this->GPS_Table[loop][3];
	//cout << xp + K*(z - zp) << endl;
	//_getch();
	Eigen::VectorXd tempvec(3);
	tempvec = Eigen::VectorXd::Zero(3);
	tempvec = xp + K*(z - zp);
	//cout << tempvec << endl;
	x(0) = tempvec(0);
	x(1) = tempvec(1);
	x(2) = tempvec(2);
	P = Pp - K* Pz*K.transpose();
	//cout << "x"<< x.transpose() << endl;
	//cout << "x.size()" << x.size() << endl;
	//cout << x(0) << "	" << x(1) << "	" << x(3) << endl;
	//this->POSE_LOG_UKF << x(0) << "	" << x(1) << "	" << x(3) << endl;
	this->POSE_LOG_UKF << x.transpose() << endl;

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downSample_Source(new pcl::PointCloud<pcl::PointXYZI>);
	string path = CustomUKF::PathSetting(GLOBAL_PATH, loop);
	pcl::io::loadPCDFile(path, *cloud_source);
	pcl::VoxelGrid<pcl::PointXYZI> vg_Source;
	vg_Source.setInputCloud(cloud_source);
	vg_Source.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
	vg_Source.filter(*cloud_downSample_Source);

	CustomUKF::globalAlign(cloud_map, cloud_downSample_Source, x, loop);
	this->DefaultLoop = false;

	posX = x(0);
	posY = x(1);
	heading = x(2);
	cout << "UKF + ICP 진행 중..." << loop << "/" << END_FRAME << "\t Computation time = " << time.toc() / 1000 << "초" << endl;
}