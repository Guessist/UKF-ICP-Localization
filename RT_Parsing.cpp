#include "RT_Parsing.h"
#include <stdlib.h>
#include <conio.h>

void
RT_Parsing::GetTotalTable(int &size, int &ref) {
	this->Filename = "RT_DATA.txt";
	ifstream RtData;
	RtData.open(this->RTdataPath + this->Filename);
	int cnt = 0;
	int cnt2 = 0;
	if (!RtData.good()){ exit(-1); }
		

	while (!RtData.eof())
	{
		//모든 라인을 읽어서 메모리에 넣는다.
		char *buf = new char[MAX_CHARS_PER_LINE];
		RtData.getline(buf, MAX_CHARS_PER_LINE);

		// 블랭크를단위로 토큰을 파싱한다
		int n = 0; // loop index
		cnt++;
		//buf에 있는 토큰들의 메모리주소를 어레이에 저장한다.
		const char* token[MAX_TOKENS_PER_LINE] = {}; // 0으로 초기화한다.z

		//라인을 파싱함.
		token[0] = strtok(buf, DELIMITER); // 첫번째 token
		if (token[0]) {

			for (n = 1; n < MAX_TOKENS_PER_LINE; n++){
				token[n] = strtok(NULL, DELIMITER); // 연속적인 토큰
				//cout << "토큰 : " << token[n	-1] << endl;
				if (!token[n]){
					this->Column = n;
					//cout << "n : " << n << endl;
					break;
					//토큰이 더이상 없을때 끝낸다.
				}
				cnt2++;
				//Resultfile << "}, " << endl;

			}
		}

		delete buf;
	}
	this->Row = cnt-2;
	TotalTable = new double*[this->Row];
	for (int i = 0; i < this->Row; i++) {
		TotalTable[i] = new double[this->Column];
		for (int j = 0; j < this->Column; j++) TotalTable[i][j] = { 0, };
	}
	/*
	for (int i = 0; i < this->Column; i++) {
		this->Lable[i] = "0";
	}*/
	cout << "column" << Column << endl;
	cout << "row " << this->Row << endl;
	
	ifstream RtData_;
	RtData_.open(this->RTdataPath + this->Filename);
	cnt = 0;

	if (!RtData_.good()) cout << "파일이 없어여 ㅠ"<<endl;
		//throw runtimeErr; // 파일이 없으면 나간다.


	while (!RtData_.eof())
	{
		//모든 라인을 읽어서 메모리에 넣는다.
		char *buf = new char[MAX_CHARS_PER_LINE];
		RtData_.getline(buf, MAX_CHARS_PER_LINE);

		// 블랭크를단위로 토큰을 파싱한다
		int n = 0; // loop index
		cnt++;
		//buf에 있는 토큰들의 메모리주소를 어레이에 저장한다.
		const char* token[MAX_TOKENS_PER_LINE] = {}; // 0으로 초기화한다.z

		//라인을 파싱함.
		token[0] = strtok(buf, DELIMITER); // 첫번째 token
		if (token[0]) {

			for (n = 1; n < MAX_TOKENS_PER_LINE; n++){
				token[n] = strtok(0, DELIMITER); // 연속적인 토큰

				if (!token[n]){

					break;
					//토큰이 더이상 없을때 끝낸다.
				}
			}
		}
		
		if (cnt == 1) {
			for (int i = 0; i < n; i++) {								
				this->Lable.push_back(token[i]);
				//cout << "Lable : " << Lable[i]<<endl;				
			}
		}		
		else {
			for (int i = 0; i < n; i++) {
				float teampToken = atof(token[i]);
				//cout << "cnt : " << cnt << ", teampToken : " << teampToken << " " << endl;
				this->TotalTable[cnt - 2][i] = teampToken;
			}
		}

		delete buf;
	}

	//cout << "cnt : " << cnt << ", this->Row : " << this->Row<< " " << endl;
	/*
	cout << "this->TotalTable[i][j]";
	for (int i = 0; i < this->Row; i++) {
		for (int j = 0; j < this->Column; j++) {

			cout << this->TotalTable[i][j] << " ";
		}
		cout << endl;
	}*/

	size = this->Row;
	ref = this->ReferenceInfo;
}

void
RT_Parsing::GetGPS_Table(double **table) {
	
	/*this->GPSData = new double*[this->Row];
	for (int i = 0; i < this->Row; i++) {
		this->GPSData[i] = new double[this->ReferenceInfo];
		for (int j = 0; j < this->ReferenceInfo < j; j++) {
			this->GPSData[i][j] = { 0, };
		}
	}*/
	int refData1(0), refData2(0), refData3(0), refData4(0), refData5(0), refData6(0), refData7(0), refData8(0), refData9(0), refData10(0);
	
	vector<string>::iterator iter;
	iter = find(this->Lable.begin(), this->Lable.end(), "RTdata.X");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) refData1++;

	iter = find(this->Lable.begin(), this->Lable.end(), "RTdata.Y");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) refData2++;

	iter = find(this->Lable.begin(), this->Lable.end(), "RTdata.heading");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) refData3++;

	iter = find(this->Lable.begin(), this->Lable.end(), "RTdata.VX");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) refData4++;

	iter = find(this->Lable.begin(), this->Lable.end(), "RTdata.VY");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) refData5++;

	iter = find(this->Lable.begin(), this->Lable.end(), "RTdata.angular_rate_yaw");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) refData6++;

	iter = find(this->Lable.begin(), this->Lable.end(), "RTdata.alt");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) refData7++;

	iter = find(this->Lable.begin(), this->Lable.end(), "RTdata.pitch");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) refData8++;

	iter = find(this->Lable.begin(), this->Lable.end(), "RTdata.roll");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) refData9++;
	cout << refData1 << " " << refData2 << " " << refData3 << " " << refData4 << " " << refData5 << " " << refData6 << endl;
		

	/*iter = find(this->Lable.begin(), this->Lable.end(), "RTdata.lat");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) refData1++;

	iter = find(this->Lable.begin(), this->Lable.end(), "RTdata.lon");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) refData2++;

	iter = find(this->Lable.begin(), this->Lable.end(), "RTdata.alt");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) refData3++;

	iter = find(this->Lable.begin(), this->Lable.end(), "RTdata.heading");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) refData4++;

	iter = find(this->Lable.begin(), this->Lable.end(), "RTdata.pitch");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) refData5++;

	iter = find(this->Lable.begin(), this->Lable.end(), "RTdata.roll");
	for (vector<string>::iterator it = this->Lable.begin(); it != iter; it++) refData6++;*/
	/*for (int i = 0; i < this->Row; i++) {
		this->GPSData[i][0] = i;
		this->GPSData[i][1] = this->TotalTable[i][refData1] - this->TotalTable[START_FRAME][refData1];
		this->GPSData[i][2] = this->TotalTable[i][refData2] - this->TotalTable[START_FRAME][refData2];
		this->GPSData[i][3] = -this->TotalTable[i][refData3] * drad;
		this->GPSData[i][4] = this->TotalTable[i][refData4];
		this->GPSData[i][5] = this->TotalTable[i][refData5];
		this->GPSData[i][6] = -this->TotalTable[i][refData6] * drad;

	}*/

	for (int i = 0; i < this->Row; i++) {
		table[i][0] = i;// +1;  //frame 번호 MATLAB에서 돌리기..
		table[i][2] = this->TotalTable[i][refData1] - this->TotalTable[START_FRAME-1][refData1];
		table[i][1] = this->TotalTable[i][refData2] - this->TotalTable[START_FRAME-1][refData2];
		table[i][3] = -this->TotalTable[i][refData3] * drad;
		table[i][4] = this->TotalTable[i][refData4];
		table[i][5] = this->TotalTable[i][refData5];
		table[i][6] = -this->TotalTable[i][refData6] * drad;
		table[i][7] = this->TotalTable[i][refData7] - this->TotalTable[START_FRAME - 1][refData7];
		table[i][8] = this->TotalTable[i][refData9] * drad;
		table[i][9] = this->TotalTable[i][refData8] * drad;
	}

	//for (int i = 0; i < this->Row; i++) {  // for check
	//	for (int j = 0; j < this->ReferenceInfo; j++) {
	//		table[i][j] = this->GPSData[i][j];
	//		//this->RT_DATA_LOG << table[i][j] << "	";
	//		}
	//	//this->RT_DATA_LOG << endl;
	//	
	//}
	//
	//
	//for (int i = 0; i < this->ReferenceInfo; i++) {  // for check
	//	for (int j = 0; j < this->Row; j++) {
	//		cout<< table[i][j]<<" ";
	//	}
	//	_getch();
	//	cout << endl;
	//}
	//
	for (int i = 0; i < Row; i++) delete[] TotalTable[i];
	delete[] TotalTable;

	/*for (int i = 0; i < Row; i++) delete[] GPSData[i];
	delete[] GPSData;*/
	std::cout << "RT Data Parsing Processing...end!" << std::endl;
	cout << endl;
}

inline string
RT_Parsing::PathSetting(string PATH, int num) {
	string path = PATH;
	string number;
	int num1(num);
	stringstream sst;
	sst << num1;
	sst >> number;
	string path1 = path + number + "_lidar.pcd";
	return path1;
}