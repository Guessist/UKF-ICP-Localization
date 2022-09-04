#include "CustomUKF.h"
#include "RT_Parsing.h"
int main(void)
{
	
	/*********************** RT Data Parsing ***********************/
	int DataSize = 0;																// 취득한 전체 데이터 사이즈
	int RefInfo = 0;																// 맵을 만들기 위해 필요한 데이터 종류의 갯수
	RT_Parsing RT_DataHandling(0, 0);
	RT_DataHandling.GetTotalTable(DataSize, RefInfo);								// RTK 데이터 parsing 
	double **GPS_Table;
	cout << "DataSize : " << DataSize << ", RefInfo : " << RefInfo << endl;
	GPS_Table = new double*[DataSize];
	for (int i = 0; i < DataSize; i++) {
		GPS_Table[i] = new double[RefInfo];
		for (int j = 0; j < RefInfo; j++) GPS_Table[i][j] = { 0, };
	}
	RT_DataHandling.GetGPS_Table(GPS_Table);
	
	/*********************** UKF with ICP ***********************/
	CustomUKF UKFmodule(GPS_Table, DataSize);
	UKFmodule.UKFwithICP();
	cout << "UKF 끝" << endl;
	_getch();
	for (int i = 0; i < DataSize; i++) delete[] GPS_Table[i];
	delete[] GPS_Table;
	return 0;
}
