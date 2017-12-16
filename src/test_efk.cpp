#include <iostream>
#include <math.h>
#include "FusionEKF.h"
#include "tools.h"

#include <fstream>

using namespace std;


int main() 
{

	// Create a Kalman Filter instance
  	FusionEKF fusionEKF;

	string line;
	ifstream myfile ("../data/obj_pose-laser-radar-synthetic-input.txt");
	if (myfile.is_open())
	{


	  int cTemp = 0;

	  while ( getline (myfile,line) )
	  {
	    //#L(for laser) meas_px meas_py timestamp gt_px gt_py gt_vx gt_vy
		//#R(for radar) meas_rho meas_phi meas_rho_dot timestamp gt_px gt_py gt_vx gt_vy

	  	stringstream linestream(line);
	  	
	  	// reads first element from the current line
		string sensor_type;

		getline(linestream, sensor_type, '\t');

	  	MeasurementPackage meas_package;
		long long timestamp;

		cout << sensor_type << "\n";

		if (sensor_type.compare("L") == 0) {
		  	meas_package.sensor_type_ = MeasurementPackage::LASER;
			meas_package.raw_measurements_ = VectorXd(2);
			float px;
		  	float py;
			linestream >>  px;
			linestream >>  py;
			meas_package.raw_measurements_ << px, py;
			linestream >>  timestamp;
			meas_package.timestamp_ = timestamp;
		} else if (sensor_type.compare("R") == 0) {

			meas_package.sensor_type_ = MeasurementPackage::RADAR;
			meas_package.raw_measurements_ = VectorXd(3);
			float ro;
			float theta;
			float ro_dot;
			linestream >>  ro;
			linestream >>  theta;
			linestream >>  ro_dot;
			meas_package.raw_measurements_ << ro,theta, ro_dot;
			linestream >>  timestamp;
			meas_package.timestamp_ = timestamp;
		}
		
		  
	  	
		//Call ProcessMeasurment(meas_package) for Kalman filter
  		fusionEKF.ProcessMeasurement(meas_package);

	  	//cout << " tipo " << data << " " << x << endl;

	  	if (cTemp > 2) 
	  	{
	  		break;
	  	}

	  	cTemp++;
	  	

	  }
	    myfile.close();
	} 
	else cout << "Unable to open file";

	///

	/*	  
	*/


	return 0;
}