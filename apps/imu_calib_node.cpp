#include <iostream>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <boost/foreach.hpp>

#include "imu_tk/io_utils.h"
#include "imu_tk/calibration.h"
#include "imu_tk/filters.h"
#include "imu_tk/integration.h"
#include "imu_tk/visualization.h"

using namespace std;
using namespace imu_tk;
using namespace Eigen;

#define foreach BOOST_FOREACH

int main(int argc, char** argv)
{
  if( argc < 3 ) {
    cout << "usage:\n" << argv[0] <<" BAG IMU_TOPIC\n";
    return -1;
  }

  vector< TriadData > acc_data, gyro_data;

  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string(argv[2]));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  foreach(rosbag::MessageInstance const m, view) {
      sensor_msgs::Imu::ConstPtr s = m.instantiate<sensor_msgs::Imu>();
      if (s != NULL) {
          acc_data.push_back(TriadData(s->header.stamp.toSec(), s->linear_acceleration.x, s->linear_acceleration.y, s->linear_acceleration.z));
          gyro_data.push_back(TriadData(s->header.stamp.toSec(), s->angular_velocity.x, s->angular_velocity.y, s->angular_velocity.z));
      }
  }

  bag.close();
  
  /*
  cout<<"Importing IMU data from the Matlab matrix file : "<< argv[1]<<endl;  
  importAsciiData( argv[1], acc_data, imu_tk::TIMESTAMP_UNIT_SEC );
  cout<<"Importing IMU data from the Matlab matrix file : "<< argv[2]<<endl;  
  importAsciiData( argv[2], gyro_data, imu_tk::TIMESTAMP_UNIT_SEC  );
  */
  
  CalibratedTriad init_acc_calib, init_gyro_calib;
  init_acc_calib.setBias( Vector3d(0, 0, 0) );
  init_gyro_calib.setScale( Vector3d(1, 1, 1) );
  
  MultiPosCalibration mp_calib;
    
  mp_calib.setInitStaticIntervalDuration(50.0);
  mp_calib.setInitAccCalibration( init_acc_calib );
  mp_calib.setInitGyroCalibration( init_gyro_calib );  
  mp_calib.setGravityMagnitude(9.8);
  mp_calib.enableVerboseOutput(true);
  mp_calib.enableAccUseMeans(false);
  //mp_calib.setGyroDataPeriod(0.01);
  mp_calib.calibrateAccGyro(acc_data, gyro_data );
  mp_calib.getAccCalib().save("imu_acc.calib");
  mp_calib.getGyroCalib().save("imu_gyro.calib");
  
//   for( int i = 0; i < acc_data.size(); i++)
//   {
//     cout<<acc_data[i].timestamp()<<" "
  //         <<acc_data[i].x()<<" "<<acc_data[i].y()<<" "<<acc_data[i].z()<<" "
  //         <<gyro_data[i].x()<<" "<<gyro_data[i].y()<<" "<<gyro_data[i].z()<<endl;
//   }
//   cout<<"Read "<<acc_data.size()<<" tuples"<<endl;
  
  return 0;
}

