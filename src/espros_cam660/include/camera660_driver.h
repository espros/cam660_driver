#ifndef CAMERA660_DRIVER_H
#define CAMERA660_DRIVER_H

#include "cam660_image.h"
#include "communication.h"
#include "cartesian_transform.hpp"

#include <dynamic_reconfigure/server.h>
#include <espros_cam660/espros_cam660Config.h>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Temperature.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32MultiArray.h>


struct Settings{

  std::string port_name;
  double frameRate;
  bool startStream;
  bool triggerSingleShot;
  bool runVideo;
  bool updateParam;
  uint integrationTimeTOF1;
  uint integrationTimeTOF2;
  uint integrationTimeTOF3;
  uint integrationTimeGray;

  int imageType;
  int hdrMode;
  int modFrequency;
  int modChannel;

  bool medianFilter;
  bool averageFilter;
  double kalmanFactor;
  uint kalmanThreshold;
  int edgeThreshold;
  int interferenceDetectionLimit;
  bool interferenceDetectionUseLastValue;

  uint minAmplitude;
  int minDistance;
  int maxDistance;  
  int lensCenterOffsetX;
  int lensCenterOffsetY;
  bool enableCartesian;
  bool enableImages;
  bool enablePointCloud;  
  bool enableTemperature;

  int roi_leftX;
  int roi_topY;
  int roi_rightX;
  int roi_bottomY;

  std::string lensData;
  com_lib::TofCam660Image::ImageType_e iType;

};



class Camera660Driver
{

public:
    Camera660Driver(const ros::Publisher &imagePublisher1_, const ros::Publisher &imagePublisher2_,
                    const ros::Publisher &pointCloud2Publisher_, const ros::Publisher &temperaturePublisher_, Settings &set_);
    ~Camera660Driver();
    void update();   
    void initCommunication();
    void closeCommunication();

private:

    double angle;
    bool lastSingleShot;
    bool lastStreaming;
    double framePeriod;
    ros::Time timeLast;
    uint frameSeq;    
    static Settings *gSettings;
    const ros::Publisher &imagePublisher1;
    const ros::Publisher &imagePublisher2;
    const ros::Publisher &pointCloud2Publisher;
    const ros::Publisher &temperaturePublisher;

    uint imageSize8;
    uint imageSize16_1;
    uint imageSize16_2;
    std::string strFrameID;
    sensor_msgs::Image img8;
    sensor_msgs::Image img16_1;
    sensor_msgs::Image img16_2;
    sensor_msgs::Temperature msgTemperature;
    com_lib::Communication communication;

    int lensTableSize;
    int numCols;
    int numRows;
    double lensAngle[101];
    double rp[101];
    double xUA[320][240];
    double yUA[320][240];
    double zUA[320][240];
    double sensorPointSizeMM;
    int oldLensCenterOffsetX;
    int oldLensCenterOffsetY;
    std::string old_lensData;

    void updateData();
    void setParameters();
    void updateTemperature(double temperature);
    void updateGrayscaleFrame(std::shared_ptr<com_lib::TofCam660Image> image);
    void updateDistanceFrame(std::shared_ptr<com_lib::TofCam660Image> image);
    void updateDistanceAmplitudeFrame(std::shared_ptr<com_lib::TofCam660Image> image);

    CartesianTransform cartesian;
};


#endif // CAMERA660_DRIVER_H
