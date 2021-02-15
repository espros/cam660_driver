#include <ros/ros.h>
#include "sensor_msgs/Temperature.h"
#include <dynamic_reconfigure/server.h>
#include <espros_cam660/espros_cam660Config.h>
#include <signal.h>
#include <cstdlib>

#include "camera660_driver.h"
#include "cam660_image.h"

using namespace com_lib;
using namespace std;

Camera660Driver *cameraDriver;
static ros::Publisher imagePublisher1;
static ros::Publisher imagePublisher2;
static ros::Publisher pointCloud2Publisher;
static ros::Publisher temperaturePublisher;
static Settings settings;

//===================================================================

void updateConfig(espros_cam660::espros_cam660Config &config, uint32_t level)
{
    settings.runVideo = false;
    (void) level; //unused parameter

    switch(config.image_type){
    case 0: settings.iType = TofCam660Image::ImageType_e::GRAYSCALE;
       break;
    case 1: settings.iType = TofCam660Image::ImageType_e::DISTANCE;
       break;
    case 2: settings.iType = TofCam660Image::ImageType_e::DISTANCE_AMPLITUDE;
       break;
    case 3: settings.iType = TofCam660Image::ImageType_e::DCS;
       break;
    default: break;
    }

    settings.lensData = config.lens_data;
    settings.frameRate = config.frame_rate;    
    settings.lensCenterOffsetX = config.lens_center_offset_x;
    settings.lensCenterOffsetY = config.lens_center_offset_y;
    settings.hdrMode = config.hdr_mode;
    settings.integrationTimeTOF1  = static_cast<uint>(config.integration_time_tof_1);
    settings.integrationTimeTOF2  = static_cast<uint>(config.integration_time_tof_2);
    settings.integrationTimeTOF3  = static_cast<uint>(config.integration_time_tof_3);
    settings.integrationTimeGray   = static_cast<uint>(config.integration_time_gray);
    settings.modFrequency = config.frequency_modulation;
    settings.modChannel = config.channel_modulation;

    settings.medianFilter = config.median_filter;
    settings.averageFilter = config.average_filter;
    settings.kalmanFactor  = config.temporal_filter_factor;
    settings.kalmanThreshold = static_cast<uint>(config.temporal_filter_threshold);

    settings.edgeThreshold = config.edge_filter_threshold;
    settings.interferenceDetectionLimit = config.interference_detection_limit;
    settings.interferenceDetectionUseLastValue = config.use_last_value;

    settings.minDistance  = config.min_distance;
    settings.maxDistance  = config.max_distance;

    settings.minAmplitude = static_cast<uint>(config.min_amplitude);
    settings.startStream  = static_cast<uint>(config.start_stream);
    settings.enableCartesian   = config.cartesian;
    settings.enableTemperature = config.publish_temperature;
    settings.enableImages      = config.publish_images;
    settings.enablePointCloud  = config.publish_point_cloud;
    settings.startStream = static_cast<uint>(config.start_stream);

    settings.roi_leftX   = config.roi_left_x;
    settings.roi_rightX  = config.roi_right_x;

    if(settings.roi_rightX - settings.roi_leftX < 7)
        settings.roi_rightX = settings.roi_leftX + 7;

    settings.roi_rightX -= (settings.roi_rightX - settings.roi_leftX + 1) % 4;
    config.roi_right_x = settings.roi_rightX;

    config.roi_height -= config.roi_height % 4;
    if(config.roi_height < 8) config.roi_height = 8;

    settings.roi_topY    = 120 - config.roi_height/2;
    settings.roi_bottomY = 119 + config.roi_height/2;

    if(config.start_stream)
        settings.runVideo = true;

    settings.triggerSingleShot = config.trigger_single_shot;
    settings.updateParam = true;

}


void initialiseNode()
{
    ros::NodeHandle nh("~");
    nh.param("port_name", settings.port_name, std::string("/dev/ttyACM0"));
    nh.param("frame_rate", settings.frameRate, 30.0);

    //advertise publishers    
    imagePublisher1 = nh.advertise<sensor_msgs::Image>("image_raw1", 100);
    imagePublisher2 = nh.advertise<sensor_msgs::Image>("image_raw2", 100);
    pointCloud2Publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI> > ("points", 100);    
    temperaturePublisher = nh.advertise<sensor_msgs::Temperature>("temperature", 100);

    settings.runVideo = false;
    settings.updateParam = false;

    ROS_INFO("Camera 660 driver version 1.3.5");
}

//======================================================================

int main(int argc, char **argv)
{
    //if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) ros::console::notifyLoggerLevelsChanged();

    ros::init(argc, argv, "cam660_node");

    dynamic_reconfigure::Server<espros_cam660::espros_cam660Config> server;
    dynamic_reconfigure::Server<espros_cam660::espros_cam660Config>::CallbackType f;
    f = boost::bind(&updateConfig, _1, _2);
    server.setCallback(f);

    initialiseNode();

    ros::Rate rate(100);

    try{
        cameraDriver = new Camera660Driver (imagePublisher1, imagePublisher2, pointCloud2Publisher, temperaturePublisher, settings);

        while(ros::ok()){
            cameraDriver->update();
            ros::spinOnce();
            rate.sleep();
        }
    }

    catch(int e){
        ROS_ERROR("Program aborted. Exit code %d", e);
    }

}






