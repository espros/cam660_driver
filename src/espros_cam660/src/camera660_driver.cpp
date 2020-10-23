
#include "camera660_driver.h"

using namespace com_lib;
using namespace std;

Settings *Camera660Driver::gSettings;

Camera660Driver::Camera660Driver(const ros::Publisher &imagePublisher1_, const ros::Publisher &imagePublisher2_,
                                 const ros::Publisher &pointCloud2Publisher_, const ros::Publisher &temperaturePublisher_, Settings &set_):
imagePublisher1(imagePublisher1_),
imagePublisher2(imagePublisher2_),
pointCloud2Publisher(pointCloud2Publisher_),
temperaturePublisher(temperaturePublisher_)
{        
    numCols = 320;
    numRows = 240;
    frameSeq = 0;
    imageSize8 = 0;
    imageSize16_1 = 0;
    imageSize16_2 = 0;        
    gSettings = &set_;    
    sensorPointSizeMM = 0.02; //sensor pixel size mm
    gSettings->runVideo = false;
    gSettings->updateParam = false;
    lastSingleShot = false;
    lastStreaming = false;
    strFrameID = "sensor_frame";    

    initCommunication();

    communication.sigReceivedGrayscale.connect(boost::bind(&Camera660Driver::updateGrayscaleFrame, this, _1));
    communication.sigReceivedDistance.connect(boost::bind(&Camera660Driver::updateDistanceFrame, this, _1));
    communication.sigReceivedDistanceAmplitude.connect(boost::bind(&Camera660Driver::updateDistanceAmplitudeFrame, this, _1));    

    gSettings->updateParam = true;        
    timeLast = ros::Time::now();

    setParameters();

    if(gSettings->startStream)
      gSettings->runVideo = true;

    cartesian.initLensTransform(sensorPointSizeMM, numCols, numRows, gSettings->lensCenterOffsetX, gSettings->lensCenterOffsetY, gSettings->lensType);
    oldLensCenterOffsetX = gSettings->lensCenterOffsetX;
    oldLensCenterOffsetY = gSettings->lensCenterOffsetY;
    oldLensType = gSettings->lensType;
}

Camera660Driver::~Camera660Driver()
{
    ROS_DEBUG("Camera660Driver::~Camera660Driver()");
    communication.stopStream();
    ros::Duration(1).sleep();
    communication.close();
}


void Camera660Driver::closeCommunication()
{
    ROS_DEBUG("Camera660Driver::closeCommunication()"); //TODo remove
    communication.close();
}


void Camera660Driver::update()
{
    if(gSettings->runVideo && !gSettings->updateParam){

        updateData(); //streaming

    }else if(gSettings->updateParam){
        setParameters(); //update parameters

        if(gSettings->triggerSingleShot && gSettings->triggerSingleShot != lastSingleShot)
            updateData(); //trigger single shot

        lastSingleShot = gSettings->triggerSingleShot;
    }
}

void Camera660Driver::setParameters()
{
    if(gSettings->updateParam)
    {
        gSettings->updateParam = false;
        ROS_INFO("update parameters");

        framePeriod = 1.0 / gSettings->frameRate;

        communication.setHDRMode(gSettings->hdrMode);
        communication.setIntegrationTime3d(gSettings->integrationTimeTOF1, gSettings->integrationTimeTOF2, gSettings->integrationTimeTOF3, gSettings->integrationTimeGray);        
        communication.setMinimalAmplitude(gSettings->minAmplitude);
        communication.setModulationFrequency(gSettings->modFrequency, gSettings->modChannel);
        communication.setFilter(gSettings->medianFilter, gSettings->averageFilter, 1000.0 * gSettings->kalmanFactor, gSettings->kalmanThreshold, gSettings->edgeThreshold, gSettings->interferenceDetectionLimit, gSettings->interferenceDetectionUseLastValue);
        communication.setRoi(gSettings->roi_leftX, gSettings->roi_topY, gSettings->roi_rightX, gSettings->roi_bottomY);

        //lens parameters for cartesian transformation
        if(oldLensCenterOffsetX != gSettings->lensCenterOffsetX  || oldLensCenterOffsetY != gSettings->lensCenterOffsetY || oldLensType != gSettings->lensType){
            cartesian.initLensTransform(sensorPointSizeMM, numCols, numRows, gSettings->lensCenterOffsetX, gSettings->lensCenterOffsetY, gSettings->lensType);
            oldLensCenterOffsetX = gSettings->lensCenterOffsetX;
            oldLensCenterOffsetY = gSettings->lensCenterOffsetY;
            oldLensType = gSettings->lensType;
        }        

    } //END if(gSettings->updateParam)
}

void Camera660Driver::updateData()
{      
    ros::Time timeNow = ros::Time::now();
    double elapsed_time = timeNow.toSec() - timeLast.toSec();

    if(elapsed_time >= framePeriod){

        timeLast = timeNow;

        ROS_DEBUG("FRAME RATE HZ: %2.4f", 1.0/elapsed_time);

        switch(gSettings->iType)
        {
            case TofCam660Image::ImageType_e::GRAYSCALE:
                  communication.getGrayscale();
                  break;
            case TofCam660Image::ImageType_e::DISTANCE:
                  communication.getDistance();
                  break;
            case TofCam660Image::ImageType_e::DISTANCE_AMPLITUDE:
                  communication.getDistanceAmplitude();
                  break;
        }

    } //end if elapsed_time

}

void Camera660Driver::initCommunication()
{
  communication.open(gSettings->port_name); //open serial port  
  ros::Duration(0.5).sleep();

  unsigned int minor, major;
  communication.getFirmwareRelease(major, minor);
  ROS_INFO("Firmware release:  major= %d  minor= %d", major, minor);

  uint16_t chipID, waferID;
  communication.getChipInformation(chipID, waferID);
  ROS_INFO("Chip ID= %d   Wafer ID= %d", chipID, waferID);
}


void Camera660Driver::updateTemperature(double temperature)
{
    ROS_DEBUG("Temperature: %2.2f", temperature);
    msgTemperature.header.frame_id = "sensor_frame";
    msgTemperature.variance = 0.0; //0.05 ?
    msgTemperature.header.stamp = ros::Time::now();
    msgTemperature.temperature = temperature;
    temperaturePublisher.publish(msgTemperature);
    msgTemperature.header.seq++;
}


void Camera660Driver::updateGrayscaleFrame(std::shared_ptr<com_lib::TofCam660Image> image)
{
    if(gSettings->enableTemperature){
        updateTemperature(image->getTemperature());
    }

    if(gSettings->enableImages)
    {
        img16_1.header.seq = frameSeq++;
        img16_1.header.stamp = ros::Time::now();
        img16_1.header.frame_id = strFrameID;
        img16_1.height = static_cast<uint32_t>(image->getHeight());
        img16_1.width = static_cast<uint32_t>(image->getWidth());
        img16_1.encoding = sensor_msgs::image_encodings::MONO16;
        img16_1.step = img16_1.width * 2;
        img16_1.is_bigendian = 0;
        uint numPix = img16_1.width * img16_1.height;
        uint dataSize =  numPix * 2;

        if(imageSize16_1 != numPix){
            imageSize16_1 = numPix;
            img16_1.data.resize(static_cast<unsigned long>(numPix));
        }

        for(int l=0; l< dataSize; l++)
            img16_1.data[l] = image->getData()[l];

        imagePublisher1.publish(img16_1);

    } //end if enableImages
}


void Camera660Driver::updateDistanceFrame(std::shared_ptr<com_lib::TofCam660Image> image)
{    
    int x, y, i, l;
    std::vector<uint8_t>& data = image->getData();

    if(gSettings->enableTemperature){
        updateTemperature(image->getTemperature());
    }

    if(gSettings->enableImages)
    {
        img16_1.header.seq = frameSeq++;
        img16_1.header.stamp = ros::Time::now();
        img16_1.header.frame_id = strFrameID;
        img16_1.height = static_cast<uint32_t>(image->getHeight());
        img16_1.width = static_cast<uint32_t>(image->getWidth());
        img16_1.encoding = sensor_msgs::image_encodings::MONO16;
        img16_1.step = img16_1.width * 2;
        img16_1.is_bigendian = 0;
        uint numPix = img16_1.width * img16_1.height;
        uint dataSize =  numPix * 2;

        if(imageSize16_1 != numPix){
            imageSize16_1 = numPix;
            img16_1.data.resize(static_cast<ulong>(dataSize));
        }

        for(l=25, i=0; i< dataSize; i++, l++)
            img16_1.data[i] = image->getData()[l];

        imagePublisher1.publish(img16_1);
    }

    if(gSettings->enablePointCloud)
    {
        static int sz_pc = 0;
        const uint nPixel = image->getWidth() * image->getHeight();
        static pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cloud->header.frame_id = strFrameID;
        cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        cloud->width = static_cast<uint32_t>(image->getWidth());
        cloud->height = static_cast<uint32_t>(image->getHeight());
        cloud->is_dense = false;

        if(sz_pc != static_cast<int>(nPixel)){
            cloud->points.resize(static_cast<ulong>(nPixel));
            sz_pc = static_cast<int>(nPixel);
        }
                
        int k;
        double px, pz, py;
        int16_t leftX   = static_cast<int16_t>(image->getLeftX());
        int16_t topY    = static_cast<int16_t>(image->getTopY());
        int16_t rightX  = static_cast<int16_t>(image->getWidth()  + leftX);
        int16_t bottomY = static_cast<int16_t>(image->getHeight() + topY);

        for(k=0, l=25, y = topY; y < bottomY; y++){
            for(x = leftX; x < rightX; x++, k++, l+=2){

                uint dist = ((static_cast<uint16_t>(data[l+1])<<8) & 0xff00) + (static_cast<uint16_t>(data[l]) & 0x00ff);
                pcl::PointXYZI &p = cloud->points[k];

                if(dist < CommunicationConstants::PixelTofCam660::LIMIT_VALID_PIXEL && dist>= gSettings->minDistance && dist <= gSettings->maxDistance){

                    if(gSettings->enableCartesian)
                    {
                        cartesian.transformPixel(x, y, dist, px, py, pz);
                        p.x = static_cast<float>(px / 1000.0); //mm -> m
                        p.y = static_cast<float>(py / 1000.0);
                        p.z = static_cast<float>(pz / 1000.0);
                        p.intensity = static_cast<float>(pz / 1000.0);

                    }else{

                        p.x = static_cast<float>(x / 1000.0);
                        p.y = static_cast<float>(y / 1000.0);
                        p.z = static_cast<float>(dist / 1000.0);
                        p.intensity = static_cast<float>(dist / 1000.0);
                    }

                }else{
                    p.x = std::numeric_limits<float>::quiet_NaN();
                    p.y = std::numeric_limits<float>::quiet_NaN();
                    p.z = std::numeric_limits<float>::quiet_NaN();
                    p.intensity = std::numeric_limits<float>::quiet_NaN();
                }

            } //ensd for x
        } //end for y

        pointCloud2Publisher.publish(cloud);

    } //end if enablePointCloud   */

}

void Camera660Driver::updateDistanceAmplitudeFrame(std::shared_ptr<TofCam660Image> image)
{    
    uint i, l;
    std::vector<uint8_t>& data = image->getData();

    if(gSettings->enableTemperature){
        updateTemperature(image->getTemperature());
    }

    if(gSettings->enableImages)
    {
        img16_1.header.seq = frameSeq;
        img16_1.header.stamp = ros::Time::now();
        img16_1.header.frame_id = strFrameID;
        img16_1.height = static_cast<uint32_t>(image->getHeight());
        img16_1.width = static_cast<uint32_t>(image->getWidth());
        img16_1.encoding = sensor_msgs::image_encodings::MONO16;
        img16_1.step = img16_1.width * 2;
        img16_1.is_bigendian = 0;
        uint numPix = img16_1.width * img16_1.height;
        int dataSize = numPix * 2;

        if(imageSize16_1 != numPix){
            imageSize16_1 = numPix;
            img16_1.data.resize(static_cast<ulong>(numPix) * 2);
        }


        for(l=0, i=25; l< dataSize; i+=4, l+=2){                                    
            img16_1.data[l]   = data[i];
            img16_1.data[l+1] = data[i+1];
        }

        imagePublisher1.publish(img16_1);


        img16_2.header.seq = frameSeq++;
        img16_2.header.stamp = ros::Time::now();
        img16_2.header.frame_id = strFrameID;
        img16_2.height = static_cast<uint32_t>(image->getHeight());
        img16_2.width = static_cast<uint32_t>(image->getWidth());
        img16_2.encoding = sensor_msgs::image_encodings::MONO16;
        img16_2.step = img16_2.width * 2;
        img16_2.is_bigendian = 0;


        if(imageSize16_2 != numPix){
            imageSize16_2 = numPix;
            img16_2.data.resize(static_cast<ulong>(numPix) * 2);
        }

        for(l=0, i=27; l < dataSize; i+=4, l+=2 ){
            img16_2.data[l]   = data[i];
            img16_2.data[l+1] = data[i+1];
        }

        imagePublisher2.publish(img16_2);
    }


    if(gSettings->enablePointCloud)
    {
        const uint nPixel = image->getWidth() * image->getHeight();
        static pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cloud->header.frame_id = strFrameID;
        cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        cloud->width = static_cast<uint32_t>(image->getWidth());
        cloud->height = static_cast<uint32_t>(image->getHeight());
        cloud->is_dense = false;

        static uint szAmp= 0;
        if(szAmp != nPixel){
            szAmp = nPixel;
            cloud->points.resize(nPixel);
        }

        uint x, y, k;             
        double px, pz, py;
        uint16_t leftX   = static_cast<uint16_t>(image->getLeftX());
        uint16_t topY    = static_cast<uint16_t>(image->getTopY());
        uint16_t rightX  = static_cast<uint16_t>(image->getWidth()  + leftX);
        uint16_t bottomY = static_cast<uint16_t>(image->getHeight() + topY);


        for(l=25, k=0, y = topY; y < bottomY; y++){
            for(x = leftX; x < rightX; x++, k++, l+=4){

                uint16_t msb= data[l+1];
                uint16_t lsb= data[l];
                uint16_t dist = ((msb << 8) & 0xff00) + (lsb & 0x00ff); //org

                msb= data[l+3];
                lsb= data[l+2];
                uint16_t ampl = ((msb << 8) & 0xff00) + (lsb & 0x00ff);

                pcl::PointXYZI &p = cloud->points[k];

                if(dist < CommunicationConstants::PixelTofCam660::LIMIT_VALID_PIXEL && dist>= gSettings->minDistance && dist <= gSettings->maxDistance){ //&& ampl>= gSettings->minAmplitude

                    if(gSettings->enableCartesian){
                        cartesian.transformPixel(x, y, (double)(dist), px, py, pz);
                        p.x = static_cast<float>(px / 1000.0); //mm -> m
                        p.y = static_cast<float>(py / 1000.0);
                        p.z = static_cast<float>(pz / 1000.0);
                    }else{
                        p.x = static_cast<float>(x / 1000.0);
                        p.y = static_cast<float>(y / 1000.0);
                        p.z = static_cast<float>(dist / 1000.0);
                    }

                    p.intensity = ampl;

                }else{
                    p.x = std::numeric_limits<float>::quiet_NaN();
                    p.y = std::numeric_limits<float>::quiet_NaN();
                    p.z = std::numeric_limits<float>::quiet_NaN();
                    p.intensity = std::numeric_limits<float>::quiet_NaN();
                }

            } //ensd for x
        } //end for y

        pointCloud2Publisher.publish(cloud);

      } //end if enablePointCloud

}
