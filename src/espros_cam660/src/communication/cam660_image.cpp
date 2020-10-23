#include "cam660_image.h"
#include "util.h"
#include <ros/ros.h> //TODO remove

using namespace std;

namespace com_lib
{

TofCam660Image::TofCam660Image(const vector<uint8_t> &data)
{
    this->data = data;
    version = data[0];
    dataType = Util::getUint16BigEndian(data, 1); //answer or data
    width  = Util::getUint16BigEndian(data, 3);
    height = Util::getUint16BigEndian(data, 5);
    roiX0  = Util::getUint16BigEndian(data, 7);
    roiY0  = Util::getUint16BigEndian(data, 9);
    roiX1  = Util::getUint16BigEndian(data, 11);
    roiY1  = Util::getUint16BigEndian(data, 13);

    temperature = static_cast<double>(Util::getInt16BigEndian(data, 21)) / 100.0;
    offset = Util::getInt16BigEndian(data, 23); //offset where measurement data begins
    ROS_DEBUG("TofCam660Image header: v= %d type=%d w= %d h= %d temp= %2.2f offset= %d roiX0 = %d, roiY0= %d, roiX1= %d, roiY1 = %d", version, dataType, width, height, temperature, offset, roiX0, roiY0, roiX1, roiY1);
}

unsigned int TofCam660Image::getWidth() const
{
    return width;
}

unsigned int TofCam660Image::getHeight() const
{
    return height;
}

unsigned int TofCam660Image::getLeftX() const
{
    return roiX0;
}

unsigned int TofCam660Image::getTopY() const
{
    return roiY0;
}

double TofCam660Image::getTemperature() const
{
    return temperature;
}

std::vector<uint8_t>& TofCam660Image::getData()
{
    return data;
}

uint8_t* TofCam660Image::getData(unsigned int index)
{
    return  (uint8_t *)(&data.data()[offset + index]);
}

} //end namespace com_lib
