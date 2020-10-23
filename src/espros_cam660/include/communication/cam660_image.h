#ifndef TOFCAM660HEADER_H
#define TOFCAM660HEADER_H

#include "communication_constants.h"
#include <vector>
#include <string>


namespace com_lib
{

class TofCam660Image
{
public:
    enum ImageType_e { GRAYSCALE, DISTANCE, DISTANCE_AMPLITUDE, DCS };

    TofCam660Image(const std::vector<uint8_t> &data);    

    unsigned int getWidth() const;
    unsigned int getHeight() const;
    unsigned int getLeftX() const;
    unsigned int getTopY() const;
    double getTemperature() const;
    std::vector<uint8_t>& getData();
    uint8_t* getData(unsigned int index);

protected:
    std::vector<uint8_t> data;

private:    
    uint8_t version;
    uint16_t dataType;
    uint16_t width;
    uint16_t height;
    uint16_t roiX0;
    uint16_t roiY0;
    uint16_t roiX1;
    uint16_t roiY1;
    uint16_t offset;
    double temperature;
};

}

#endif // TOFCAM660HEADER_H
