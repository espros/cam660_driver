#include "cartesian_transform.hpp"
#include <stdint.h>
#include <ros/ros.h>
#include <ros/package.h>


CartesianTransform::CartesianTransform(){

}

CartesianTransform::~CartesianTransform(){

}


double CartesianTransform::interpolate(double x_in, double x0, double y0, double x1, double y1)
{
    if(fabs(x1 - x0) < std::numeric_limits<double>::epsilon())  return y0;
    else return ((x_in-x0)*(y1-y0)/(x1-x0) + y0);
}

double CartesianTransform::getAngle(double x, double y, double sensorPointSizeMM)
{
    double radius = sensorPointSizeMM * sqrt(x*x + y*y);
    double alfaGrad = 0;

    for(int i=1; i < lensTableSize; i++)
    {
        if(radius >= rp[i-1] && radius <= rp[i]){

            alfaGrad = interpolate(radius, rp[i-1], angle[i-1], rp[i], angle[i]);
        }
    }

    return alfaGrad;
}



void CartesianTransform::initLensTransform(double sensorSizeMM, int width, int height, int offsetX, int offsetY, std::string& lensData)
{
    int x, y, row, col;
    numCols = width;
    numRows = height;

    loadLensDistortionTable(lensData);

    int r0 = 1 - numRows/2 + offsetY; //lens optical center offset
    int c0 = 1 - numCols/2 + offsetX;

    for(y=numRows-1, row = r0; y >=0; row++, y--){
        for(x=numCols-1, col = c0; x >=0; col++, x--){

            double c = col - 0.5;
            double r = row - 0.5;

            double angleGrad = getAngle(c, r, sensorSizeMM);
            double angleRad =  angleGrad * 3.14159265 / 180.0;

            double rp = sqrt(c * c + r * r);
            double rUA = sin(angleRad);

            xUA[x][y] = c * rUA / rp;
            yUA[x][y] = r * rUA / rp;
            zUA[x][y] = cos(angleRad);
        }
    }

}


// function for cartesian transfrmation
void CartesianTransform::transformPixel(uint srcX, uint srcY, double srcZ, double &destX, double &destY, double &destZ)
{
    destX = srcZ * xUA[srcX][srcY];
    destY = srcZ * yUA[srcX][srcY];
    destZ = srcZ * zUA[srcX][srcY];
}


void CartesianTransform::loadLensDistortionTable(std::string &lensData)
{
    int i;
    char str[100];

    std::string fname = ros::package::getPath("espros_cam660") + "/lensData/" + lensData;

    ROS_DEBUG("initLensTransform: %s ", fname.data());

    std::ifstream myfile;
    myfile.open(fname);

    if(myfile.is_open()){

        lensTableSize  = readLine("size", myfile);
        ROS_DEBUG("lensTableSize= %d", lensTableSize);

        readLine("angle", myfile);
        ROS_DEBUG("angle:");

        for(i=0; i<lensTableSize && !myfile.eof(); i++){
            myfile.getline(str, 100);
            angle[i] = std::strtod(str, nullptr);
            ROS_DEBUG("%2.4f ", angle[i]);
        }

        readLine("radius", myfile);

        ROS_DEBUG("radius:");

        for(i=0; i<lensTableSize && !myfile.eof(); i++){
            myfile.getline(str, 100);
            rp[i] = std::strtod(str, nullptr);
            ROS_DEBUG("%2.4f ", rp[i]);
        }

        myfile.close();
    }
}

int CartesianTransform::readLine(std::string str, std::ifstream &file)
{
    int val = 0;
    char txt[256];
    std::string strNum;
    std::string strLine;

    do{
        file.getline(txt, 100);
        strLine = txt;
    }while(strLine.find(str) == std::string::npos && !file.eof());


    int i=0;
    while((strLine[i]< 48 || strLine[i]>57) && i< strLine.size()) ++i;

    for(; (strLine[i]>= 48 && strLine[i]<=57) && i< strLine.size(); ++i)
        strNum.push_back(strLine[i]);

    if(strNum.size() > 0)
        val = std::stoi(strNum);    

    return val;
}


























