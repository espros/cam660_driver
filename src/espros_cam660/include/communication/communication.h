/**
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @defgroup communication_implementation Base Implementation
 * @brief Abstract base implementation
 * @ingroup communication
 *
 * @{
 */
#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <list>
#include <string>
#include <iostream>
#include <exception>
#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>

#include "epc_timer.h"
#include "serial_connection.h"
#include "cam660_image.h"

namespace com_lib
{

class Communication
{

public:
    Communication();
    virtual ~Communication();
    bool open(std::string &portName);
    void close();

    //Information commands
    ErrorNumber_e getIdentification(bool &isBootloader);
    ErrorNumber_e getIdentification(bool &isBootloader, unsigned int &version);
    ErrorNumber_e getChipInformation(uint16_t &chipId, uint16_t &waferId);
    ErrorNumber_e getFirmwareRelease(unsigned int &major, unsigned int &minor);
    ErrorNumber_e getTemperature(double &temperature);

    //Setup commands
    ErrorNumber_e setHDRMode(const uint8_t hdrMode);
    ErrorNumber_e setIntegrationTime3d(const int intTime1, const int intTime2, const int intTime3, const int integrationTimeGrayscale);
    ErrorNumber_e setModulationFrequency(const uint8_t modulationFrequency, const uint8_t channel);
    ErrorNumber_e setMinimalAmplitude(const unsigned int amplitude);
    ErrorNumber_e setFilter(const bool medianFilter, const bool averageFilter, const uint16_t temporalFactor, const uint16_t temporalThreshold, const uint16_t edgeThreshold, const uint16_t interferenceDetectionLimit, const bool interferenceDetectionUseLastValue);
    ErrorNumber_e setOffset(const int offset);
    ErrorNumber_e setRoi(const unsigned int xMin, const unsigned int yMin, const unsigned int xMax, const unsigned int yMax);

    //Acquisition commands        
    void getDcs();    
    void stopStream();
    void startStream();
    void getGrayscale();
    void getDistance();
    void getDistanceAmplitude();


    //signals
    boost::signals2::signal<void ()> sigReceivedAnswer;
    boost::signals2::signal<void (const ErrorNumber_e &errorNumber)> sigErrorInternal;
    boost::signals2::signal<void (std::shared_ptr<TofCam660Image>)> sigReceivedDistance;
    boost::signals2::signal<void (std::shared_ptr<TofCam660Image>)> sigReceivedGrayscale;
    boost::signals2::signal<void (std::shared_ptr<TofCam660Image>)> sigReceivedDistanceAmplitude;    
    boost::signals2::signal<void (const uint32_t identification)>  sigReceivedIdentification;
    boost::signals2::signal<void (const uint16_t integrationTime)> sigReceivedIntegrationTime;
    boost::signals2::signal<void (const uint32_t firmwareRelease)> sigReceivedFirmwareRelease;
    boost::signals2::signal<void (const int16_t  temperature)> sigReceivedTemperature;
    boost::signals2::signal<void (const uint8_t year, const uint8_t week)> sigReceivedProductionInfo;
    boost::signals2::signal<void (const uint16_t waferId, const uint16_t chipId)> sigReceivedChipInformation;


private:
    enum CommunicationState_e  { COMMUNICATION_STATE_UNCONNECTED, COMMUNICATION_STATE_NORMAL };

    //private slots:
    void onReceivedData(const std::vector<uint8_t> &array, const uint8_t type);
    void onTimeout();

    ErrorNumber_e sendCommand(uint8_t *data, int size, bool streamMode = false);
    ErrorNumber_e sendCommandWithoutData(const uint8_t command, int size, bool streamMode = false);
    ErrorNumber_e sendCommandSingleByte(const uint8_t command, const uint8_t payload, int size = CommunicationConstants::Command::SIZE_PAYLOAD, bool streaming = false);
    ErrorNumber_e sendCommandUint16(const uint8_t command, const uint16_t payload);
    ErrorNumber_e sendCommandInt16(const uint8_t command, const int16_t payload);
    ErrorNumber_e sendCommand2xUint16(const uint8_t command, const uint16_t payload0, const uint16_t payload1);

    void sendErrorSignal(const ErrorNumber_e errorNumber);
    bool openInternal(std::string &portName);
    void processIdentification(const std::vector<uint8_t> &array);
    void processChipInformation(const std::vector<uint8_t> &array);
    void processTemperature(const std::vector<uint8_t> &array);
    void processFirmwareRelease(const std::vector<uint8_t> &array);
    void processIntegrationTime(const std::vector<uint8_t> &array);
    void processProductionInfo(const std::vector<uint8_t>  &array);
    void processDistanceAmplitude(const std::vector<uint8_t> &array);
    void processDistance(const std::vector<uint8_t> &array);
    void processGrayscale(const std::vector<uint8_t> &array);
    void insert8Value(uint8_t *output, int index, const int8_t value);
    void insertValue(uint8_t *output, int index, const int16_t value);
    int  boolToInt8(bool state);

    bool startStreamMode;    
    CommunicationState_e state;             ///<Defines the state of the communication    

    unsigned int timeout;
    unsigned int xMin;
    unsigned int yMin;
    unsigned int xMax;
    unsigned int yMax;

public:
    SerialConnection *serialConnection;     ///<SerialConnection instance


}; //end Communication

}  //end namespace com_lib

#endif // COMMUNICATION_H

/** @} */
