/***
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @addtogroup communication_implementation
 */
#include "communication.h"
#include "communication_constants.h"
#include "u32_helper.h"
#include "u16_helper.h"
#include "int16_helper.h"
#include "chip_information_helper.h"
#include "util.h"
#include <iostream>
#include <cstring>
#include <ros/ros.h>
#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>

#define IDENTIFY_SIZE                  12
#define GET_TEMPERATURE_SIZE           13
#define SET_COMMAND_ANSWER_SIZE        11
#define SET_INTEGRATION_TIME_DIS_SIZE  11
#define SET_MODULATION_SIZE            11
#define SET_FILTER_SIZE                11
#define GET_INTEGRATION_TIME_DIS_SIZE  10
#define GET_CHIP_INFORMATION_SIZE      15
#define GET_FIRMWARE_VERSION_SIZE      15
#define GET_DISTANCE_GRAYSCALE_SIZE 28880
#define GET_DISTANCE_AMPLITUDE_SIZE 38480
#define GET_DISTANCE_SIZE           19280
#define GET_GRAYSCALE_SIZE           9680


using namespace std;

namespace com_lib
{

//There are two different timeout times. During connecting, the timeout is short, becuse on each com port a device will be searched. To prevent long searching time, the timeout must be small.
static const unsigned int TIMEOUT_CONNECTING = 100;  ///<Communication timeout in ms during connecting phase
static const unsigned int TIMEOUT_NORMAL = 1000;     ///<Communication timeout in ms during normal operaion

Communication::Communication()
{  
    serialConnection = new SerialConnection;

    timeout = TIMEOUT_CONNECTING;

    serialConnection->sigReceivedData.connect(boost::bind(&Communication::onReceivedData, this, _1, _2));

    //Member initialization
    state = CommunicationState_e::COMMUNICATION_STATE_UNCONNECTED;
    startStreamMode = false;

    xMin = 0;
    yMin = 0;
    xMax = 319;
    yMax = 239;
}

Communication::~Communication()
{
    delete serialConnection;
}

/**
 * @brief Open the serial port
 *
 * @param id Id of the serial port = index of the list when the function "availableDevices" is called
 * @retval true Port is open
 * @retval false Port is not open
 * @return Port open or not
 */
bool Communication::openInternal(string &portName)
{
    if(!serialConnection->openPort(portName)){
        return false;
    }

    return true;
}

/**
 * @brief Open the serial port
 *
 * @param id Id of the serial port = index of the list when the function "availableDevices" is called
 * @retval true Port is open
 * @retval false Port is not open
 * @return Port open or not
 */
bool Communication::open(string &portName)
{    
    bool connected = openInternal(portName);

    ROS_DEBUG("Communication::open %d", connected); //TODO remove

    if(connected){
        state = COMMUNICATION_STATE_NORMAL;
        timeout = TIMEOUT_NORMAL;
    }

    return connected;
}

/**
 * @brief Close the serial port
 *
 */
void Communication::close()
{
    serialConnection->closePort();
    state = COMMUNICATION_STATE_UNCONNECTED;
}


/**
 * @brief Send error signal to connected slots
 *
 * This function emits the error signal. It emits it anyway internally. If the connection is established
 * it emits it also externally. The reason is, that during the connecting phase, some commands are sent to detect
 * if a device is listening. If no device answers, there would be an error.
 *
 * @param errorNumber Error number to send
 */

void Communication::sendErrorSignal(const ErrorNumber_e errorNumber)
{
    //Emit the error internally
    sigErrorInternal(errorNumber); //lsi


    //Emit the error external
    switch(state)
    {
    case CommunicationState_e::COMMUNICATION_STATE_NORMAL:
        //no break
    default:
        //Do not emit the error external
        break;
    }
}

/**
 * @brief helper function to send the command
 *
 * All commands are sent over this function. Here also the timeouts
 * are handled.
 *
 * @param data Pointer to the already filled data to send
 * @param blocking set to true to run command blocking
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::sendCommand(uint8_t *data, int size, bool streamMode)
{  
    if(!streamMode || startStreamMode){
        serialConnection->sendData(data);
        startStreamMode = false;
    }

    ROS_DEBUG("read command expected size: %d", size);

    int sz = 0;
    int count = 0;
    serialConnection->rxArray.clear();

    for(int n= 0; n < size; n+= sz){
        sz = serialConnection->readRxData(size);

        if(sz == -1){
            ROS_ERROR("Communication::sendCommand serialConnection->readRxData sz=-1 count = %d ", count);
            return ERROR_NUMBER_SERIAL_PORT_ERROR;
        }else if(sz == 0){
            ROS_ERROR("Communication::sendCommand serialConnection->readRxData timeout: %d/%d bytes received", count, size);
            return ERROR_NUMBER_SERIAL_PORT_ERROR;
        }
    }

    return ERROR_NUMMBER_NO_ERROR;
}

/**
 * @brief Send a command without data
 *
 * This function is used for commands without any payload.
 *
 * @param command Command to send
 * @param blocking set to true to run command blocking
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::sendCommandWithoutData(const uint8_t command, int size, bool streamMode)
{
    uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];

    memset(output, 0, sizeof(output));

    //Add the command    
    insertValue(output, CommunicationConstants::Command::INDEX_COMMAND, command);

    return sendCommand(output, size, streamMode);
}

/**
 * @brief Send single byte command
 *
 * This function is used for commands with only one byte of payload.
 *
 * @param command Command to send
 * @param payload Payload to send
 * @param blocking set to true tur run command blocking
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::sendCommandSingleByte(const uint8_t command, const uint8_t payload, int size, bool streaming)
{
    uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];

    memset(output, 0, sizeof(output));

    //Add the command    
    insertValue(output, CommunicationConstants::Command::INDEX_COMMAND, command);

    //Add the single byte at the first position
    output[CommunicationConstants::Command::INDEX_DATA] = payload;

    return sendCommand(output, size, streaming);
}

/**
 * @brief Send 16bit / 2byte command
 *
 * This function is used for commands with one 16bit value as payload
 *
 * @param command Command to send
 * @param payload Payload to send
 * @param blocking set to true tur run command blocking
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::sendCommandUint16(const uint8_t command, const uint16_t payload)
{
    uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];

    memset(output, 0, sizeof(output));

    //Add the command
    output[CommunicationConstants::Command::INDEX_COMMAND] = command;

    //Add the payload
    Util::setUint16LittleEndian(output, CommunicationConstants::Command::INDEX_DATA, payload);

    return sendCommand(output, CommunicationConstants::Command::SIZE_PAYLOAD);
}


/**
 * @brief Send 16bit / 2byte command signed
 *
 * This function is used for commands with one 16bit value as payload
 *
 * @param command Command to send
 * @param payload Payload to send
 * @param blocking set to true tur run command blocking
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::sendCommandInt16(const uint8_t command, const int16_t payload)
{
    uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];

    memset(output, 0, sizeof(output));

    //Add the command
    output[CommunicationConstants::Command::INDEX_COMMAND] = command;

    //Add the payload
    Util::setInt16LittleEndian(output, CommunicationConstants::Command::INDEX_DATA, payload);

    return sendCommand(output, CommunicationConstants::Command::SIZE_PAYLOAD);
}



/**
 * @brief Send 2 x 16bit / 4byte command
 *
 * This function is used for commands with two 16bit value as payload
 *
 * @param command Command to send
 * @param payload0 First payload value to send
 * @param payload1 Second payload value to send
 * @param blocking set to true tur run command blocking
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::sendCommand2xUint16(const uint8_t command, const uint16_t payload0, const uint16_t payload1)
{
    uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];

    memset(output, 0, sizeof(output));

    //Add the command
    output[CommunicationConstants::Command::INDEX_COMMAND] = command;

    //Add the payload values
    Util::setUint16LittleEndian(output, CommunicationConstants::Command::INDEX_DATA, payload0);
    Util::setUint16LittleEndian(output, CommunicationConstants::Command::INDEX_DATA+2, payload1);

    return sendCommand(output, CommunicationConstants::Command::SIZE_PAYLOAD);
}

/**
 * @brief Timeout callback
 *
 * This function is called, if a command times out. It is used to handle the timeouts
 */
void Communication::onTimeout()
{
    //Prevent the timer of generating further signals    
    sendErrorSignal(ErrorNumber_e::ERROR_NUMBER_TIMEOUT);
}

/**
 * @brief Process identification data
 *
 * This function is called when identification data is received.
 *
 * @param array Pointer to the received data
 */
void Communication::processIdentification(const std::vector<uint8_t> &array)
{
    //Here the array is already cut to the payload, so get the data at index 0
    uint32_t identification = Util::getUint32LittleEndian(array, 0);
    sigReceivedIdentification(identification);
}

/**
 * @brief Process chip information data
 *
 * This function is called when chip information data is received.
 *
 * @param array Pointer to the received data
 */
void Communication::processChipInformation(const std::vector<uint8_t> &array)
{
    //Here the array is already cut to the payload at index 1
    uint16_t chipId  = static_cast<uint16_t>(Util::getUint16BigEndian(array, 1));
    uint16_t waferId = static_cast<uint16_t>(Util::getUint16BigEndian(array, 3));
    sigReceivedChipInformation(chipId, waferId);
}

/**
 * @brief Process temperature data
 *
 * This function is called when temperature data is received.
 *
 * @param array Pointer to the received data
 */
void Communication::processTemperature(const std::vector<uint8_t> &array)
{
    int16_t temperature = static_cast<int16_t>(Util::getInt16BigEndian(array, 1));
    sigReceivedTemperature(temperature);
}

/**
 * @brief Process firmware release data
 *
 * This function is called when the firmware release data is received.
 *
 * @param array Pointer to the received data
 */
void Communication::processFirmwareRelease(const std::vector<uint8_t> &array)
{
    //Here the array is already cut to the payload, so get the data at index 1
    for(int i=0; i< array.size(); i++){
        if(i<30)
            ROS_DEBUG("Communication::processFirmwareRelease array[%d]= %d", i, array[i]); //TODO remove
    }

    uint32_t firmwareRelease = Util::getUint32BigEndian(array, 1);
    sigReceivedFirmwareRelease(firmwareRelease);
}

/**
 * @brief Process integration time data
 *
 * This function is called when integration time data is received
 *
 * @param array Pointer to the received data
 */
void Communication::processIntegrationTime(const std::vector<uint8_t> &array)
{
    //Here the array is already cut to the payload, so get the data at index 0
    uint16_t integrationTime = static_cast<uint16_t>(Util::getUint16LittleEndian(array, 0));
    sigReceivedIntegrationTime(integrationTime);
}

/**
 * @brief Process production info
 *
 * This function is called when the production info is received
 *
 * @param array Pointer to the received data
 */
void Communication::processProductionInfo(const std::vector<uint8_t> &array)
{
    //Here the array is already cut to the payload, so get the data at index 0
    uint8_t year = array.at(0);
    uint8_t week = array.at(1);
    sigReceivedProductionInfo(year, week);
    ROS_INFO_STREAM("year = " << year << "week = " << week);
}

/**
 * @brief Handle received data
 *
 * This function is called when data from the device has been received.
 * It handles the received data depending on the type.
 *
 * @param array Pointer to the received data
 * @param type Type of the data
 */
void Communication::onReceivedData(const std::vector<uint8_t> &array, const uint8_t type)
{

    if(array.size()>3)
        ROS_DEBUG("Communication::onReceivedData: d0= %d, d1= %d, d2= %d, d3= %d, ...,  type= %d", array[0], array[1], array[2], array[3], type);
    else ROS_DEBUG("Communication::onReceivedData size= %d  type= %d", (int)array.size(),  type);

    switch(type)
    {
    case CommunicationConstants::Type::DATA_ACK:
        ROS_DEBUG("received Ack");
        break;
    case CommunicationConstants::Type::DATA_NACK:
        sendErrorSignal(ErrorNumber_e::ERROR_NUMBER_NOT_ACKNOWLEDGE);
        ROS_DEBUG("received Nack");
        break;
    case CommunicationConstants::Type::DATA_CHIP_INFORMATION:
        processChipInformation(array);
        ROS_DEBUG("received Chip Information");
        break;
    case CommunicationConstants::Type::DATA_TEMPERATURE:
        processTemperature(array);
        ROS_DEBUG("received Temperature");
        break;
    case CommunicationConstants::Type::DATA_DISTANCE_AMPLITUDE:
        processDistanceAmplitude(array);
        ROS_DEBUG("received distance amplitude");
        break;
    case CommunicationConstants::Type::DATA_DISTANCE:
        ROS_DEBUG("received distance");
        processDistance(array);        
        break;
    case CommunicationConstants::Type::DATA_GRAYSCALE:
        processGrayscale(array);
        ROS_DEBUG("receivedGrayscale");
        break;
    case CommunicationConstants::Type::DATA_FIRMWARE_RELEASE:
        processFirmwareRelease(array);
        ROS_DEBUG("received firmware release");
        break;
    case CommunicationConstants::Type::DATA_INTEGRATION_TIME:
        processIntegrationTime(array);
        ROS_DEBUG("received integration time");
        break;
    case CommunicationConstants::Type::DATA_PRODUCTION_INFO:
        processProductionInfo(array);
        ROS_DEBUG("received production info");
        break;
    default:
        ROS_DEBUG("received unknown type= %d", type);
        break;
    }

    sigReceivedAnswer();
}


/***************************************************************************
 * Information commands --> blocking and non blocking
 ***************************************************************************/
/**
 * @brief Request the identification
 *
 * This identification helps to detect, which device is connected to the PC and if the bootloader
 * is active or the application. When the bootloader is active, the flag "isBootloader" is set.
 * In addition the version of the device is read. This is the coast version and has nothing to do with
 * the firmware version.
 *
 * @param device Reference where the device is written to
 * @param isBootloader Reference to the bootloader flag
 * @param version Reference where the version is written to
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::getIdentification(bool &isBootloader, unsigned int &version)
{
    return ErrorNumber_e::ERROR_NUMMBER_NO_ERROR;
}



/**
 * @brief Request the identification
 *
 * This identification helps to detect, which device is connected to the PC and if the bootloader
 * is active or the application. When the bootloader is active, the flage "isBootloader" is set.
 *
 * @param device Reference where the device is written to
 * @param isBootloader Reference to the bootloader flag
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::getIdentification(bool &isBootloader)
{
    unsigned int versionNotUsed;

    return getIdentification(isBootloader, versionNotUsed); //TODO...
}

/**
 * @brief Request the chip information
 *
 * @param chipId Reference to the variable to write the chip id
 * @param waferId Reference to the variable to write the wafer id
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::getChipInformation(uint16_t &chipId, uint16_t &waferId)
{
    ChipInformationHelper chipInformationHelper;

    //Temporary connect the signal to the helper
    boost::signals2::connection cn = sigReceivedChipInformation.connect(boost::bind(&ChipInformationHelper::onReceivedChipInformation, &chipInformationHelper, _1, _2));

    //Send this command blocking
    ErrorNumber_e status = sendCommandWithoutData(CommunicationConstants::CommandList::COMMAND_GET_CHIP_INFORMATION, GET_CHIP_INFORMATION_SIZE);

    //The helper has the value
    chipId = chipInformationHelper.getChipId();
    waferId = chipInformationHelper.getWaferId();

    //Disconnect the signal from the helper
    cn.disconnect();

    return status;
}

/**
 * @brief Request the firmware rlease
 *
 * @param major Reference to write the major number
 * @param minor Reference to write the minor number
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::getFirmwareRelease(unsigned int &major, unsigned int &minor)
{
    U32Helper releaseHelper;

    //Temporary connect the signal to the helper
    boost::signals2::connection cn = sigReceivedFirmwareRelease.connect(boost::bind(&U32Helper::onReceivedData, &releaseHelper, _1));

    //Send this command blocking
    ErrorNumber_e status = sendCommandWithoutData(CommunicationConstants::CommandList::COMMAND_GET_FIRMWARE_RELEASE, GET_FIRMWARE_VERSION_SIZE);

    //Disconnect the signal from the helper
    cn.disconnect();

    major = releaseHelper.getValueMsb();
    minor = releaseHelper.getValueLsb();

    return status;
}

ErrorNumber_e Communication::getTemperature(double &temperature)
{
    Int16Helper temperatureHelper;

    //Temporary connect the signal to the helper
    boost::signals2::connection cn = sigReceivedTemperature.connect(boost::bind(&Int16Helper::onReceivedData, &temperatureHelper, _1));

    //Send this command blocking
    ErrorNumber_e status = sendCommandWithoutData(CommunicationConstants::CommandList::COMMAND_GET_TEMPERATURE, GET_TEMPERATURE_SIZE);

    //Disconnect the signal from the helper
    cn.disconnect();

    temperature= static_cast<double>(temperatureHelper.getValue()) / 100.0;

    return status;
}


/***************************************************************************
 * Acquisition commands --> blocking and non blocking
 ***************************************************************************/
/**
 * @brief Request grayscale
 *
 * This function will be answered by the signal "receivedGrayscale"
 */
void Communication::getGrayscale()
{
    //int dataSize = 153635;
    int dataSize = 2 * (xMax - xMin + 1) * (yMax - yMin + 1) + 10 + 25;
    ROS_DEBUG("Communication::getGrayscale()");  //TODO remove
    sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_GET_GRAYSCALE, static_cast<uint8_t>(0), dataSize, false);
}

/**
 * @brief Request distance
 *
 * This function will be answered by the signal "receivedDistance"
 *
 * @param acquisitionMode Mode for acquisition: 0 = single, 1 = auto repeat, 3 = stream
 */
void Communication::getDistance()
{  
    //int dataSize = 153635;   //2 * 320 * 240 + 10 + 25;
    int dataSize = 2 * (xMax - xMin + 1) * (yMax - yMin + 1) + 10 + 25;

    ROS_DEBUG("Communication::getDistance()");  //TODO remove
    sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_GET_DISTANCE, static_cast<uint8_t>(0), dataSize, false);
}

/**
 * @brief Request distance and amplitude
 *
 * This function will be answered by the signal "receivedDistanceAmplitude"
 */
void Communication::getDistanceAmplitude()
{
    //int dataSize = 307235;
    int dataSize = 4 * (xMax - xMin + 1) * (yMax - yMin + 1) + 10 + 25;
    ROS_DEBUG("Communication::getDistanceAmplitude()");  //TODO remove
    sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_GET_DISTANCE_AMPLITUDE, static_cast<uint8_t>(0), dataSize, false);
}

/**
 * @brief Request dcs
 *
 * This function will be answered by the signal "receivedDcs"
 */
void Communication::getDcs()
{
    //int dataSize = 614435;
    int dataSize = 8 * (xMax - xMin + 1) * (yMax - yMin + 1) + 10 + 25;

    ROS_DEBUG("Communication::getDCS()");  //TODO remove
    sendCommandSingleByte(CommunicationConstants::CommandList::COMMAND_GET_DCS, static_cast<uint8_t>(0), dataSize, false);
}


void Communication::startStream(){
    startStreamMode = true;
}

void Communication::stopStream()
{
    sendCommandWithoutData(CommunicationConstants::CommandList::COMMAND_STOP_STREAM, false);
}

void Communication::insertValue(uint8_t *output, int index, const int16_t value)
{
    output[index]   = static_cast<int8_t>(value >> 8);
    output[index + 1] = static_cast<int8_t>(value & 0xFF);
}


void Communication::insert8Value(uint8_t *output, int index, const int8_t value)
{
    output[index] = value;
}

int Communication::boolToInt8(bool state)
{
    if(state) return 1;
    else      return 0;
}

/***************************************************************************
 * Setup commands --> blocking
 ***************************************************************************/

ErrorNumber_e Communication::setHDRMode(const uint8_t hdrMode)
{
    uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];
    memset(output, 0, sizeof(output));

    //Add the command and data
    insertValue(output, CommunicationConstants::Command::INDEX_COMMAND, CommunicationConstants::CommandList::COMMAND_SET_HDR);
    insert8Value(output, 3,  hdrMode);

    //Send blocking
    return sendCommand(output, SET_COMMAND_ANSWER_SIZE);
}



/**
 * @brief Set integration time 3D
 *
 * @param index Index of the integration time
 * @param integrationTime Integration time in us
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::setIntegrationTime3d(const int intTime1, const int intTime2, const int intTime3, const int integrationTimeGrayscale)
{
    uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];
    memset(output, 0, sizeof(output));

    //Add the command and data
    insertValue(output, CommunicationConstants::Command::INDEX_COMMAND, CommunicationConstants::CommandList::COMMAND_SET_INT_TIMES);
    insertValue(output, CommunicationConstants::IntegrationTime::INDEX_INTEGRATION_TIME_3D,     intTime1);
    insertValue(output, CommunicationConstants::IntegrationTime::INDEX_INTEGRATION_TIME_3D + 2, intTime2);
    insertValue(output, CommunicationConstants::IntegrationTime::INDEX_INTEGRATION_TIME_3D + 4, intTime3);
    insertValue(output, CommunicationConstants::IntegrationTime::INDEX_INTEGRATION_TIME_GRAYSCALE, integrationTimeGrayscale);

    //Send blocking
    return sendCommand(output, SET_INTEGRATION_TIME_DIS_SIZE);
}

/**
 * @brief Set modulation frequency
 *
 * @param modulationFrequency Selected modulation frequency
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::setModulationFrequency(const uint8_t modulationFrequency, const uint8_t channel)
{
    uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];

    memset(output, 0, sizeof(output));

    //Add the command
    insertValue(output, CommunicationConstants::Command::INDEX_COMMAND, CommunicationConstants::CommandList::COMMAND_SET_MODULATION);

    output[3] = modulationFrequency;
    output[4] = channel;
    output[5] = 0;

    //Send blocking
    return sendCommand(output, SET_MODULATION_SIZE);
}


/**
 * @brief Set the filter settings
 *
 * Factor example:
 * 300 gives 300 x actualValue + 700 x lastValue
 *
 * @param threshold Threshold where the filter is cleared
 * @param factor Factor for the actual value
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::setFilter(const bool medianFilter, const bool averageFilter, const uint16_t temporalFactor, const uint16_t temporalThreshold, const uint16_t edgeThreshold, const uint16_t interferenceDetectionLimit, const bool interferenceDetectionUseLastValue)
{
    uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];

    memset(output, 0, sizeof(output));

    //Add the command
    insertValue(output, CommunicationConstants::Command::INDEX_COMMAND, CommunicationConstants::CommandList::COMMAND_SET_FILTER);

    //Insert temporal filter factor
    insertValue(output, 3, temporalFactor);

    //Insert temporal filter threshold
    insertValue(output, 5, temporalThreshold);

    //Insert median filter
    insert8Value(output, 7, boolToInt8(medianFilter));

    //Insert average filter
    insert8Value(output, 8, boolToInt8(averageFilter));

    //Insert edge filter threshold
    insertValue(output, 9, edgeThreshold);

    //Insert interference detection use last value flag
    insert8Value(output, 11, boolToInt8(interferenceDetectionUseLastValue));

    //Insert edge filter interference detection limit
    insertValue(output, 12, interferenceDetectionLimit);

    //Insert edge filter threshold low
    insertValue(output, 14, 0);

    //Insert edge filter threshold high
    insertValue(output, 16, 0);

    //Send blocking
    return sendCommand(output, SET_FILTER_SIZE);
}

/***************************************************************************
 * Update commands --> non blocking
 ***************************************************************************/

ErrorNumber_e Communication::setOffset(const int offset)
{
    uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];
    memset(output, 0, sizeof(output));

    //Add the command
    insertValue(output, CommunicationConstants::Command::INDEX_COMMAND, CommunicationConstants::CommandList::COMMAND_SET_OFFSET);

    //Add the amplitude
    insertValue(output, 3, offset);    

    return sendCommand(output, SET_COMMAND_ANSWER_SIZE);
}

ErrorNumber_e Communication::setRoi(const unsigned int xMin, const unsigned int yMin, const unsigned int xMax, const unsigned int yMax)
{    
    this->xMin = xMin;
    this->yMin = yMin;
    this->xMax = xMax;
    this->yMax = yMax;

    uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];
    memset(output, 0, sizeof(output));

    //Add the command
    insertValue(output, CommunicationConstants::Command::INDEX_COMMAND, CommunicationConstants::CommandList::COMMAND_SET_ROI);

    //Add the amplitude
    insertValue(output, 3, xMin);
    insertValue(output, 5, yMin);
    insertValue(output, 7, xMax);
    insertValue(output, 9, yMax);

    return sendCommand(output, SET_COMMAND_ANSWER_SIZE);
}

ErrorNumber_e Communication::setMinimalAmplitude(const unsigned int amplitude){
    uint8_t output[CommunicationConstants::Command::SIZE_TOTAL];

    memset(output, 0, sizeof(output));

    //Add the command
    insertValue(output, CommunicationConstants::Command::INDEX_COMMAND, CommunicationConstants::CommandList::COMMAND_SET_MIN_AMPLITUDE);

    //Add the amplitude
    insertValue(output, CommunicationConstants::Amplitude::INDEX_AMPLITUDE, amplitude);

    return sendCommand(output, SET_COMMAND_ANSWER_SIZE);
}

//======================================================================================================


void Communication::processDistanceAmplitude(const vector<uint8_t> &array)
{
    std::shared_ptr<TofCam660Image> header(new TofCam660Image(array));
    sigReceivedDistanceAmplitude(header); //Forward the data
}

void Communication::processDistance(const vector<uint8_t> &array)
{
    std::shared_ptr<TofCam660Image> header(new TofCam660Image(array));
    sigReceivedDistance(header); //Forward the data
}


void Communication::processGrayscale(const vector<uint8_t> &array)
{
    std::shared_ptr<TofCam660Image> header(new TofCam660Image(array));
    sigReceivedGrayscale(header); //Forward the data
}


}//end namespace

/** @} */
