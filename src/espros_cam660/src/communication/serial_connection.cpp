#include <ros/ros.h>
#include <communication_constants.h>
#include "serial_connection.h"
#include "util.h"
#include <termios.h>



namespace com_lib
{

SerialConnection::SerialConnection():
expectedSize(0),
fileID(0)
{    
    dataArray.resize(307325);
}

SerialConnection::~SerialConnection()
{
    if(fileID != 0)
        closePort();        

    deviceListString.clear();    

}

/**
 * @brief Open the serial port
 *
 * @param id Id of the serial port = index of the list when the function "availableDevices" is called
 * @retval true Port is open
 * @retval false Port is not open
 * @return Port open or not
 */
bool SerialConnection::openPort(std::string portName)
{  
    if(fileID > 0){
        closePort();                
        ros::Duration(0.001).sleep();
    }

    //if serial port is busy, repeat 5 times
    if((fileID = open(portName.data(), O_RDWR | O_NOCTTY | O_SYNC)) <= 0 ){
        for(int i=0; i<6; i++){
            portName = "/dev/ttyACM" + std::to_string(i);
            if((fileID = open(portName.data(), O_RDWR | O_NOCTTY | O_SYNC)) > 0){
                break;
            }
        }
    }

    if(fileID <=0 ){
        ROS_ERROR ("Error %d opening port %s: %s", errno, portName.c_str(), strerror (errno));
        throw 1;
    }

    ROS_INFO ("openPort: %s  fd= %d", portName.c_str(), fileID);
    setInterfaceAttribs(B4000000);  // set speed to 10000000 bps, 8n1 (no parity)    
    
    return true;
}

/**
 * @brief Close the port
 *
 */
void SerialConnection::closePort()
{              
    fileID = close(fileID);
    ROS_INFO("closePort: fd = %d\n", fileID);
}


/**
 * @brief Send a command
 *
 * This function sends a command. It adds the marking and the checksum and sends it
 * to the device.
 *
 * Important: The size of the buffer must be big enough to add the markings an the checksum.
 *
 * @param data Pointer to the data to send
 */
ssize_t SerialConnection::sendData(uint8_t *data)
{
    if(fileID <=0 ){
        ROS_ERROR("Error SerialConnection::sendData fileID = 0 \n");
        return 0;
    }

    //Add the start command mark at the beginning
    data[0] = com_const::Uart::START_MARK_COMMAND;

    //Add the end mark at the end
    data[35] = static_cast<uint8_t>((com_const::Uart::END_MARK >> 0) & 0xFF);
    data[36] = static_cast<uint8_t>((com_const::Uart::END_MARK >> 8) & 0xFF);
    data[37] = static_cast<uint8_t>((com_const::Uart::END_MARK >> 16) & 0xFF);
    data[38] = static_cast<uint8_t>((com_const::Uart::END_MARK >> 24) & 0xFF);

    /*std::string str= "SEND DATA: ";char buf[4];
    for(int i=0; i<39; i++){
        sprintf(buf, "%x ", data[i]); str.append(buf);
    }ROS_DEBUG_STREAM(str); //This is just to print out the data*/

    return write(fileID, (uint8_t *)(data), com_const::Command::SIZE_TOTAL);
}

/**
 * @brief Extract the expected size from the received data
 *
 * @param array Pointer to the received data
 * @return Expected size
 */
int SerialConnection::getExpextedSize(uint8_t* array)
{
    return Util::getUint32LittleEndian(array, com_const::Data::INDEX_LENGTH);
}

/**
 * @brief Extract the type from the received data
 *
 * @param array Pointer to the received data
 * @return Received type
 */
uint8_t SerialConnection::getAnswType(uint8_t* array)
{
    return array[com_const::Uart::INDEX_ANSWER_TYPE];
}

uint8_t SerialConnection::getDataType(uint8_t* array)
{
    return array[com_const::Uart::INDEX_DATA_TYPE];
}

uint8_t SerialConnection::getType()
{
    uint8_t type = 0;

    //Check if the end marking is present. Only use the data if this is the case.
    if(getAnswType(rxArray) == 1){
        type = getDataType(rxArray);

        if(type == 3) type = com_const::Type::DATA_GRAYSCALE;
        else if(type == 0) type = com_const::Type::DATA_DISTANCE_AMPLITUDE;
        else if(type == 1) type = com_const::Type::DATA_DISTANCE;

    }else{

        if(rxArray[6] == 2){
            type = 2;
        }else if(rxArray[6] == 3){
            type = 3;
        }else if(rxArray[6] == 4){
            type = 4;
        }
    }

    return type;
}



bool SerialConnection::checkEndMark(uint8_t * array, int size)
{
    int i = size - 4;

    if(array[i] == 0xbe && array[i+1] == 0xba && array[i+2] == 0xfe && array[i+3] == 0xca)
        return true;

    return false;
}



int SerialConnection::setInterfaceAttribs(int speed)
{
    tcflush(fileID, TCIOFLUSH);

    struct termios tty;
    memset (&tty, 0, sizeof tty);

    tcgetattr (fileID, &tty); //TODO...

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    // no canonical processing
    // disable IGNBRK for mismatched speed tests; otherwise receive break as \000 chars

    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_oflag &= ~(ONLCR | OCRNL); //TODO...

    tty.c_lflag = 0;                // no signaling chars, no echo,
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN); //TODO...

    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_iflag &= ~(INLCR | IGNCR | ICRNL); //TODO...

    tty.c_cc[VMIN]  = 0;            // non-blocking read
    tty.c_cc[VTIME] = 10;            // 1 second read timeout

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars                
    tty.c_cflag &= ~(PARENB | PARODD);  // shut off parity
    tty.c_cflag &= ~CSTOPB;    //one stop bit
    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls   
    tty.c_cflag |= CRTSCTS;   //data DTR hardware control do not use it

    tcflush(fileID, TCIOFLUSH);

    if (tcsetattr (fileID, TCSANOW, &tty) != 0){
        ROS_ERROR("error %d from tcsetattr", errno);
        throw 3;
    }
   
    return 0;
}


bool SerialConnection::processData(int size)
{    
    //Check for the marking byte
    if(rxArray[0] != com_const::Uart::START_MARK_DATA){
        ROS_ERROR("SerialConnection::processData error com_const::Uart::START_MARK_DATA");
        return false;
    }

    if(checkEndMark(rxArray, size) == false){
        ROS_ERROR("SerialConnection::processData error checkEndMark");
        return false;
    }

    //Get the expected size. Cancel here if not enough bytes received
    if (size < (static_cast<int>(com_const::Uart::SIZE_HEADER))){
        ROS_ERROR("SerialConnection::processData error sizeHeader");
        return false;
    }

    //Get the expexted size
    expectedSize = getExpextedSize(rxArray);

    //Cancel here if not enough bytes received
    if (size < (static_cast<int>(expectedSize + com_const::Uart::DATA_OVERHEAD_SIZE))){
        ROS_INFO("SerialConnection::processData error size= %d", size);
        return false;
    }

    uint8_t type = getType();

    if(dataArray.size() != expectedSize && expectedSize > 1){

        ROS_DEBUG("SerialConnection::processData dataArray.size()= %d  expectedSize = %d", (int)dataArray.size(), expectedSize);
        dataArray.resize(expectedSize);

    }
        
    std::copy(rxArray + com_const::Uart::SIZE_HEADER, rxArray + com_const::Uart::SIZE_HEADER + expectedSize, dataArray.begin());

    sigReceivedData(dataArray, type);

    return true;
}



/**
 * @brief Slot called on reception
 *
 * This function is called, when data from the serial port has been received.
 */
ErrorNumber_e SerialConnection::readRxData(int size)
{    
    uint8_t buf[4096];
    int n = 0;

    for(int i=0; i< size; i+=n)
    {
        memset(buf, 0, sizeof(buf)); //clear buffer

        int buf_size = size;
        if(buf_size > sizeof(buf))
            buf_size = sizeof(buf);

        n = read(fileID, buf, buf_size);

        if(n > 0){                        
            memcpy(rxArray + i, buf, n);

        }else if(n == -1){
            ROS_ERROR("Error on  SerialConnection::readRxData= -1");
            throw 6;

        }else if(n == 0 && i < size-1){

            ROS_ERROR("serialConnection->readRxData %d bytes from %d received", i, size);
            return ERROR_NUMBER_SERIAL_PORT_ERROR;
        }

    }

    processData(size);
    
    return ERROR_NUMMBER_NO_ERROR;
}



} //end namespace com_lib
