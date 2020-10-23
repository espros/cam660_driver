#include <ros/ros.h>
#include <communication_constants.h>
#include "serial_connection.h"
#include "util.h"
#include <termios.h>

#define termios asmtermios
#define winsize asmwinsize
#define termio asmtermio

#undef  termios
#undef  winsize
#undef  termio


typedef boost::asio::serial_port_base asio_serial;
using namespace std;

namespace com_lib
{

SerialConnection::SerialConnection()
{  
    expectedSize = 0;
    fileDescription = 0;
}

SerialConnection::~SerialConnection()
{
    if(fileDescription != 0 ){
        closePort();
        ROS_INFO("Close serial port"); //TODO remove
    }

    deviceListString.clear();
    rxArray.clear();
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
    if(fileDescription > 0){
        closePort();                
        ros::Duration(1).sleep();
    }

    //is serial port is busy, repeat 5 times
    if((fileDescription = open(portName.data(), O_RDWR | O_NOCTTY | O_SYNC)) <= 0 ){
        for(int i=0; i<6; i++){
            portName = "/dev/ttyACM" + std::to_string(i);
            if((fileDescription = open(portName.data(), O_RDWR | O_NOCTTY | O_SYNC)) > 0){
                break;
            }
        }
    }

    if(fileDescription <=0 ){
        ROS_ERROR ("Error %d opening %s: %s", errno, portName.c_str(), strerror (errno));
        return false;
    }

    ROS_INFO ("Open serial port: %s %d", portName.c_str(), fileDescription);
    setInterfaceAttribs(B4000000);  // set speed to 10000000 bps, 8n1 (no parity)    
    rxArray.clear();
    return true;
}

/**
 * @brief Close the port
 *
 */
void SerialConnection::closePort()
{              
    fileDescription = close(fileDescription);
    ROS_DEBUG("ClosePort: fd = %d\n", fileDescription);
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
    if(fileDescription <=0 ){
        ROS_ERROR("Error SerialConnection::sendData fileDescription =0 \n");
        return 0;
    }

    //Add the start command mark at the beginning
    data[0] = CommunicationConstants::Uart::START_MARK_COMMAND;

    //Add the end mark at the end
    data[35] = static_cast<uint8_t>((CommunicationConstants::Uart::END_MARK >> 0) & 0xFF);
    data[36] = static_cast<uint8_t>((CommunicationConstants::Uart::END_MARK >> 8) & 0xFF);
    data[37] = static_cast<uint8_t>((CommunicationConstants::Uart::END_MARK >> 16) & 0xFF);
    data[38] = static_cast<uint8_t>((CommunicationConstants::Uart::END_MARK >> 24) & 0xFF);

    std::string str= "SEND DATA: ";
    char buf[4];
    for(int i=0; i<39; i++){ //This is just to print out the data
        sprintf(buf, "%x ", data[i]);
        str.append(buf);
    }

    ROS_DEBUG_STREAM(str);

    return write(fileDescription, (uint8_t *)(data), CommunicationConstants::Command::SIZE_TOTAL);
}


/**
 * @brief Extract the expected size from the received data
 *
 * @param array Pointer to the received data
 * @return Expected size
 */
int SerialConnection::getExpextedSize(const std::vector<uint8_t> &array)
{
    int expectedSize = Util::getUint32LittleEndian(array, CommunicationConstants::Data::INDEX_LENGTH);
    return expectedSize;
}

/**
 * @brief Extract the type from the received data
 *
 * @param array Pointer to the received data
 * @return Received type
 */
uint8_t SerialConnection::getAnswType(const vector<uint8_t> &array)
{
    return array.at(CommunicationConstants::Uart::INDEX_ANSWER_TYPE);
}

uint8_t SerialConnection::getDataType(const std::vector<uint8_t> &array)
{
    return array.at(CommunicationConstants::Uart::INDEX_DATA_TYPE);
}

/**
 * @brief Slot called on reception
 *
 * This function is called, when data from the serial port has been received.
 */
int SerialConnection::readRxData(int size)
{    
    uint8_t buf[320000] = {0};
    int n = read(fileDescription, buf, size);

    if(n == -1){
        ROS_ERROR("Error on  SerialConnection::readRxData");
        return -1;
    }
    rxArray.insert(std::end(rxArray), buf, buf + n); //Append the new data to the rxArray buffer

    for(int i=0; i< rxArray.size(); i++ ){
        if(i<100 || i>rxArray.size()-100)
        ROS_DEBUG("SerialConnection::readRxData  rxArray[%d]= %x", i, rxArray[i]); //TODO remove
    }

    processData(rxArray);
    return n;
}

/**
 * @brief Process the received data
 *
 * @param array Pointer to the received byte array
 */
bool SerialConnection::processData(std::vector<uint8_t> array)
{        
    if (array.size() == 0){
        ROS_ERROR("ERROR SerialConnection::processData array.size() = 0");
        return false;
    }

    //Check for the marking byte
    if(array.at(0) != CommunicationConstants::Uart::START_MARK_DATA){
        array.erase(array.begin(), array.begin() + 1 );
        return true;
    }

    if(checkEndMark(array) == false)
        return true;

    int k = array.size() - 6; //TODO remove
    int zz = array.size();    
    ROS_DEBUG("SerialConnection::processData: size = %d  data= %x %x %x %x %x %x %x %x %x %x %x %x %x  %x %x %x %x %x %x %x %x", zz, array[0], array[1], array[2], array[3], array[4], array[5], array[6], array[7],  array[8], array[9], array[10], array[11], array[12], array[k-2], array[k-1], array[k], array[k+1], array[k+2], array[k+3], array[k+4], array[k+5]); //TODO remove

    //Cancel here if no marking bytes
    //Get the expected size. Cancel here if not enough bytes received
    if (array.size() < (static_cast<int>(CommunicationConstants::Uart::SIZE_HEADER)))
        return true;

    //Get the expexted size
    expectedSize = getExpextedSize(array);

    //Cancel here if not enough bytes received
    if (array.size() < (static_cast<int>(expectedSize + CommunicationConstants::Uart::DATA_OVERHEAD_SIZE)))
        return true;

    //Check if the end marking is present. Only use the data if this is the case.
    uint8_t answer_type = getAnswType(array);
    uint8_t type = 0;

    if(answer_type == 1){
        type = getDataType(array);

        if(type== 3) type = CommunicationConstants::Type::DATA_GRAYSCALE;
        else if(type == 0) type = CommunicationConstants::Type::DATA_DISTANCE_AMPLITUDE;
        else if(type == 1) type = CommunicationConstants::Type::DATA_DISTANCE;

    }else{

        if(array.at(6) == 2){
            type = 2;
        }else if(array.at(6) == 3){
            type = 3;
        }else if(array.at(6) == 4){
            type = 4;
        }
    }

    vector<uint8_t> dataArray = array;
    dataArray.erase(dataArray.begin(), dataArray.begin() + CommunicationConstants::Uart::SIZE_HEADER);

    //Remove end mark at the end
    dataArray.erase(dataArray.begin() + expectedSize,  dataArray.begin() + expectedSize + CommunicationConstants::Uart::SIZE_END_MARK);
    dataArray.erase(dataArray.begin() + expectedSize, dataArray.end());

    //Remove the remaining: thats the checksum
    array.erase(array.begin(), array.begin() + (expectedSize + CommunicationConstants::Uart::SIZE_END_MARK) );
    sigReceivedData(dataArray, type);

    array.clear();

    return false;
}

bool SerialConnection::checkEndMark(std::vector<uint8_t> array)
{
    int i = array.size() - 4;

    if(array[i] == 0xbe && array[i+1] == 0xba && array[i+2] == 0xfe && array[i+3] == 0xca){   
        return true;
    }

    return false;
}

int SerialConnection::setInterfaceAttribs(int speed)
{
    tcflush(fileDescription, TCIOFLUSH);

    struct termios tty;
    memset (&tty, 0, sizeof tty);

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars

    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,

    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // non-blocking read
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls

    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    //tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fileDescription, TCSANOW, &tty) != 0){
        ROS_ERROR("Error %d from tcsetattr", errno);
        return -1;
    }

    ROS_DEBUG("setInterfaceAttribs fd= %d", fileDescription);

    return 0;
}


void SerialConnection::setBlocking(int should_block){
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fileDescription, &tty) != 0){
        ROS_ERROR("Error %d from tggetattr", errno);
        return;
    }
    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 30; // 30 seconds read timeout
    if (tcsetattr (fileDescription, TCSANOW, &tty) != 0)
        ROS_ERROR ("Error %d setting term attributes", errno);
}


} //end namespace com_lib
