/**
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @defgroup serial_connection Serial Connection
 * @brief Specialized serial port
 * @ingroup communication
 *
 * @{
 */
#ifndef VIRTUAL_COM_PORT_CONNECTION_H
#define VIRTUAL_COM_PORT_CONNECTION_H

#include <vector>
#include <string>
#include <list>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <queue>
#include <mutex>
#include <condition_variable>

#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp> //?
#include "communication_constants.h"

namespace com_lib
{

class SerialConnection
{    

public:
    SerialConnection();
    ~SerialConnection();

    boost::signals2::signal< void (const std::vector<uint8_t>&, const uint8_t)>  sigReceivedData;
    std::vector<uint8_t> rxArray;

    bool openPort(std::string portName);
    void closePort();    
    int  readRxData(int size);
    ssize_t sendData(uint8_t *data);


private:

    int getExpextedSize(const std::vector<uint8_t> &array);
    uint8_t getAnswType(const std::vector<uint8_t> &array);
    uint8_t getDataType(const std::vector<uint8_t> &array);
    int setInterfaceAttribs (int speed);
    void setBlocking(int should_block);
    bool processData(std::vector<uint8_t> array);
    bool checkEndMark(std::vector<uint8_t> array);

    int expectedSize;
    int fileDescription;
    bool waitForSpecificDataType;

    uint8_t expectedType;    
    std::list<uint8_t> generalAnswerTypes;
    std::vector<std::string> deviceListString;
};


}

#endif // VIRTUAL_COM_PORT_CONNECTION_H

/** @} */
