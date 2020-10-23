/**
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @defgroup communication_constants Communication Constants
 * @ingroup communication
 *
 * @brief Constants needed for communication on the firmware and on the PC.
 *
 * @{
 */
#ifndef COMMUNICATION_COMMUNICATIONCONSTANTS_H_
#define COMMUNICATION_COMMUNICATIONCONSTANTS_H_

#include <stdint.h>

//! Communication constants
/*!
 * Constants needed for communication on the firmware and on the PC.
 */


#define SINGLE 0
#define AUTO_REPEAT 1
#define STREAM 3

#define HDR_OFF 0
#define HDR_SPATIAL 1
#define HDR_TEMPORAL 2


namespace com_lib
{


enum ModulationFrequency_e
{
  MODULATION_FREQUENCY_12MHZ    = 0,
  MODULATION_FREQUENCY_24MHZ    = 1,
  MODULATION_FREQUENCY_6MHZ     = 2,
  MODULATION_FREQUENCY_3MHZ     = 3,
  MODULATION_FREQUENCY_1_5MHZ   = 4,
  MODULATION_FREQUENCY_0_75MHZ  = 5
};

enum ErrorNumber_e
{
  ERROR_NUMMBER_NO_ERROR = 0,
  ERROR_NUMBER_TIMEOUT = 32768,
  ERROR_NUMBER_NOT_ACKNOWLEDGE = 32769,
  ERROR_NUMBER_INVALID_PARAMETER = 32770,
  ERROR_NUMBER_SERIAL_PORT_ERROR = 32771,
  ERROR_NUMBER_INVALID_DATA = 32772
};
}


namespace CommunicationConstants
{

namespace AnswerType
{
  const uint32_t INDEX = 0;
  const uint8_t ANSWER_ACK = 0;
  const uint8_t ANSWER_ERROR = 1;
  const uint8_t ANSWER_FIRMWARE_RELEASE = 2;
  const uint8_t ANSWER_CHIP_INFORMATION = 3;
  const uint8_t ANSWER_NACK = 255;
}

namespace Uart
{
  const uint32_t COMMAND_BUFFER_SIZE = 1 /*Start Mark*/ + 2 /*command*/ + 32 /*payload*/ + 4 /*CRC*/;
  const uint32_t START_MARK_SIZE = 1;                                            ///<Start mark size in bytes
  const uint8_t START_MARK_COMMAND = 0xF5;                                       ///<Start marker for command (host to camera)
  const uint8_t START_MARK_DATA = 0xFA;                                          ///<Start marker for the data (camera to host)
  const uint32_t END_MARK = 0xCAFEBABE;                                          ///<End marker if no CRC is used
  const uint32_t SIZE_END_MARK = 4;
  const uint32_t SIZE_HEADER = 6;
  const uint32_t INDEX_COMMAND = 1;                                              ///<Index of the command
  const uint32_t PAYLOAD_SIZE = 32;                                              ///<Size of the payload
  const uint32_t INDEX_ANSWER_TYPE = 1;                                            ///<Index of the data type (answer or data)
  const uint32_t INDEX_DATA_TYPE = 8;                                            ///<Index of the data type (answer or data)
  const uint32_t INDEX_DATA_SIZE = 2;                                            ///<Index of the data size
  const uint32_t INDEX_DATA_PAYLOAD = 6;                                         ///<Index of the payload
  const uint8_t VALUE_DATA_ANSWER = 0;                                           ///<Data from camera to host is answer
  const uint8_t VALUE_DATA_MEASUREMENT = 1;                                      ///<Data from camera to host is measurement data
  const uint32_t DATA_OVERHEAD_SIZE = 10;                                         ///<Communication overhead size camera to host
}

namespace Data
{
const uint8_t START_MARK = 0xFA;                                           ///<Start marker for data
const uint32_t INDEX_TYPE = 1;                                             ///<Index where the type is found in the buffer
const uint32_t INDEX_LENGTH = 2;                                           ///<Index where the length is found in the buffer
const uint32_t INDEX_DATA = 4;                                             ///<Index where the data is found in the buffer
const uint32_t SIZE_HEADER = 4;                                            ///<Number of bytes for the data header
const uint32_t SIZE_CHECKSUM = sizeof(uint32_t);                           ///<Size of the checksum
const uint32_t SIZE_OVERHEAD = SIZE_HEADER + SIZE_CHECKSUM;                ///<Nuber of overhead bytes = additional bytes to payload
}

namespace Command
{
const uint8_t START_MARK = 0xF5;                                           ///<Start marker for commands
const uint32_t INDEX_COMMAND = 1;                                          ///<Index where the command is found in the buffer
const uint32_t INDEX_DATA = 3;                                             ///<Index where the data if ound in a command
const uint32_t INDEX_CHECKSUM = 10;                                        ///<Index where the checksum is found in a command
const uint32_t SIZE_TOTAL = 39;                                            ///<Number of bytes for one command
const uint32_t SIZE_PAYLOAD = 8;                                           ///<Number of bytes of the payload of a command
}

namespace Type
{
const uint8_t DATA_ACK = 0x00;                                             ///<Acknowledge from sensor to host
const uint8_t DATA_NACK = 0x01;                                            ///<Not acknowledge from sensor to host
const uint8_t DATA_IDENTIFICATION = 0x00; //02                                  ///<Identification to identify the device
const uint8_t DATA_AMPLITUDE = 0x04;                                       ///<Amplitude information
const uint8_t DATA_DISTANCE_AMPLITUDE = 0x05;                              ///<Distance and amplitude information
const uint8_t DATA_GRAYSCALE = 0x06;                                       ///<Grayscale information
const uint8_t DATA_DCS = 0x07;                                             ///<DCS data
const uint8_t DATA_DCS_DISTANCE_AMPLITUDE = 0x08;                          ///<DCS, distance and amplitude all together
const uint8_t DATA_INTEGRATION_TIME = 0x09;                                ///<Integration time, answer to COMMAND_GET_INTEGRATION_TIME_3D
const uint8_t DATA_DISTANCE = 0x0A;                              ///<Distance and grayscale data
const uint8_t DATA_LENS_CALIBRATION_DATA = 0xF7;                           ///<Lens calibration data
const uint8_t DATA_TRACE = 0xF8;                                           ///<Trace data
const uint8_t DATA_PRODUCTION_INFO = 0xF9;                                 ///<Production info
const uint8_t DATA_CALIBRATION_DATA = 0xFA;                                ///<Calibration data
const uint8_t DATA_REGISTER = 0xFB;                                        ///<Register data
const uint8_t DATA_TEMPERATURE = 0x04;                                     ///<Temperature data
const uint8_t DATA_CHIP_INFORMATION = 0x03;                                ///<Chip information data
const uint8_t DATA_FIRMWARE_RELEASE = 0x02;                                ///<Firmware release
const uint8_t DATA_ERROR = 0xFF;                                           ///<Error number
}

namespace CommandList
{
const uint16_t COMMAND_SET_ROI = 0;                                            ///<Set the ROI
const uint16_t COMMAND_SET_INT_TIMES = 1;                                      ///<Set the integration times (all at once)
const uint16_t COMMAND_GET_DISTANCE_AMPLITUDE = 2;                             ///<Get distance and amplitude (single or stream)
const uint16_t COMMAND_GET_DISTANCE = 3;                                       ///<Get distance (single or stream)
const uint16_t COMMAND_GET_GRAYSCALE = 5;                                      ///<Get grayscale (single or stream)
const uint16_t COMMAND_STOP_STREAM = 6;                                        ///<Stop the stream
const uint16_t COMMAND_GET_DCS = 7;                                            ///<Get DCS (single or stream)
const uint16_t COMMAND_SET_OFFSET = 20;                                        ///<Set the offset
const uint16_t COMMAND_SET_MIN_AMPLITUDE = 21;                                 ///<Set the minimal amplitude
const uint16_t COMMAND_SET_FILTER = 22;                                        ///<Set the filter settings (all at once)
const uint16_t COMMAND_SET_MODULATION = 23;                                    ///<Set the modulation settings
const uint16_t COMMAND_SET_BINNING = 24;                                       ///<Set the binning settings
const uint16_t COMMAND_SET_HDR = 25;                                           ///<Set the HDR settings
const uint16_t COMMAND_SET_SHUTTER_MODE = 26;                                  ///<Set the shutter mode
const uint16_t COMMAND_SET_ABS = 27;                                           ///<Set the ABS (enable/disable)
const uint16_t COMMAND_SET_COMPENSATION = 28;                                  ///<Set the compensations (enable/disable)
const uint16_t COMMAND_SET_DLL_STEP = 29;                                      ///<Set the DLL step
const uint16_t COMMAND_SHUTTER = 100;                                          ///<Force a shutter (in case of external shutter)
const uint16_t COMMAND_CALIBRATE = 30;                                         ///<Calibrate DRNU
const uint16_t COMMAND_CALIBRATE_PRODUCTION = 31;                              ///<Calibrate for production
const uint16_t COMMAND_DELETE_CALIBRATION = 32;                                ///<Delete the calibration data
const uint16_t COMMAND_DEBUG = 33;                                             ///<Debug command with different sub commands
const uint16_t COMMAND_CALIBRATE_GRAYSCALE = 34;                               ///<Calibrate Grayscale
const uint16_t COMMAND_CALIBRATE_AMBIENT_LIGHT = 35;                           ///<Calibrate Ambient Light
const uint16_t COMMAND_GET_CHIP_INFORMATION= 36;                               ///<Read chip ID and wafer ID
const uint16_t COMMAND_GET_FIRMWARE_RELEASE = 37;                              ///<Read firmware release
const uint16_t COMMAND_SET_DATA_IP_ADDRESS = 38;                               ///<Set data ip address
const uint16_t COMMAND_GET_TEMPERATURE = 74;                                   ///<Read temperature
const uint16_t COMMAND_SEND_FIRMWARE_UPDATE = 200;                             ///<Send firmware update file
}

namespace Update
{
const uint32_t INDEX_CONTROL = 2;                                          ///<Index of the control byte
const uint32_t INDEX_INDEX = 3;                                            ///<Index of the index
const uint32_t INDEX_DATA = 6;                                             ///<Index of the update data
const uint8_t CONTROL_START = 0;                                           ///<Control byte start update
const uint8_t CONTROL_WRITE_DATA = 1;                                      ///<Control byte write data
const uint8_t CONTROL_COMPLETE = 2;                                        ///<Control byte update complete
static const uint32_t PASSWORD_DELETE = 0x654321;                          ///<Password needed to prevent an accidently delete
}

namespace IntegrationTime
{
//const uint32_t INDEX_INDEX_3D = 2;                                         ///<Index of the integration time 3d index
const uint32_t INDEX_INTEGRATION_TIME_3D = 3;                              ///<Index of the integration time 3d
const uint32_t INDEX_INTEGRATION_TIME_GRAYSCALE = 9;                       ///<Index of the integration time grayscale
}

namespace Identification
{
const uint8_t CHIP_TYPE_EPC611 = 0x06;                                         ///<Chip type for EPC611
const uint8_t CHIP_TYPE_EPC635 = 0x04;                                         ///<Chip type for EPC635
const uint32_t MASK_CHIP_TYPE_DEVICE = 0x00FFFF00;                             ///<Mask out chip type and device
const uint32_t SHIFT_CHIP_TYPE_DEVICE = 8;                                     ///<Shift for chip type and device
const uint8_t DEVICE_TOFRANGE = 0x00;                                          ///<Device value for a TofRange
const uint8_t DEVICE_TOFFRAME = 0x01;                                          ///<Device value for a TofFrame
const uint8_t DEVICE_TOFCAM = 0x00;                                            ///<Device value for a TofCam

const uint32_t MASK_VERSION = 0x000000FF;                                      ///<Mask to get the version
const uint32_t SHIFT_VERSION = 0;                                              ///<Shift for the version

const uint32_t VALUE_BOOTLOADER = 0x80000000;                              ///<Or this value with the identification in the bootloader
const uint32_t VALUE_APPLICATION = 0x00000000;                             ///<Or this value with the identification in the application
}

namespace ModulationFrequency
{
const uint8_t VALUE_10MHZ = 0;                                             ///<Value for 10MHz for command "COMMAND_SET_MODULATION_FREQUENCY"
const uint8_t VALUE_20MHZ = 1;                                             ///<Value for 20MHz for command "COMMAND_SET_MODULATION_FREQUENCY"
}

namespace ChipInformation
{
const uint32_t INDEX_WAFER_ID = 6;                                         ///<Index of wafer id in data type "DATA_CHIP_INFORMATION"
const uint32_t INDEX_CHIP_ID = 4;                                          ///<Index of chip id in data type "DATA_CHIP_INFORMATION"
}
namespace Acquisition
{
const uint32_t INDEX_ACQUISITION_TYPE = 2;                                ///<Index of the acquisition type
const uint8_t VALUE_SINGLE_MEASUREMENT = 0;                               ///<The sensor makes one acquisition on command
const uint8_t VALUE_AUTO_REPEAT_MEASUREMENT = 1;                          ///<The sensor starts to make acquisitions until it gets a stop command
const uint8_t VALUE_STREAMING_MEASUREMENT = 2;                            ///<The sensor makes an acquisition, and during sending the data, the next acquisition is started already
}

namespace TofCam660Header
{
const uint32_t NUM_INTEGRATION_TIME_3D = 3;
const uint32_t SIZE = 25;                                                ///<Number of bytes for the header, 2 bytes spare
const uint32_t INDEX_VERSION = 0;
const uint32_t INDEX_FRAME_COUNTER = 1;
const uint32_t INDEX_TIMESTAMP = 3;
const uint32_t INDEX_FIRMWARE_VERSION = 5;
const uint32_t INDEX_HARDWARE_VERSION = 9;
const uint32_t INDEX_CHIP_ID = 10;
const uint32_t INDEX_WIDTH = 12;
const uint32_t INDEX_HEIGHT = 14;
const uint32_t INDEX_ORIGIN_X = 16;
const uint32_t INDEX_ORIGIN_Y = 18;
const uint32_t INDEX_CURRENT_INTEGRATION_TIME_3D = 20;
const uint32_t INDEX_CURRENT_INTEGRATION_TIME_GRAYSCALE = 24;
const uint32_t INDEX_INTEGRATION_TIME_GRAYSCALE = 26;
const uint32_t INDEX_INTEGRATION_TIME_0 = 28;
const uint32_t INDEX_INTEGRATION_TIME_1 = 30;
const uint32_t INDEX_INTEGRATION_TIME_2 = 32;
const uint32_t INDEX_AMPLITUDE_LIMIT    = 44;
const uint32_t INDEX_OFFSET = 54;
const uint32_t INDEX_BINNING = 56;
const uint32_t INDEX_DISTANCE_TEMPORAL_FILTER_FACTOR = 57;
const uint32_t INDEX_DISTANCE_TEMPORAL_FILTER_THRESHOLD = 59;
const uint32_t INDEX_SINGLE_VALUE_TEMPORAL_FILTER_FACTOR = 61;
const uint32_t INDEX_SINGLE_VALUE_TEMPORAL_FILTER_THRESHOLD = 63;
const uint32_t INDEX_MODULATION_FREQUENCY = 65;
const uint32_t INDEX_MODULATION_CHANNEL = 66;
const uint32_t INDEX_FLAGS = 67;
const uint32_t INDEX_TEMPERATURE = 69;
const uint32_t INDEX_ILLUMINATION_BEAM = 71;
const uint32_t INDEX_BEAM_B_DISTANCE = 72;
const uint32_t INDEX_BEAM_B_AMPLITUDE = 74;
}

namespace ROI
{
const uint32_t INDEX_ROI_X_MIN = 2;
const uint32_t INDEX_ROI_Y_MIN = 4;
const uint32_t INDEX_ROI_X_MAX = 6;
const uint32_t INDEX_ROI_Y_MAX = 8;
}

namespace PixelTofCam660
{
const uint32_t LIMIT_VALID_PIXEL = 16000;
const uint32_t VALUE_LOW_AMPLITUDE = 16001;
const uint32_t VALUE_ADC_OVERFLOW = 16002;
const uint32_t VALUE_SATURATION = 16003;
const uint16_t MASK_OUT_CONFIDENCE = 0x3FFF;
}

namespace Flags
{
const uint32_t MASK_AUTO_MODULATION = 0x1;
const uint32_t MASK_AUTO_INTEGRATION_TIME = 0x2;
const uint32_t MASK_DCS_FILTER = 0x4;
const uint32_t MASK_GAUSSIAN_FILTER = 0x8;
const uint32_t MASK_COMPENSATED = 0x10;
}

namespace Amplitude
{
const uint32_t INDEX_INDEX = 2;                                          ///<Index of the index
const uint32_t INDEX_AMPLITUDE = 3;                                      ///<Index of the minimal amplitude
}

}

#endif /* COMMUNICATION_COMMUNICATIONCONSTANTS_H_ */
