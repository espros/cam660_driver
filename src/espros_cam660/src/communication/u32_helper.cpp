#include "u32_helper.h"
#include <iostream>
#include <ros/ros.h> //TODO remove

namespace com_lib
{

U32Helper::U32Helper()
{
  value = 0;
}

uint32_t U32Helper::getValue()
{
  return value;
}

void U32Helper::onReceivedData(const uint32_t value)
{  
  this->value = value;
}

uint16_t U32Helper::getValueLsb()
{
  return value & 0xffff;
}

uint16_t U32Helper::getValueMsb()
{
  return (value >> 16) & 0x0ffff;
}

}
