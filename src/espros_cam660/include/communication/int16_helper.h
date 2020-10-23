/**
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @defgroup int16_helper 16Bit Helper
 * @brief Helper class for 16Bit values
 * @ingroup communication
 *
 * @{
 */
#ifndef INT16HELPER_H
#define INT16HELPER_H

#include <stdint.h>

namespace com_lib
{
//! Helper class for 16Bit values
/*!
 * This class helps, when 16Bit data is being read blocking. It receives the signals and
 * stores the value. Later it can be read using the getter functions. There are functions for the
 * lsb/msb only or for the whole value.
 */

class Int16Helper
{
  public:
    Int16Helper();
    int16_t getValue();
    void onReceivedData(const int16_t value); //boost slot

  private:
    int16_t value;
};

}

#endif // INT16HELPER_H

/** @} */
