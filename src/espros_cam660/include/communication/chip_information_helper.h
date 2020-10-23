/**
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @defgroup chip_information_helper Chip Information Helper
 * @brief Helper class for the  Chip Information
 * @ingroup communication
 *
 * @{
 */
#ifndef CHIPINFORMATIONHELPER_H
#define CHIPINFORMATIONHELPER_H

#include <stdint.h>

namespace com_lib
{

//! Helper class for Chip Information
/*!
 * This class helps, when the Chip Information is being read blocking. It receives the signals and
 * stores the values. Later they can be read using the getter functions.
 */
class ChipInformationHelper
{

public:
    ChipInformationHelper();    
    uint16_t getWaferId();
    uint16_t getChipId();

  //public slots
    void onReceivedChipInformation(const uint16_t chipId, const uint16_t waferId);

private:
    uint16_t waferId;
    uint16_t chipId;
};

}

#endif // CHIPINFORMATIONHELPER_H

/** @} */
