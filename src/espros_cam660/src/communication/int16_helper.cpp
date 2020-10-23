#include "int16_helper.h"

namespace com_lib
{

Int16Helper::Int16Helper()
{
    value = 0;
}

int16_t Int16Helper::getValue()
{
    return value;
}

void Int16Helper::onReceivedData(const int16_t value)
{
    this->value = value;
}


}
