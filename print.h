#pragma once

#include <stdio.h>
#include "SDFat/common/PrintInterface.h"

class PrintStdio : public PrintInterface
{
public:
    size_t write(uint8_t ch)
    {
        return printf("%c", (char)ch);
    }
};