
//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#pragma once

// C++ Includes
#include <memory>

// FRC includes

// Team 302 includes
#include <hw/DragonDigitalInput.h>

// Third Party includes
#include <pugixml/pugixml.hpp>


class DigitalInputDefn
{
    public:

        DigitalInputDefn() = default;
        virtual ~DigitalInputDefn() = default;

        //-----------------------------------------------------------------------
        // Method:      ParseXML
        // Description: Parse a motor XML element and create a DragonTalon from
        //              its definition.
        //
        //<!-- ====================================================
        //     digitalInput
        //     ====================================================
        //        enum DIGITAL_INPUT_TYPE
        //        {
        //            UNKNOWN_DIGITAL_INPUT_TYPE = -1,
        //            CARGO_PRESENT,
        //            HATCH_PRESENT_SIDE1,
        //            HATCH_PRESENT_SIDE2,
        //            ARM_MIN_POSITION,
        //            ARM_MAX_POSITION,
        //			      FRONT_LEFT_LINE_DETECT,
        //			      FRONT_RIGHT_LINE_DETECT,
        //			      BACK_LEFT_LINE_DETECT,
        //			      BACK_RIGHT_LINE_DETECT,
        //            MAX_DIGITAL_INPPUT_TYPES
        //        };
        //
        //     ==================================================== -->
        //<!ELEMENT digitalInput EMPTY>
        //<!ATTLIST digitalInput
        //          usage             (  0 |  1 |  2 |  3 |  4 | 5 | 6 | 7 | 8 ) "0"
        //          digitalId         (  0 |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 |
        //                              10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 |
        //                              20 | 21 | 22 | 23 | 24 | 25 ) "0"
        //          reversed          (true | false ) "false"
        //>
        //
        //
        // Returns:     DragonDigitalInput*
        //-----------------------------------------------------------------------
        std::shared_ptr<DragonDigitalInput> ParseXML
        (
            pugi::xml_node      DigitalNode
        );

};
