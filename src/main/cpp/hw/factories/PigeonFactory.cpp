
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

//========================================================================================================
/// @class PigeonFactory
/// @brief This creates the pigeon
//========================================================================================================

// C++ Includes
#include <algorithm>
#include <locale>
#include <map>
#include <memory>
#include <string>

// FRC includes

// Team 302 includes
#include <hw/factories/PigeonFactory.h>
#include <hw/DragonPigeon.h>
#include <utils/Logger.h>


// Third Party Includes


using namespace std;

/// @brief  Find or create the analog input factory
/// @return PigeonFactory* pointer to the factory
PigeonFactory* PigeonFactory::m_factory = nullptr;
PigeonFactory* PigeonFactory::GetFactory()
{
	if ( PigeonFactory::m_factory == nullptr )
	{
		PigeonFactory::m_factory = new PigeonFactory();
	}
	return PigeonFactory::m_factory;
}

PigeonFactory::PigeonFactory()
{
    m_centerPigeon = nullptr;
    m_shooterPigeon = nullptr;
}


/// @brief  Create the requested analog input
/// @return shared_ptr<DragonPigeon>   the mechanism or nullptr if mechanism doesn't 
///         exist and cannot be created.
DragonPigeon* PigeonFactory::CreatePigeon
(
    int 	canID,
    DragonPigeon::PIGEON_TYPE type, 
    DragonPigeon::PIGEON_USAGE usage, 
    double  rotation
)
{
    switch (usage)
    {
        case DragonPigeon::PIGEON_USAGE::CENTER_OF_ROBOT:
            if (m_centerPigeon == nullptr)
            {
                m_centerPigeon = new DragonPigeon( canID, usage, type, rotation );
            }
            return m_centerPigeon;
            break;
        case DragonPigeon::PIGEON_USAGE::CENTER_OF_SHOOTER:
            if (m_shooterPigeon == nullptr)
            {
                m_shooterPigeon = new DragonPigeon( canID, usage, type, rotation );
            }
            return m_shooterPigeon;
            break;
        default:
            break;
    }
    return nullptr;
}

DragonPigeon* PigeonFactory::GetPigeon
(
    DragonPigeon::PIGEON_USAGE usage 
) const
{
    switch (usage)
    {
        case DragonPigeon::PIGEON_USAGE::CENTER_OF_ROBOT:
            return m_centerPigeon;
            break;
        case DragonPigeon::PIGEON_USAGE::CENTER_OF_SHOOTER:
            return m_shooterPigeon;
            break;
        default:
            break;
    }
    return nullptr;
}
