
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
/// Logger.cpp
//========================================================================================================
///
/// File Description:
///     This logs error messages, and can also be used to log non-error events.
///
//========================================================================================================


// C++ Includes
#include <algorithm>
#include <iostream>
#include <locale>
#include <string>

// FRC includes
#include <frc/SmartDashboard/SendableChooser.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

// Team 302 includes
#include <utils/Logger.h>


// Third Party Includes

using namespace frc;
using namespace std;

/// @brief constructor that creates/initializes the object
Logger::Logger() : m_option( LOGGER_OPTION::EAT_IT ),
                   m_level( LOGGER_LEVEL::PRINT ),
                   m_alreadyDisplayed(),
                   m_OptionChooser(),
                   m_LevelChooser()
{
}

/// @brief Find or create the singleton logger
/// @returns Logger* pointer to the logger
Logger* Logger::m_instance = nullptr;
Logger* Logger::GetLogger()
{
    if ( Logger::m_instance == nullptr )
    {
        Logger::m_instance = new Logger();
    }
    return Logger::m_instance;
}

/// @brief Display/select logging options/levels on dashboard
void Logger::PutLoggingSelectionsOnDashboard()
{
    // set up option menu
    m_OptionChooser.SetDefaultOption("EAT_IT", LOGGER_OPTION::EAT_IT);
    m_OptionChooser.AddOption("DASHBOARD", LOGGER_OPTION::DASHBOARD);
    m_OptionChooser.AddOption("CONSOLE", LOGGER_OPTION::CONSOLE);
    frc::SmartDashboard::PutData("Logging Options", &m_OptionChooser);

    // set up level menu
    m_LevelChooser.SetDefaultOption("ERROR_ONCE", LOGGER_LEVEL::ERROR_ONCE);
    m_LevelChooser.AddOption("ERROR", LOGGER_LEVEL::ERROR);
    m_LevelChooser.AddOption("WARNING_ONCE", LOGGER_LEVEL::WARNING_ONCE);
    m_LevelChooser.AddOption("WARNING", LOGGER_LEVEL::WARNING);
    m_LevelChooser.AddOption("PRINT_ONCE", LOGGER_LEVEL::PRINT_ONCE);
    m_LevelChooser.AddOption("PRINT", LOGGER_LEVEL::PRINT);
    frc::SmartDashboard::PutData("Logging Levels", &m_LevelChooser);

    m_CyclingCounter = 0;
}

/// @brief Read logging option from dashboard, but not every 20ms
void Logger::PeriodicLog()
{
    m_CyclingCounter += 1;          // count 20ms loops
    if (m_CyclingCounter >= 25)     // execute every 500ms
    {
        m_CyclingCounter = 0;

        //
        // Check for a new option selection
        //
        LOGGER_OPTION SelectedOption = m_OptionChooser.GetSelected();

        if (SelectedOption != m_option)
        {
            cout << "SelectedOption = " << SelectedOption;    // print integer value
            m_option = SelectedOption;

            switch(SelectedOption)
            {
                case CONSOLE:
                    cout << " CONSOLE" << endl;
                    break;

                case DASHBOARD:
                    cout << " DASHBOARD" << endl;
                    break;

                case EAT_IT:
                    cout << " EAT_IT" << endl;
                    break;

                default:
                    cout << " Out of range !" << endl;
                    m_option = EAT_IT;
                    break;
            }
        }

        //
        // Check for a new level selection
        //
        LOGGER_LEVEL SelectedLevel = m_LevelChooser.GetSelected();

        if (SelectedLevel != m_level)
        {
            cout << "SelectedLevel = " << SelectedLevel;    // print integer value
            m_level = SelectedLevel;

            switch(SelectedLevel)
            {
                case ERROR_ONCE:
                    cout << " ERROR_ONCE" << endl;
                    break;

                case ERROR:
                    cout << " ERROR" << endl;
                    break;

                case WARNING_ONCE:
                    cout << " WARNING_ONCE" << endl;
                    break;

                case WARNING:
                    cout << " WARNING" << endl;
                    break;

                case PRINT_ONCE:
                    cout << " PRINT_ONCE" << endl;
                    break;

                case PRINT:
                    cout << " PRINT" << endl;
                    break;

                default:
                    cout << " Out of range !" << endl;
                    m_level = WARNING;
                    break;
            }
        }
    }
}

/// @brief Log a message indicating the code has reached a given point
/// @param [in] std::string: message indicating location in code
void Logger::Arrived_at
(
    const std::string&   message
)
{
    LogError( LOGGER_LEVEL::PRINT, "Arrived_at ", message );
}

/// @brief set the option for where the logging messages should be displayed
/// @param [in] LOGGER_OPTION:  logging option for where to log messages
void Logger::SetLoggingOption
(
    LOGGER_OPTION option
)
{
    m_option = option;
}

/// @brief set the level for messages that will be displayed
/// @param [in] LOGGER_LEVEL:  logging level for which messages to display
void Logger::SetLoggingLevel
(
    LOGGER_LEVEL level
)
{
    m_level = level;
}

/// @brief log a message
/// @param [in] std::string: classname or object identifier
/// @param [in] std::string: message
void Logger::LogError       // calling LogError() without a LOGGER_LEVEL defaults to PRINT
(
    const string&   locationIdentifier,
    const string&   message
)
{
    LogError( LOGGER_LEVEL::PRINT, locationIdentifier, message );
}

/// @brief log a message
/// @param [in] LOGGER_LEVEL: message level
/// @param [in] std::string: classname or object identifier
/// @param [in] std::string: message
void Logger::LogError
(
    LOGGER_LEVEL    level,
    const string&   locationIdentifier,
    const string&   message
)
{
    //  Is the level important enough to display?
    //  (Lower numeric values are more important)
    if ( level <= m_level )
    {
        auto display = true;
        // If the error level is *_ONCE, display it only the first time it happens
        if ((level == ERROR_ONCE) || (level == WARNING_ONCE) || (level == PRINT_ONCE))
        {
            string key = locationIdentifier + message;
            auto it = m_alreadyDisplayed.find(key);
            display = ( it == m_alreadyDisplayed.end() );   // Display if it can't find the key
            if (display)
            {
                m_alreadyDisplayed.insert(key);             // and save the key
            }
        }

        if (display)
        {
            switch ( m_option )
            {
                case LOGGER_OPTION::CONSOLE:
                    cout << locationIdentifier << ": " << message << endl;
                    break;

                case LOGGER_OPTION::DASHBOARD:
                    SmartDashboard::PutString( locationIdentifier.c_str(), message.c_str());
                    break;

                case LOGGER_OPTION::EAT_IT:
                    // keep quiet
                    break;

                default:
                    cout << "Logger: Invalid m_option = " << m_option << endl;
                    m_option = EAT_IT;
                    break;
            }
        }
    }
}

/// @brief Write a message and value to the dashboard, for monitoring the value
/// @param [in] std::string: classname or object identifier
/// @param [in] double: value that should be written (int or bool could also be 'doubled' and sent)
void Logger::OnDash
(
    const string&   locationIdentifier,     // <I> - classname or object identifier
    double          val                     // <I> - numerical value
)
{
    if (m_option != Logger::LOGGER_OPTION::EAT_IT)
    {
        SmartDashboard::PutNumber( locationIdentifier.c_str(), val );
    }
}

//
//  ToNtTable() sends an identifier and information (string or value) to the specified network table.
//  Anything on the network can then read the message.
//
void Logger::ToNtTable
(
    const std::string&  ntName,
    const std::string&  identifier,
    const std::string&  msg
)
{
    if (m_option != Logger::LOGGER_OPTION::EAT_IT)
    {
        auto table = nt::NetworkTableInstance::GetDefault().GetTable(ntName);
        table.get()->PutString(identifier, msg);
    }
}

void Logger::ToNtTable
(
    const std::string&  ntName,
    const std::string&  identifier,
    double              value
)
{
    if (m_option != Logger::LOGGER_OPTION::EAT_IT)
    {
        auto table = nt::NetworkTableInstance::GetDefault().GetTable(ntName);
        table.get()->PutNumber(identifier, value);
    }
}

void Logger::ToNtTable
(
    std::shared_ptr<nt::NetworkTable>   ntable,
    const std::string&                  identifier,
    const std::string&                  msg
)
{
    if (m_option != Logger::LOGGER_OPTION::EAT_IT)
    {
        ntable.get()->PutString(identifier, msg);
    }
}

void Logger::ToNtTable
(
    std::shared_ptr<nt::NetworkTable>   ntable,
    const std::string&                  identifier,
    double                              value
)
{
    if (m_option != Logger::LOGGER_OPTION::EAT_IT)
    {
        ntable.get()->PutNumber(identifier, value);
    }
}
