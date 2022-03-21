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

//C++ Includes
#include <memory>
#include <string>

#include <frc/Filesystem.h>

//Team 302 includes
#include <auton/primitives/ResetPosition.h>
#include <auton/PrimitiveParams.h>
#include <auton/primitives/IPrimitive.h>
#include <subsys/ChassisFactory.h>
#include <hw/factories/PigeonFactory.h>
#include <utils/Logger.h>

using namespace std;
using namespace frc;

ResetPosition::ResetPosition() : m_chassis(ChassisFactory::GetChassisFactory()->GetIChassis())
{
}

void ResetPosition::Init(PrimitiveParams* params)
{
    string pathToLoad = params->GetPathName();

    if (pathToLoad != "")
    {
 	    auto deployDir = frc::filesystem::GetDeployDirectory();
        deployDir += "/paths/" + pathToLoad;
        m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDir);

        m_chassis->ResetPosition(m_trajectory.InitialPose());

        Logger::GetLogger()->ToNtTable(string("Auton Info"), string("ResetPosX"), m_chassis.get()->GetPose().X().to<double>());
        Logger::GetLogger()->ToNtTable(string("Auton Info"), string("ResetPosY"), m_chassis.get()->GetPose().Y().to<double>());
        Logger::GetLogger()->ToNtTable(string("Auton Info"), string("InitialPoseX"), m_trajectory.InitialPose().X().to<double>());
        Logger::GetLogger()->ToNtTable(string("Auton Info"), string("InitialPoseY"), m_trajectory.InitialPose().Y().to<double>());
        Logger::GetLogger()->ToNtTable(string("Auton Info"), string("InitialPoseOmega"), m_trajectory.InitialPose().Rotation().Degrees().to<double>());
        
    }
}

void ResetPosition::Run()
{

}

bool ResetPosition::IsDone()
{
    return true;
}