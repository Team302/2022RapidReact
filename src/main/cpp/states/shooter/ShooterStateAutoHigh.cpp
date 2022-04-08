//====================================================================================================================================================
/// Copyright 2022 Lake Orion Robotics FIRST Team 302 
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// C++ Includes
#include <array>
#include <string>

// FRC includes

// Team 302 includes
#include <hw/DragonLimelight.h>
#include <hw/factories/LimelightFactory.h>
#include <states/IState.h>
#include <states/shooter/ShooterStateAutoHigh.h>
#include <subsys/MechanismFactory.h>
#include <subsys/Shooter.h>
#include <utils/Logger.h>


// Third Party Includes

using namespace std;

ShooterStateAutoHigh::ShooterStateAutoHigh
(
    ControlData*                    control, 
    ControlData*                    control2,
    double                          primaryTarget,
    double                          secondaryTarget,
    array<double,3>                 primaryFunctionCoeff,
    array<double,3>                 secondaryFunctionCoeff
) : ShooterState(control, control2, primaryTarget, secondaryTarget), 
    m_dragonLimeLight(LimelightFactory::GetLimelightFactory()->GetLimelight()), 
    m_shooterTarget(primaryTarget),
    m_shooterTarget2(secondaryTarget),
    m_primaryFunctionCoeff(primaryFunctionCoeff),
    m_secondaryFunctionCoeff(secondaryFunctionCoeff)
{
}

void ShooterStateAutoHigh::Init() 
{
    auto shooter = GetShooter();    
    if (shooter != nullptr)
    {       
        auto shooterTarget = GetPrimaryTarget();
        auto shooterTarget2 = GetSecondaryTarget();

        double inches = 90.0;
        if (m_dragonLimeLight != nullptr)
        {
            auto distance = m_dragonLimeLight->EstimateTargetDistance();
            inches = distance.to<double>();
        }
       
        shooterTarget = 0.0019*inches*inches - 0.2762*inches + 52.784; //y = 0.0019x2 - 0.2762x + 54.784
        shooterTarget2 = 0.0026*inches*inches - 0.4067*inches + 51.411; //y = 0.0026x2 - 0.4067x + 51.411


       /* if (inches > 110)
        {
            shooterTarget = 54;
            shooterTarget2 = 0.65;
        }
        else
        {
            shooterTarget = 46.75; //46
            shooterTarget2 = 0.35
            
        }*/
        /**
        auto shooterTarget = m_primaryFunctionCoeff[0]*inches*inches + 
                             m_primaryFunctionCoeff[1]*inches + 
                             m_primaryFunctionCoeff[2] +
                             GetPrimaryTarget() + 
                             indexerOffset;
        auto shooterTarget2 = m_secondaryFunctionCoeff[0]*inches*inches + 
                              m_secondaryFunctionCoeff[1]*inches + 
                              m_secondaryFunctionCoeff[2] +
                              GetSecondaryTarget() +
                              indexerOffset;
        **/
       /*
        auto logger = Logger::GetLogger();
        auto nt = shooter->GetNetworkTableName();
        auto cd = GetPrimaryControlData();
        auto cd2 = GetSecondaryControlData();

        logger->ToNtTable(nt, string("control data Mode1"), cd->GetMode());
        logger->ToNtTable(nt, string("control data Identifier1"), cd->GetIdentifier());
        logger->ToNtTable(nt, string("control data p1"), cd->GetP());
        logger->ToNtTable(nt, string("control data i1"), cd->GetI());
        logger->ToNtTable(nt, string("control data d1"), cd->GetD());
        logger->ToNtTable(nt, string("control data f1"), cd->GetF());
        logger->ToNtTable(nt, string("target 1"), shooterTarget);

        logger->ToNtTable(nt, string("control data Mode2"), cd2->GetMode());
        logger->ToNtTable(nt, string("control data Identifier2"), cd2->GetIdentifier());
        logger->ToNtTable(nt, string("control data p2"), cd2->GetP());
        logger->ToNtTable(nt, string("control data i2"), cd2->GetI());
        logger->ToNtTable(nt, string("control data d2"), cd2->GetD());
        logger->ToNtTable(nt, string("control data f2"), cd2->GetF());
        logger->ToNtTable(nt, string("target 2"), shooterTarget2);
        */
        
        shooter->SetControlConstants(0, GetPrimaryControlData());
        shooter->SetSecondaryControlConstants(0, GetSecondaryControlData());
        shooter->UpdateTargets(shooterTarget, shooterTarget2);

    }
}
