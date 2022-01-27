
//====================================================================================================================================================
// Copyright 2020 Lake Orion Robotics FIRST Team 302 
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


#include <frc/geometry/Pose2d.h>  //pose2d
#include <frc/geometry/Rotation2d.h>

#pragma once

class DragonTargetFinder
{
    public: frc::Rotation2d GetCurrentRotaion(frc::Pose2d);
    public: frc::Transform2d GetDistance2TargetXYR(frc::Pose2d);

    public: int GetFieldQuadrant(frc::Pose2d lCurPose);
    public: frc::Pose2d GetPosCenterTarget;
    
    public: double GetAngle2Target(frc::Pose2d lCurPose);
    public: double GetDistance2TargetHyp(frc::Pose2d);


    private:
      double PI = 3.14159265358979323846;
      frc::Pose2d PosCenterTarget =  frc::Pose2d(8.212_m, 4.162_m,0_deg);
};