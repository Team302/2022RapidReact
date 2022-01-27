/*========================================================================================================
 * IDragonTargetFinder.h
 *========================================================================================================
 *
 * File Description:  Routines to point robot toward center of targets
 *
 *========================================================================================================*/

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

#include <states/chassis/DragonTargetFinder.h>

frc::Rotation2d DragonTargetFinder::GetCurrentRotaion(frc::Pose2d lCurPose)
{
    frc::Rotation2d CurrentRotaion = (lCurPose.Rotation());  //Current rotation pos in Sin Cos
    return CurrentRotaion;
}

frc::Transform2d DragonTargetFinder::GetDistance2TargetXYR(frc::Pose2d lCurPose)
{
   frc::Transform2d Distance2Target = PosCenterTarget - lCurPose ;
   return Distance2Target;

}

int DragonTargetFinder::GetFieldQuadrant(frc::Pose2d lCurPose)
{ 
    //  What quadruarnt is the robot in based on center of target               
//                  |
//               II |   I
//             -----+------
//              III |   IV
//                  |
    int i=0;
        if (lCurPose.X() > PosCenterTarget.X() && lCurPose.Y() > PosCenterTarget.Y()){i = 1;}   // -180 thru -90
        if (lCurPose.X() < PosCenterTarget.X() && lCurPose.Y() > PosCenterTarget.Y()){i = 2;}   // 0 thru -90
        if (lCurPose.X() < PosCenterTarget.X() && lCurPose.Y() < PosCenterTarget.Y()){i = 3;}   // 0 thru 90
        if (lCurPose.X() > PosCenterTarget.X() && lCurPose.Y() < PosCenterTarget.Y()){i = 4;}   // 90  thru 180
    return i;
}

double DragonTargetFinder::GetAngle2Target(frc::Pose2d lCurPose)
{
        // return angle to target  180deg thru -180deg  
        // 0 Degrees is pointing at center of target based on field position
        frc::Transform2d Distance2Target = GetDistance2TargetXYR(lCurPose);
        double dDistX2Target = Distance2Target.X().to<double>();
        double dDistY2Target = Distance2Target.Y().to<double>();
        //formulate a vector based on Distance to target
        double dHypotenuse = sqrt(dDistX2Target*dDistX2Target + dDistY2Target*dDistY2Target);
        // c = dhypotenuse
        // a= dDistY2Target
        // b= dDistX2Target
        // α = arcsin(a / c)
        // β = arcsin(b / c)

        double dAngleARad = dDistY2Target / dHypotenuse;
        double dAngleAA = asin(dAngleARad);

        double dAngleBRad = dDistX2Target / dHypotenuse; 
        double dAngleBB = asin(dAngleBRad);

        //Chassis Quadarant location based on radians to target.  ///////////////
        int iQuadrantsLoc = 0;   // Quadrants I,II,III,IV.  Standard radians rotation counter clockwise

        double dDeg2Target = (dAngleAA * (180.0/PI));  //convert rad to degrees.
        double dDeg2TargetB = (dAngleBB * (180.0/PI));

        if ((dAngleAA) < 0 && (dAngleBB > 0)) {iQuadrantsLoc = 1;}//neg quadraunt
        if ((dAngleAA) < 0 && (dAngleBB < 0)) {iQuadrantsLoc = 2;dDeg2Target = -90 + dDeg2TargetB;}//neg quadraunt
        if ((dAngleAA) > 0 && (dAngleBB < 0)) {iQuadrantsLoc = 3;dDeg2Target = 90 + abs(dDeg2TargetB);}//Pos quadraunt
        if ((dAngleAA) > 0 && (dAngleBB > 0)) {iQuadrantsLoc = 4;}//Pos quadraunt
       // bool bSpinNegToTarget = false;
       // if (dDeg2Target < 0)
       // {
       // bSpinNegToTarget = true;
       // }
        /////////////////////////////////////////////////////////////////////

    return dDeg2Target;

}

double DragonTargetFinder::GetDistance2TargetHyp(frc::Pose2d lCurPose)
{
        // return distance to target straight line "Hypotenuse"  
        frc::Transform2d Distance2Target = GetDistance2TargetXYR(lCurPose);
        double dDistX2Target = Distance2Target.X().to<double>();
        double dDistY2Target = Distance2Target.Y().to<double>();
        //formulate a vector based on Distance to target
        double dHypotenuse = sqrt(dDistX2Target*dDistX2Target + dDistY2Target*dDistY2Target);
 
    return dHypotenuse;

}



