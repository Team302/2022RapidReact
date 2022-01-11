/*
 * DragonServo.cpp
 */

#include <hw/DragonServo.h>
#include <hw/usages/ServoUsage.h>

#include <frc/Servo.h>

using namespace frc;

DragonServo::DragonServo
(
	ServoUsage::SERVO_USAGE	    deviceUsage,		// <I> - Usage of the servo
	int 						deviceID,			// <I> - PWM ID
	double 						minAngle,			// <I> - Minimun desired angle
	double						maxAngle			// <I> - Maximum desired angle

) : m_usage( deviceUsage ),
    m_servo(new frc::Servo(deviceID)),
	m_minAngle( minAngle ),
	m_maxAngle( maxAngle )
{
    
}

void DragonServo::Set(double value)
{
    if ( m_servo != nullptr )
    {
        m_servo->Set( value );
    }
}
void DragonServo::SetOffline()
{
    if ( m_servo != nullptr )
    {
        m_servo->SetOffline();
    }
}
double DragonServo::Get() const
{
    double value = 0.0;
    if ( m_servo != nullptr )
    {
        value = m_servo->Get();
    }
    return value;
}
void DragonServo::SetAngle(double angle)
{
    if ( m_servo != nullptr )
    {
        m_servo->SetAngle( angle );
    }
}
double DragonServo::GetAngle() const
{
    double angle = 0.0;
    if ( m_servo != nullptr )
    {
        angle = m_servo->GetAngle();
    }
    return angle;
}

void DragonServo::MoveToMaxAngle()
{
	SetAngle( m_maxAngle );
}

void DragonServo::MoveToMinAngle()
{
	SetAngle( m_minAngle );
}

ServoUsage::SERVO_USAGE DragonServo::GetUsage() const
{
    return m_usage;
}