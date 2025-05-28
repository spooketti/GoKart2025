package frc.robot.subsystems.tank;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface TankIO {

    public static class TankData 
    {
        public double leftEncoderPositionRad = 0;
        public double rightEncoderPositionRad = 0;
        public double leftMotorSpeedRadPerSec = 0;
        public double rightMotorSpeedRadPerSec = 0;
        public double leftMotorVolts = 0;
        public double rightMotorVolts = 0;

        //refers to the chassis
        public double angularVelocityRadSec = 0;
        public double linearVelocityXMeterSec = 0;
        public double linearVelocityYMeterSec = 0;
    }

    public default void setLeftGoalSpeed(double speed)
    {

    }

    public default void setLeftVoltage(double volts)
    {
        
    }

    public default void setRightGoalSpeed(double speed)
    {

    }

    public default void setRightVoltage(double volts)
    {
        
    }

    public default double getLeftSpeed()
    {
        return 0;//TankData.leftMotorSpeedRadPerSec;
    }

    public default double getLeftEncoderPosition()
    {
        return 0;
    }

    public default double getRightEncoderPosition()
    {
        return 0;
    }

    public default double getRightSpeed()
    {
        return 0;//TankData.rightMotorSpeedRadPerSec;
    }

    public default double getAngularVelocityRadPerSec()
    {
        return 0;
    }

    public default Pose2d getPose()
    {
        return new Pose2d(0,0,new Rotation2d(0));
    }

    public default void updateData(TankData data)
    {
        
    }
    
}
