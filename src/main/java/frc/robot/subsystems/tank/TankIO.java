package frc.robot.subsystems.tank;

public interface TankIO {

    public static class TankData 
    {
        public static double leftMotorSpeedRadPerSec = 0;
        public static double rightMotorSpeedRadPerSec = 0;
    }

    public default void setLeftSpeed(double speed)
    {

    }

    public default void setRightSpeed(double speed)
    {

    }

    public default double getLeftSpeed()
    {
        return TankData.leftMotorSpeedRadPerSec;
    }

    public default double getRightSpeed()
    {
        return TankData.rightMotorSpeedRadPerSec;
    }

    public default void updateData()
    {
        
    }
    
}
