package frc.robot.subsystems.tank;

public interface TankIO {

    public static class TankData 
    {
        public double leftMotorSpeedRadPerSec = 0;
        public double rightMotorSpeedRadPerSec = 0;
    }

    public default void setLeftSpeed(double speed)
    {

    }

    public default void setRightSpeed(double speed)
    {

    }

    public default double getLeftSpeed()
    {
        return 0;//TankData.leftMotorSpeedRadPerSec;
    }

    public default double getRightSpeed()
    {
        return 0;//TankData.rightMotorSpeedRadPerSec;
    }

    public default void updateData(TankData data)
    {
        
    }
    
}
