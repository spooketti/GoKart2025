package frc.robot.subsystems.tank;

public interface TankIO {

    public static class TankData 
    {
        public double leftMotorSpeedRadPerSec = 0;
        public double rightMotorSpeedRadPerSec = 0;
        public double leftMotorVolts = 0;
        public double rightMotorVolts = 0;
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

    public default double getRightSpeed()
    {
        return 0;//TankData.rightMotorSpeedRadPerSec;
    }

    public default void updateData(TankData data)
    {
        
    }
    
}
