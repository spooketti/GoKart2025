package frc.robot.subsystems.tank;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Tank extends SubsystemBase
{
    private TankIO tankIO;

    public Tank()
    {
        if(Robot.isSimulation())
        {
            tankIO = new TankSim();
            return;
        }
        tankIO = new TankReal();
    }

    public void setLeftSpeed(double leftSpeed)
    {
        tankIO.setLeftSpeed(leftSpeed);
    }

    public void setRightSpeed(double rightSpeed)
    {
        tankIO.setRightSpeed(rightSpeed);
    }

    public double getLeftSpeed()
    {
        return tankIO.getLeftSpeed();
    }

    public double getRightSpeed()
    {
        return tankIO.getRightSpeed();
    }

    public void logData()
    {
        SmartDashboard.putNumber("LeftMotorSpeed",getLeftSpeed());
        SmartDashboard.putNumber("RightMotorSpeed", getRightSpeed());
    }

    public void updateData()
    {
       tankIO.updateData();
    }

    @Override
    public void periodic()
    {
        logData();
       updateData();
    }



}
