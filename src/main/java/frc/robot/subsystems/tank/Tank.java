package frc.robot.subsystems.tank;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.tank.TankIO.TankData;

public class Tank extends SubsystemBase
{
    private TankIO tankIO;
    private TankData tankData = new TankData();
    private Field2d field = new Field2d();

    public Tank()
    {
        if(Robot.isSimulation())
        {
            tankIO = new TankSim();
            return;
        }
        tankIO = new TankReal();
    }

    public void setLeftGoalSpeed(double leftSpeed)
    {
        tankIO.setLeftGoalSpeed(leftSpeed);
    }

    public void setRightGoalSpeed(double rightSpeed)
    {
        tankIO.setRightGoalSpeed(rightSpeed);
    }

    public double getLeftSpeed()
    {
        return tankIO.getLeftSpeed();
    }

    public double getRightSpeed()
    {
        return tankIO.getRightSpeed();
    }

    public double getAngularVelocityRadPerSec()
    {
        return tankIO.getAngularVelocityRadPerSec();
    }

    public void logData()
    {
        SmartDashboard.putNumber("LeftMotorSpeed",getLeftSpeed());
        SmartDashboard.putNumber("RightMotorSpeed", getRightSpeed());
        SmartDashboard.putNumber("LeftMotorVolts",tankData.leftMotorVolts);
        SmartDashboard.putNumber("RightMotorVolts",tankData.rightMotorVolts);
        SmartDashboard.putData("fuk",field);
    }

    public void updateData()
    {
        field.setRobotPose(tankIO.getPose());
       tankIO.updateData(tankData);
    }

    @Override
    public void periodic()
    {
        logData();
       updateData();
    }



}
