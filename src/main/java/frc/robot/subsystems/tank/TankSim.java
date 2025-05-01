package frc.robot.subsystems.tank;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class TankSim implements TankIO
{
    private FlywheelSim leftMotor = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getCIM(2), 0.000326, 1),DCMotor.getCIM(2));
    private FlywheelSim rightMotor = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getCIM(2), 0.000326, 1),DCMotor.getCIM(2));

    public void setLeftSpeed(double speedRadPS)
    {
        leftMotor.setAngularVelocity(speedRadPS);
    }

    public void setRightSpeed(double speedRadPS)
    {
        rightMotor.setAngularVelocity(speedRadPS);
    }

    public double getLeftSpeed()
    {
        return leftMotor.getAngularVelocityRadPerSec();
    }

    public double getRightSpeed()
    {
        return rightMotor.getAngularVelocityRadPerSec();
    }

    public void updateData(TankData data)
    {
        leftMotor.update(0.02);
        rightMotor.update(0.02);
        data.leftMotorSpeedRadPerSec = leftMotor.getAngularVelocityRadPerSec();
        data.rightMotorSpeedRadPerSec = rightMotor.getAngularVelocityRadPerSec();
    }
}
