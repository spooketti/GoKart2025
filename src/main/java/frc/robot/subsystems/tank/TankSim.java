package frc.robot.subsystems.tank;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class TankSim implements TankIO
{
    private FlywheelSim leftMotor = new FlywheelSim(null, DCMotor.getCIM(2), null);
    private FlywheelSim rightMotor = new FlywheelSim(null, DCMotor.getCIM(2), null);

    public void setLeftSpeed(double speedRadPS)
    {
        leftMotor.setAngularVelocity(speedRadPS);
    }

    public void setRightSpeed(double speedRadPS)
    {
        rightMotor.setAngularVelocity(speedRadPS);
    }
}
