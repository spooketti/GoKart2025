package frc.robot.subsystems.tank;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class TankSim implements TankIO
{
    private FlywheelSim leftMotor = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getCIM(2), 0.326, 1),DCMotor.getCIM(2));
    private FlywheelSim rightMotor = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getCIM(2), 0.326, 1),DCMotor.getCIM(2));
    private PIDController leftPID = new PIDController(2, 0, 0);
    private PIDController rightPID = new PIDController(2, 0, 0);

    public void setLeftGoalSpeed(double speedRadPS)
    {
        leftPID.setSetpoint(speedRadPS);
        setLeftVoltage(leftPID.calculate(getLeftSpeed(),speedRadPS));
    }

    public void setRightGoalSpeed(double speedRadPS)
    {
        leftPID.setSetpoint(speedRadPS);
        setRightVoltage(rightPID.calculate(getRightSpeed(),speedRadPS));
    }

    public double getLeftSpeed()
    {
        return leftMotor.getAngularVelocityRadPerSec();
    }

    public double getRightSpeed()
    {
        return rightMotor.getAngularVelocityRadPerSec();
    }

    public void setLeftVoltage(double volts)
    {
        volts = MathUtil.clamp(volts, -12, 12);
        leftMotor.setInputVoltage(volts);
    }

    public void setRightVoltage(double volts)
    {
        volts = MathUtil.clamp(volts, -12, 12);
        rightMotor.setInputVoltage(volts);
    }

    public void updateData(TankData data)
    {
        leftMotor.update(0.02);
        rightMotor.update(0.02);
        data.leftMotorSpeedRadPerSec = leftMotor.getAngularVelocityRadPerSec();
        data.rightMotorSpeedRadPerSec = rightMotor.getAngularVelocityRadPerSec();
        data.leftMotorVolts = leftMotor.getInputVoltage();
        data.leftMotorVolts = rightMotor.getInputVoltage();
    }
}
