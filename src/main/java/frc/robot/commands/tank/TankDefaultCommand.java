package frc.robot.commands.tank;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class TankDefaultCommand extends Command {

    private final DoubleSupplier leftSupplier;
    private final DoubleSupplier rightSupplier;

    public TankDefaultCommand(DoubleSupplier leftSupplier, DoubleSupplier rightSupplier)
    {
        this.leftSupplier = leftSupplier;
        this.rightSupplier = rightSupplier;
        addRequirements(Robot.tank);
    }

    private static double deadband(double deadBand, double value)
    {
        // System.out.println(value);
        if(deadBand>Math.abs(value))
        {
            return 0;
        }
        return value;
    }

    @Override
    public void execute()
    {
        // System.out.println(leftSupplier.getAsDouble());
        double clampedLeftSpeed = -deadband(0.1, leftSupplier.getAsDouble());
        double clampedRightSpeed = -deadband(0.1, rightSupplier.getAsDouble());
        Robot.tank.setLeftGoalSpeed(clampedLeftSpeed*Constants.TankConstants.maxAngularVelocityRadSec);
        Robot.tank.setRightGoalSpeed(clampedRightSpeed*Constants.TankConstants.maxAngularVelocityRadSec);
    }
}
