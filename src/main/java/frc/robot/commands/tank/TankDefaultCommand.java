package frc.robot.commands.tank;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class TankDefaultCommand extends Command {

    private final DoubleSupplier leftSupplier;
    private final DoubleSupplier rightSupplier;

    public TankDefaultCommand(DoubleSupplier leftSupplier, DoubleSupplier rightSupplier)
    {
        this.leftSupplier = leftSupplier;
        this.rightSupplier = rightSupplier;
    }

    @Override
    public void execute()
    {
        Robot.tank.setLeftSpeed(leftSupplier.getAsDouble());
        Robot.tank.setRightSpeed(rightSupplier.getAsDouble());
    }
}
