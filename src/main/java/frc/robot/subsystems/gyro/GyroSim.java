package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Robot;

public class GyroSim implements GyroIO {

    @Override
    public void updateData(GyroData data) {
        Rotation2d deltaTheta = Rotation2d.fromRadians(Robot.tank.getAngularVelocityRadPerSec() * Constants.SimConstants.simPeriodicLoop);
        
        data.angleDeg = (data.angleDeg + deltaTheta.getDegrees() + 360) % 360;
    }
}
