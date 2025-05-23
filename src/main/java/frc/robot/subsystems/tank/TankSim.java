package frc.robot.subsystems.tank;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Robot;

public class TankSim implements TankIO
{
    private FlywheelSim leftMotor = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getCIM(2), 0.326, 1),DCMotor.getCIM(2));
    private FlywheelSim rightMotor = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getCIM(2), 0.326, 1),DCMotor.getCIM(2));
    private PIDController leftPID = new PIDController(2, 0, 0);
    private PIDController rightPID = new PIDController(2, 0, 0);
    private DifferentialDriveKinematics tankKinematics = new DifferentialDriveKinematics(Constants.TankConstants.distanceBetweenTracksMeter);
    private DifferentialDrivePoseEstimator tankPoseEstimator = new DifferentialDrivePoseEstimator(tankKinematics, new Rotation2d(0), 0, 0, new Pose2d(0,0,new Rotation2d(0)));
    private DifferentialDriveWheelSpeeds tankSpeeds = new DifferentialDriveWheelSpeeds(0,0);
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,0);
    private double leftDisplacementMeters = 0;
    private double rightDisplacementMeters = 0;

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
        return leftMotor.getAngularVelocityRadPerSec()*Constants.TankConstants.wheelRadiusMeters;
    }

    public double getRightSpeed()
    {
        return rightMotor.getAngularVelocityRadPerSec()*Constants.TankConstants.wheelRadiusMeters;
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

    public double getAngularVelocityRadPerSec()
    {
        return chassisSpeeds.omegaRadiansPerSecond;
    }

    public Pose2d getPose()
    {
        return tankPoseEstimator.getEstimatedPosition();
    }

    public void updateData(TankData data)
    {

        leftMotor.update(Constants.SimConstants.simPeriodicLoop);
        rightMotor.update(Constants.SimConstants.simPeriodicLoop);
        data.leftMotorSpeedRadPerSec = leftMotor.getAngularVelocityRadPerSec();
        data.rightMotorSpeedRadPerSec = rightMotor.getAngularVelocityRadPerSec();
        data.leftMotorVolts = leftMotor.getInputVoltage();
        data.rightMotorVolts = rightMotor.getInputVoltage();
        tankSpeeds.leftMetersPerSecond = getLeftSpeed();
        tankSpeeds.rightMetersPerSecond = getRightSpeed();
        chassisSpeeds = getChassisSpeeds();
        leftDisplacementMeters += getLeftSpeed()*Constants.SimConstants.simPeriodicLoop;
        rightDisplacementMeters += getRightSpeed()*Constants.SimConstants.simPeriodicLoop;
        // System.out.println(chassisSpeeds);
        data.angularVelocityRadSec = chassisSpeeds.omegaRadiansPerSecond;
        data.linearVelocityXMeterSec = chassisSpeeds.vxMetersPerSecond;
        data.linearVelocityYMeterSec = chassisSpeeds.vyMetersPerSecond;
        tankPoseEstimator.update(new Rotation2d(Units.degreesToRadians(Robot.gyro.getAngleDeg())), leftDisplacementMeters,rightDisplacementMeters);
    }

    public ChassisSpeeds getChassisSpeeds()
    {
        return tankKinematics.toChassisSpeeds(tankSpeeds);
    }
}
