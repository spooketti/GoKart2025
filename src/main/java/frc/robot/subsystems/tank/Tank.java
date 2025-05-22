package frc.robot.subsystems.tank;

import static edu.wpi.first.units.Units.Rotation;

import java.io.File;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.tank.TankIO.TankData;

public class Tank extends SubsystemBase
{
    private DifferentialDriveKinematics tankKinematics = new DifferentialDriveKinematics(Constants.TankConstants.distanceBetweenTracksMeter);
    private DifferentialDrivePoseEstimator tankPoseEstimator = new DifferentialDrivePoseEstimator(tankKinematics, new Rotation2d(0), 0, 0, new Pose2d(0,0,new Rotation2d(0)));
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
        field.setRobotPose(tankPoseEstimator.getEstimatedPosition());
       tankIO.updateData(tankData);
    }

    @Override
    public void periodic()
    {
        logData();
       updateData();
    }



}
