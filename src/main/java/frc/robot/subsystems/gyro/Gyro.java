package frc.robot.subsystems.gyro;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.gyro.GyroIO.GyroData;

public class Gyro extends SubsystemBase
{
    private GyroIO gyroIO;
    private GyroData gyroData = new GyroData(); 

    public Gyro()
    {
        if(Robot.isReal())
        {
            //get a load of this guy
            gyroIO = new GyroSim();
            return;
        }
        gyroIO = new GyroSim();
    }

    public double getAngleDeg()
    {
        return gyroData.angleDeg;
    }

    public void setGyroAngleRad(double rad)
    {
        gyroData.angleRad = rad;
    }

    @Override
    public void periodic()
    {
        gyroIO.updateData(gyroData);
    }
}
