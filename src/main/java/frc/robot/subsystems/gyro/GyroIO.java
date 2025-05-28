package frc.robot.subsystems.gyro;

public interface GyroIO {

    public static class GyroData 
    {
        public double angleDeg = 0;
        public double angleRad = 0;
    }

    public default void updateData(GyroData data)
    {
        
    }
}
