package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    @AutoLog
    public static class GyroIOInputs {
        public double yaw = 0;
    }
    
    public default void updateInputs(GyroIOInputs inputs) {}

    public default double getYaw() {return 0;}

    public default void setYaw(double yaw) {}
}
