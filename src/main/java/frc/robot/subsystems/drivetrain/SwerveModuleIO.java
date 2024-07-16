package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {

    @AutoLog
    public static class SwerveModuleIOInputs {
        public double magnitude = 0;
        public double direction = 0;
        public double targetMagnitude = 0;
        public double targetDirection = 0;
    }

    public default void updateInputs(SwerveModuleIOInputs inputs) {}

    public default void setStateTarget(SwerveModuleState state) {}

    public default double getMagnitude() {return 0;}

    public default double getDirection() {return 0;}
    
    public default SwerveModuleState getState() {return null;}
    
}
