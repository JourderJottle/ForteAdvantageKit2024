package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    
    @AutoLog
    public static class ShooterIOInputs {
        public double position = 0;
        public double shooterSpeed = 0;
        public double feederCurrent = 0;
        public double targetPosition = 0;
        public double targetShooterSpeed = 0;
        public double targetFeederCurrent = 0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setPosition(double position) {}

    public default void setShooterSpeed(double speed) {}

    public default void setFeederCurrent(double current) {}

    public default double getPosition() {return 0;}

    public default double getShooterSpeed() {return 0;}

    public default double getFeederCurrent() {return 0;}
}
