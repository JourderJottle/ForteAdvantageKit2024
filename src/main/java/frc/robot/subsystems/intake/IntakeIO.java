package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    
    @AutoLog
    public static class IntakeIOInputs {
        public double position = 0;
        public double current = 0;
        public double targetPosition = 0;
        public double targetCurrent = 0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setPosition(double position) {}

    public default void setCurrent(double current) {}

    public default double getPosition() {return 0;}
}
