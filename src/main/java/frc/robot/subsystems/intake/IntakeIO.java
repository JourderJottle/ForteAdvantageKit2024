package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    
    @AutoLog
    public static class IntakeIOInputs {
        public double intakePosition = 0;
        public double intakeVoltage = 0;
        public double targetIntakePosition = 0;
        public double targetIntakeVoltage = 0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setIntakePosition(double position) {}

    public default void setIntakeVoltage(double voltage) {}

    public default double getIntakePosition() {return 0;}
}
