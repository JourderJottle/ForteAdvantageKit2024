package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    
    @AutoLog
    public static class ClimbIOInputs {
        public double position = 0;
        public double current = 0;
        public double targetCurrent = 0;
    }

    public default void updateInputs(ClimbIOInputs inputs) {}

    public default void setCurrent(double current) {}

    public default double getPosition() {return 0;}

}
