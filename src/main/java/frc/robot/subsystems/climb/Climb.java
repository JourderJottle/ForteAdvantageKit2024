package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    
    ClimbIO io;
    ClimbIOInputsAutoLogged inputs;

    public Climb(ClimbIO io) {
        this.io = io;
        this.inputs = new ClimbIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
    }

    public void setCurrent(double current) {
        this.io.setCurrent(current);
    }

}
