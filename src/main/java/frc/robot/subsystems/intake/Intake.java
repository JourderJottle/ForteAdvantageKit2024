package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    IntakeIO io;
    IntakeIOInputsAutoLogged inputs;
    
    public Intake(IntakeIO io) {
        this.io = io;
        this.inputs = new IntakeIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
    }

    public void setIntakeDown() {
        this.io.setIntakePosition(Constants.INTAKE_DOWN_POSITION);
    }

    public void setIntakeUp() {
        this.io.setIntakePosition(Constants.INTAKE_UP_POSITION);
    }

    public void setIntakeSpeed(double speed) {
        this.io.setIntakeVoltage(speed);
    }
}
