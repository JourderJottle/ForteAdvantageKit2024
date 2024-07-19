package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    SwerveModuleIO io;
    SwerveModuleIOInputsAutoLogged inputs;

    public SwerveModule(SwerveModuleIO io) {
        this.io = io;
        this.inputs = new SwerveModuleIOInputsAutoLogged();
    }

    public void periodic() {
        this.io.updateInputs(this.inputs);
    }

    public void setStateTarget(SwerveModuleState state) {
        this.io.setStateTarget(SwerveModuleState.optimize(state, new Rotation2d(this.io.getDirection())));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.io.getDistanceTraveled(), new Rotation2d(this.io.getDirection()));
    }
}
