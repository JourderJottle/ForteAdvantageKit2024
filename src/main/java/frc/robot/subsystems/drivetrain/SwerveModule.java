package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    SwerveModuleIO io;
    SwerveModuleIOInputsAutoLogged inputs;

    public SwerveModule(SwerveModuleIO io) {
        this.io = io;
        inputs = new SwerveModuleIOInputsAutoLogged();
    }

    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setStateTarget(SwerveModuleState state) {
        io.setStateTarget(SwerveModuleState.optimize(state, new Rotation2d(io.getDirection())));
    }
}
