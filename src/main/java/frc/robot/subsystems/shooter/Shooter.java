package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
    
    ShooterIO io;
    ShooterIOInputsAutoLogged inputs;

    public Shooter(ShooterIO io) {
        this.io = io;
        this.inputs = new ShooterIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
    }

    public boolean targetObject(double floorDistance, double height) {
        double adjHeight = height - RobotMap.SHOOTER_HEIGHT;
        double hypotenuse = Math.sqrt(floorDistance * floorDistance + adjHeight * adjHeight);
        this.io.setPosition(Math.acos(floorDistance / hypotenuse));
        return Math.abs(this.inputs.targetPosition - this.inputs.position) < Math.PI / 12;
    }

    public void setShooterSpeed(double speed) {
        this.io.setShooterSpeed(speed);
    }

    public void setFeederCurrent(double current) {
        this.io.setFeederCurrent(current);
    }
}
