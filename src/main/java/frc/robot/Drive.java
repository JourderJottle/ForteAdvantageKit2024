package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class Drive extends Command {

    Supplier<Double> forward, side, turn;
    Drivetrain drivetrain;

    public Drive(Supplier<Double> forward, Supplier<Double> side, Supplier<Double> turn) {
        this.forward = forward;
        this.side = side;
        this.turn = turn;
        drivetrain = Drivetrain.getInstance();
    }

    public void execute() {
        drivetrain.drive(new ChassisSpeeds(forward.get() * SwerveModule.maxSpeed, 
        side.get() * SwerveModule.maxSpeed,
        turn.get() * 2 * Math.PI));
    }
}
