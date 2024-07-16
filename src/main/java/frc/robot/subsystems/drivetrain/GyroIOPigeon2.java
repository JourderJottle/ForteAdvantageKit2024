package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 implements GyroIO {

    Pigeon2 pigeon;

    public GyroIOPigeon2(int pigeonID) {
        this.pigeon = new Pigeon2(pigeonID);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw = this.getYaw();
    }

    @Override
    public double getYaw() {
        return Units.degreesToRadians(this.pigeon.getAngle());
    }

    @Override
    public void setYaw(double yaw) {
        this.pigeon.setYaw(Units.radiansToDegrees(yaw));
    }
}
