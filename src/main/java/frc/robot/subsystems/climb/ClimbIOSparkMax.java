package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimbIOSparkMax implements ClimbIO {

    CANSparkMax driver;
    RelativeEncoder encoder;
    double targetCurrent = 0;

    public ClimbIOSparkMax(int id) {

        this.driver = new CANSparkMax(id, MotorType.kBrushless);

        this.driver.restoreFactoryDefaults();

        this.driver.setIdleMode(IdleMode.kBrake);

        this.driver.burnFlash();

        this.encoder = this.driver.getEncoder();
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.position = this.encoder.getPosition();
        inputs.current = this.driver.getAppliedOutput();
        inputs.targetCurrent = this.targetCurrent;
    }

    @Override
    public void setCurrent(double current) {
        this.driver.set(current);
    }

    @Override
    public double getPosition() {
        return this.encoder.getPosition();
    }

}
