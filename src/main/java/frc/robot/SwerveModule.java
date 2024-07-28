package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class SwerveModule {

    double driveGearRatio = 5.14, turnGearRatio = 12.8, wheelCircumference = Units.inchesToMeters(4 * Math.PI);
    double driveConversionFactor = wheelCircumference / driveGearRatio;
    public static double maxSpeed = 5;

    TalonFX drive;
    CANSparkMax turn;
    SparkPIDController turnController;
    RelativeEncoder turnEncoder;
    AnalogEncoder turnAbsoluteEncoder;

    public SwerveModule(int driveID, int turnID, int encoderID, double offset) {

        drive = new TalonFX(driveID);
        turn = new CANSparkMax(turnID, MotorType.kBrushless);

        turn.restoreFactoryDefaults();

        turn.burnFlash();

        turnController = turn.getPIDController();

        Slot0Configs config = new Slot0Configs();
        config.kV = 0.2 / driveConversionFactor;
        config.kP = 0.1 / driveConversionFactor;
        config.kI = 0;
        config.kD = 0;

        turnController.setFF(0);
        turnController.setP(1);
        turnController.setI(0);
        turnController.setD(0);

        turnController.setPositionPIDWrappingEnabled(true);
        turnController.setPositionPIDWrappingMinInput(0);
        turnController.setPositionPIDWrappingMaxInput(2 * Math.PI);

        turnEncoder = turn.getEncoder();
        turnAbsoluteEncoder = new AnalogEncoder(encoderID);

        turnEncoder.setPosition(turnAbsoluteEncoder.getAbsolutePosition() - offset);

        turnEncoder.setPositionConversionFactor(2 * Math.PI / turnGearRatio);
    }

    public void setState(SwerveModuleState state) {
        drive.setControl(new VelocityVoltage(0).withSlot(0).withVelocity(state.speedMetersPerSecond / driveConversionFactor));
        turnController.setReference(state.angle.getRadians(), ControlType.kPosition);
    }

    public double getAngle() {
        return turnEncoder.getPosition();
    }

    public double getAbsolutePosition() {
        return turnAbsoluteEncoder.getAbsolutePosition();
    }
}
