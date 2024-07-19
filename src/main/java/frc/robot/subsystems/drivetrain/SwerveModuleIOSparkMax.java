package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {

    CANSparkMax magnitude, direction;
    SparkPIDController magnitudeController, directionController;
    RelativeEncoder magnitudeEncoder, directionEncoder;
    DutyCycleEncoder absoluteEncoder;
    double targetMagnitude = 0, targetDirection = 0;

    public SwerveModuleIOSparkMax(int magnitudeID, int directionID, int encoderID, double encoderOffset) {

        this.magnitude = new CANSparkMax(magnitudeID, MotorType.kBrushless);
        this.direction = new CANSparkMax(directionID, MotorType.kBrushless);

        this.magnitude.restoreFactoryDefaults();
        this.direction.restoreFactoryDefaults();
        this.magnitude.burnFlash();
        this.direction.burnFlash();

        this.magnitudeController = magnitude.getPIDController();
        this.directionController = direction.getPIDController();

        this.magnitudeController.setFF(Constants.DRIVE_MAGNITUDE_FF);
        this.magnitudeController.setP(Constants.DRIVE_MAGNITUDE_KP);
        this.magnitudeController.setI(Constants.DRIVE_MAGNITUDE_KI);
        this.magnitudeController.setD(Constants.DRIVE_MAGNITUDE_KD);
        this.directionController.setP(Constants.DRIVE_DIRECTION_KP);
        this.directionController.setI(Constants.DRIVE_DIRECTION_KI);
        this.directionController.setD(Constants.DRIVE_DIRECTION_KD);
        this.directionController.setPositionPIDWrappingEnabled(true);
        this.directionController.setPositionPIDWrappingMinInput(0);
        this.directionController.setPositionPIDWrappingMaxInput(2 * Math.PI);

        this.magnitudeEncoder = magnitude.getEncoder();
        this.directionEncoder = direction.getEncoder();
        this.absoluteEncoder = new DutyCycleEncoder(encoderID);

        this.directionEncoder.setPosition((this.absoluteEncoder.getAbsolutePosition() - encoderOffset)
        / RobotMap.DRIVE_DIRECTION_ABSOLUTE_ENCODER_UNITS_PER_ROTATION / RobotMap.DRIVE_DIRECTION_GEAR_RATIO);

        this.magnitudeEncoder.setVelocityConversionFactor(RobotMap.DRIVE_WHEEL_CIRCUMFERENCE / RobotMap.DRIVE_MAGNITUDE_GEAR_RATIO / 60);
        this.magnitudeEncoder.setPositionConversionFactor(RobotMap.DRIVE_WHEEL_CIRCUMFERENCE / RobotMap.DRIVE_MAGNITUDE_GEAR_RATIO);
        this.directionEncoder.setPositionConversionFactor(2 * Math.PI / RobotMap.DRIVE_DIRECTION_GEAR_RATIO);

    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.magnitude = this.getMagnitude();
        inputs.direction = this.getDirection();
        inputs.targetMagnitude = this.targetMagnitude;
        inputs.targetDirection = this.targetDirection;
    }

    @Override
    public void setStateTarget(SwerveModuleState state) {
        this.targetMagnitude = state.speedMetersPerSecond;
        this.targetDirection = state.angle.getRadians();
        this.magnitudeController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        this.directionController.setReference(state.angle.getRadians(), ControlType.kPosition);
    }

    @Override
    public double getMagnitude() {
        return this.magnitudeEncoder.getVelocity();
    }

    @Override
    public double getDistanceTraveled() {
        return this.magnitudeEncoder.getPosition();
    }

    @Override
    public double getDirection() {
        return this.directionEncoder.getPosition();
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(this.getMagnitude(), new Rotation2d(this.getDirection()));
    }
}
