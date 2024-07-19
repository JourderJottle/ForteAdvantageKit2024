package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.RobotMap;

public class IntakeIOSparkMax implements IntakeIO {
    
    CANSparkMax position, intake;
    SparkPIDController positionController;
    RelativeEncoder positionEncoder;
    double positionTarget = 0, currentTarget = 0;

    public IntakeIOSparkMax(int positionID, int intakeID) {
        this.position = new CANSparkMax(positionID, MotorType.kBrushless);
        this.intake = new CANSparkMax(intakeID, MotorType.kBrushless);

        this.position.restoreFactoryDefaults();
        this.intake.restoreFactoryDefaults();

        this.position.burnFlash();
        this.intake.burnFlash();

        this.positionController = this.position.getPIDController();

        this.positionController.setP(Constants.INTAKE_POSITION_KP);
        this.positionController.setI(Constants.INTAKE_POSITION_KI);
        this.positionController.setD(Constants.INTAKE_POSITION_KD);

        this.positionEncoder = this.position.getEncoder();

        this.positionEncoder.setPositionConversionFactor(2 * Math.PI / RobotMap.INTAKE_POSITION_GEAR_RATIO);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.position = positionEncoder.getPosition();
        inputs.current = intake.getAppliedOutput();
        inputs.targetPosition = this.positionTarget;
        inputs.targetCurrent = this.currentTarget;
    }

    @Override
    public void setPosition(double position) {
        this.positionTarget = position;
        this.positionController.setReference(position, ControlType.kPosition);
    }

    @Override
    public void setCurrent(double current) {
        this.currentTarget = current;
        this.intake.set(current);
    }

    @Override
    public double getPosition() {
        return this.positionEncoder.getPosition();
    }
}
