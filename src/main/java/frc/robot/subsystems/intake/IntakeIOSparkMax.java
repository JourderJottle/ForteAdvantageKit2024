package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.RobotMap;

public class IntakeIOSparkMax implements IntakeIO {
    
    CANSparkMax positionMotor, intakeMotor;
    SparkPIDController positionMotorController;
    RelativeEncoder positionMotorEncoder;
    double positionTarget = 0, voltageTarget = 0;

    public IntakeIOSparkMax(int positionMotorID, int intakeMotorID) {
        this.positionMotor = new CANSparkMax(positionMotorID, MotorType.kBrushless);
        this.intakeMotor = new CANSparkMax(intakeMotorID, MotorType.kBrushless);

        this.positionMotor.restoreFactoryDefaults();
        this.intakeMotor.restoreFactoryDefaults();

        this.positionMotor.burnFlash();
        this.intakeMotor.burnFlash();

        this.positionMotorController = this.positionMotor.getPIDController();

        this.positionMotorController.setP(Constants.INTAKE_POSITION_KP);
        this.positionMotorController.setI(Constants.INTAKE_POSITION_KI);
        this.positionMotorController.setD(Constants.INTAKE_POSITION_KD);

        this.positionMotorEncoder = this.positionMotor.getEncoder();

        this.positionMotorEncoder.setPositionConversionFactor(RobotMap.INTAKE_POSITION_GEAR_RATION);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakePosition = positionMotorEncoder.getPosition();
        inputs.intakeVoltage = intakeMotor.getAppliedOutput();
        inputs.targetIntakePosition = this.positionTarget;
        inputs.targetIntakeVoltage = this.voltageTarget;
    }

    @Override
    public void setIntakePosition(double position) {
        this.positionTarget = position;
        this.positionMotorController.setReference(position, ControlType.kPosition);
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        this.voltageTarget = voltage;
        this.intakeMotor.set(voltage);
    }

    @Override
    public double getIntakePosition() {
        return this.positionMotorEncoder.getPosition();
    }
}
