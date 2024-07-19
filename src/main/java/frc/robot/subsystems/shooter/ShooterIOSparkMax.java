package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.RobotMap;

public class ShooterIOSparkMax implements ShooterIO {
    
    CANSparkMax position, feeder, shooterSpeedLead, shooterSpeedFollower;
    SparkPIDController positionController, shooterSpeedController;
    RelativeEncoder positionEncoder, shooterEncoder;
    double positionTarget = 0, shooterSpeedTarget = 0, feederCurrentTarget = 0;

    public ShooterIOSparkMax(int positionID, int feederID, int shooterSpeedLeadID, int shooterSpeedFollowerID) {
        this.position = new CANSparkMax(positionID, MotorType.kBrushless);
        this.feeder = new CANSparkMax(feederID, MotorType.kBrushless);
        this.shooterSpeedLead = new CANSparkMax(shooterSpeedLeadID, MotorType.kBrushless);
        this.shooterSpeedFollower = new CANSparkMax(shooterSpeedFollowerID, MotorType.kBrushless);

        this.position.restoreFactoryDefaults();
        this.feeder.restoreFactoryDefaults();
        this.shooterSpeedLead.restoreFactoryDefaults();
        this.shooterSpeedFollower.restoreFactoryDefaults();

        this.shooterSpeedFollower.follow(shooterSpeedLead);

        this.position.burnFlash();
        this.feeder.burnFlash();
        this.shooterSpeedLead.burnFlash();
        this.shooterSpeedFollower.burnFlash();

        this.positionController = this.position.getPIDController();
        this.shooterSpeedController = this.shooterSpeedLead.getPIDController();

        this.positionController.setP(Constants.SHOOTER_POSITION_KP);
        this.positionController.setI(Constants.SHOOTER_POSITION_KI);
        this.positionController.setD(Constants.SHOOTER_POSITION_KD);
        this.shooterSpeedController.setFF(Constants.SHOOTER_SPEED_FF);
        this.shooterSpeedController.setP(Constants.SHOOTER_SPEED_KP);
        this.shooterSpeedController.setI(Constants.SHOOTER_SPEED_KI);
        this.shooterSpeedController.setD(Constants.SHOOTER_SPEED_KD);

        this.positionEncoder = this.position.getEncoder();
        this.shooterEncoder = this.position.getEncoder();

        this.positionEncoder.setPositionConversionFactor(2 * Math.PI / RobotMap.SHOOTER_POSITION_GEAR_RATIO);
        this.shooterEncoder.setVelocityConversionFactor(RobotMap.FLYWHEEL_CIRCUMFERENCE / RobotMap.SHOOTER_SPEED_GEAR_RATIO / 60);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.position = this.getPosition();
        inputs.shooterSpeed = this.getShooterSpeed();
        inputs.feederCurrent = this.getFeederCurrent();
        inputs.targetPosition = this.positionTarget;
        inputs.targetShooterSpeed = this.shooterSpeedTarget;
        inputs.targetFeederCurrent = this.feederCurrentTarget;
    }

    @Override
    public void setPosition(double position) {
        this.positionTarget = position;
        this.positionController.setReference(position, ControlType.kPosition);
    }

    @Override
    public void setShooterSpeed(double speed) {
        this.shooterSpeedTarget = speed;
        this.shooterSpeedController.setReference(speed, ControlType.kVelocity);
    }

    @Override
    public void setFeederCurrent(double current) {
        this.feederCurrentTarget = current;
        this.feeder.set(current);
    }

    @Override
    public double getPosition() {
        return this.positionEncoder.getPosition();
    }

    @Override
    public double getShooterSpeed() {
        return this.shooterEncoder.getVelocity();
    }

    @Override
    public double getFeederCurrent() {
        return this.feeder.getAppliedOutput();
    }
}
