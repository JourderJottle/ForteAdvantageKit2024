package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

    SwerveDriveKinematics kinematics;
    SwerveModule frontLeft, frontRight, backLeft, backRight;
    SwerveModule[] modules;
    SwerveDriveOdometry odometry;
    GyroIO gyro;
    GyroIOInputsAutoLogged gyroInputs;

    public Drivetrain(GyroIO gyro) {
        if (Constants.DRIVETRAIN_IS_REAL) {
            this.frontLeft = new SwerveModule(new SwerveModuleIOSparkMax(RobotMap.FRONT_LEFT_MAGNITUDE_ID, RobotMap.FRONT_LEFT_DIRECTION_ID, RobotMap.FRONT_LEFT_ABSOLUTE_ENCODER_ID, Constants.FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET));
            this.frontRight = new SwerveModule(new SwerveModuleIOSparkMax(RobotMap.FRONT_RIGHT_MAGNITUDE_ID, RobotMap.FRONT_RIGHT_DIRECTION_ID, RobotMap.FRONT_RIGHT_ABSOLUTE_ENCODER_ID, Constants.FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET));
            this.backLeft = new SwerveModule(new SwerveModuleIOSparkMax(RobotMap.BACK_LEFT_MAGNITUDE_ID, RobotMap.BACK_LEFT_DIRECTION_ID, RobotMap.BACK_LEFT_ABSOLUTE_ENCODER_ID, Constants.BACK_LEFT_ABSOLUTE_ENCODER_OFFSET));
            this.backRight = new SwerveModule(new SwerveModuleIOSparkMax(RobotMap.BACK_RIGHT_MAGNITUDE_ID, RobotMap.BACK_RIGHT_DIRECTION_ID, RobotMap.BACK_RIGHT_ABSOLUTE_ENCODER_ID, Constants.BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET));
        }
        this.modules = new SwerveModule[] {this.frontLeft, this.frontRight, this.backLeft, this.backRight};

        this.gyro = gyro;
        this.gyroInputs = new GyroIOInputsAutoLogged();

        kinematics = new SwerveDriveKinematics(new Translation2d[] {
            new Translation2d(RobotMap.CHASSIS_WIDTH / 2, -RobotMap.CHASSIS_WIDTH / 2),
            new Translation2d(RobotMap.CHASSIS_WIDTH / 2, RobotMap.CHASSIS_WIDTH / 2),
            new Translation2d(-RobotMap.CHASSIS_WIDTH / 2, -RobotMap.CHASSIS_WIDTH / 2),
            new Translation2d(-RobotMap.CHASSIS_WIDTH / 2, RobotMap.CHASSIS_WIDTH / 2)
        });

        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(this.gyro.getYaw()), this.getSwerveModulePositions());
    }

    @Override
    public void periodic() {
        for (SwerveModule sm : this.modules) sm.periodic();
        this.gyro.updateInputs(this.gyroInputs);
        this.odometry.update(new Rotation2d(this.gyro.getYaw()), this.getSwerveModulePositions());
    }

    public void driveSwerve(double xSpeed, double ySpeed, double rotation) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, new Rotation2d(gyro.getYaw()));
        SwerveModuleState[] states = this.kinematics.toSwerveModuleStates(speeds);
        for (int i = 0; i < 4; i++) this.modules[i].setStateTarget(states[i]);
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) modulePositions[i] = this.modules[i].getPosition();
        return modulePositions;
    }
}