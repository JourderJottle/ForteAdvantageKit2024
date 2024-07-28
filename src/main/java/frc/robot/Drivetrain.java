package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    double chassisWidth = Units.inchesToMeters(19.5);;

    SwerveModule frontLeft, frontRight, backLeft, backRight;
    Pigeon2 gyro;
    SwerveDriveKinematics kinematics;
    static Drivetrain instance = null;

    public Drivetrain() {
        frontLeft = new SwerveModule(1, 2, 2, 0);
        frontRight = new SwerveModule(3, 4, 3, 0);
        backLeft = new SwerveModule(5, 6, 1, 0);
        backRight = new SwerveModule(7, 8, 0, 0);
        gyro = new Pigeon2(10);
        kinematics = new SwerveDriveKinematics(new Translation2d[] {
            new Translation2d(chassisWidth / 2, -chassisWidth / 2),
            new Translation2d(chassisWidth / 2, chassisWidth / 2),
            new Translation2d(-chassisWidth / 2, -chassisWidth / 2),
            new Translation2d(-chassisWidth / 2, chassisWidth / 2)
        });

    }

    @Override
    public void periodic() {
        
    }

    public void drive(ChassisSpeeds speeds) {
        ChassisSpeeds trueSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyro.getRotation2d().div(360 / 2.0 / Math.PI));
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(trueSpeeds);
        frontLeft.setState(SwerveModuleState.optimize(states[0], new Rotation2d(frontLeft.getAngle())));
        frontRight.setState(SwerveModuleState.optimize(states[1], new Rotation2d(frontRight.getAngle())));
        backLeft.setState(SwerveModuleState.optimize(states[2], new Rotation2d(backLeft.getAngle())));
        backRight.setState(SwerveModuleState.optimize(states[3], new Rotation2d(backRight.getAngle())));
        SmartDashboard.putNumber("frontLeft", frontLeft.getAbsolutePosition());
        SmartDashboard.putNumber("frontRight", frontRight.getAbsolutePosition());
        SmartDashboard.putNumber("backLeft", backLeft.getAbsolutePosition());
        SmartDashboard.putNumber("backRight", backRight.getAbsolutePosition());
    }

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }
}

