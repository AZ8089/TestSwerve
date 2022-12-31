package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;

public class SwerveSubsystem extends SubsystemBase{
    //declare and instantiate all swerve modules
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftSteerMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftSteerEncoderReversed,
        DriveConstants.kFrontLeftAbsoluteEncoderPort,
        DriveConstants.kFrontLeftAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightSteerMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightSteerEncoderReversed,
        DriveConstants.kFrontRightAbsoluteEncoderPort,
        DriveConstants.kFrontRightAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftSteerMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftSteerEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightSteerMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightSteerEncoderReversed,
        DriveConstants.kBackRightAbsoluteEncoderPort,
        DriveConstants.kBackRightAbsoluteEncoderOffsetRad,
        DriveConstants.kBackRightAbsoluteEncoderReversed);

    //declare and instantiate Inertial Measurement Unit (Pigeon2)
    private Pigeon2 imu = new Pigeon2(DriveConstants.kPigeonPort); 

    //SwerveSubsystem constructor
    public SwerveSubsystem() {

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                resetImu();
            } catch(Exception e) {
            }        
        }).start();
    }


    public void resetImu() {
        Pigeon2Configuration config = new Pigeon2Configuration();
        config.MountPoseYaw = 0;
        config.MountPosePitch = 0;
        config.MountPoseRoll = 90;
        imu.configAllSettings(config);
    }

    public double getHeading() {
        return Math.IEEEremainder(imu.getYaw(), 360); //Pigeon2 is continuous (yaw value will go past 360 degrees). this converts it to between -180 and 180
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("robot Heading", getHeading());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMps);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

}
