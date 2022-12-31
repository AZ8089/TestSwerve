package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = 0.1016; //in meters //4 inches
        public static final double kDriveMotorGearRatio = 1/5.84628; //TODO: measure drive motor gear ratio
        public static final double kSteerMotorGearRatio = 1/18.0; //TODO: measure steer motor gear ratio
        public static final double kDriveEncoderRotToMeters = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kSteerEncoderRot2Rad = kSteerMotorGearRatio *kDriveEncoderRotToMeters;
        public static final double kDriveEncoderRpm2Mps = kDriveEncoderRotToMeters / 60.0; //rpm = rotations per minute //mps = meters per second
        public static final double kSteerEncoderRPM2RadPerSec = kSteerEncoderRot2Rad / 60.0;
        public static final double kCANCoderCounts = 4096.0; // CANCoders have a resolution of 4096 counts per revolution
        public static final double kAbsoluteEncoderCountsPerMin2Rad = 2.0*Math.PI/kCANCoderCounts; 

        //PID values
        public static final double kPSteer = 0.5; //TODO: Determine actual kPSteer Value
        public static final double kISteer = 0.0; //TODO: Determine actual kISteer Value
        public static final double kDSteer = 0.0; //TODO: Determine actual kDSteer Value
    }
    private final CANSparkMax mDriveMotor;
    private final CANSparkMax mSteerMotor;
    
    private final RelativeEncoder mDriveEncoder;
    private final RelativeEncoder mSteerEncoder;

    private final PIDController mSteerPidController;

    private final CANCoder mAbsoluteEncoder;
    private final boolean mAbsoluteEncoderReversed;
    private final double mAbsoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int steerMotorId, boolean driveMotorReversed, boolean steerMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    //absolute encoders
    mAbsoluteEncoderOffsetRad = absoluteEncoderOffset;
    mAbsoluteEncoderReversed = absoluteEncoderReversed;
    mAbsoluteEncoder = new CANCoder(absoluteEncoderId); //default is degrees per second
    mAbsoluteEncoder.configFeedbackCoefficient(ModuleConstants.kAbsoluteEncoderCountsPerMin2Rad, "rad", SensorTimeBase.PerSecond); //convert to radians per second
    //motors
    mDriveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    mSteerMotor = new CANSparkMax(steerMotorId, MotorType.kBrushless);
    //inverting motors
    mDriveMotor.setInverted(driveMotorReversed);
    mSteerMotor.setInverted(steerMotorReversed);
    //motor encoders
    mDriveEncoder = mDriveMotor.getEncoder();
    mSteerEncoder = mSteerMotor.getEncoder();

    //so that we can work with meters and radians instead of rotations
    mDriveEncoder.getPosition();
    mDriveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRotToMeters);
    mDriveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRpm2Mps);
    mSteerEncoder.setVelocityConversionFactor(ModuleConstants.kSteerEncoderRot2Rad);
    mSteerEncoder.setVelocityConversionFactor(ModuleConstants.kSteerEncoderRPM2RadPerSec);
    
    //TODO: hardware PID?
    
    mSteerPidController = new PIDController(ModuleConstants.kPSteer, ModuleConstants.kISteer, ModuleConstants.kDSteer);
    mSteerPidController.enableContinuousInput(-Math.PI, Math.PI);

    
    }

    public double getDrivePosition() {
        return mDriveEncoder.getPosition();
    }

    public double getSteerPosition() {
        return mSteerEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return mDriveEncoder.getVelocity();
    }
    
    public double getSteerVelocity() {
        return mSteerEncoder.getVelocity();
    }

    public double getAbsoluteEncoder() {
        return mAbsoluteEncoder.getPosition()-mAbsoluteEncoderOffsetRad;
    }

    public void resetEncoders() {
        mDriveEncoder.setPosition(0);
        mSteerEncoder.setPosition(getAbsoluteEncoder());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        //if statement allows us to ignore commands that don't have substantial driving celocity
        if(Math.abs(state.speedMetersPerSecond) <0.001) {
            stop();
            return;
        }
        //by taking in the desired state and the current angle the wheels are at, change desired state so that the difference between current and desired angle is minimized
        state = SwerveModuleState.optimize(state, getState().angle);
        //set motors to desired state
        mDriveMotor.set(state.speedMetersPerSecond / DriveConstants.kMaxSpeedMps);
        mSteerMotor.set(mSteerPidController.calculate(getSteerPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + mAbsoluteEncoder.getDeviceID()+ "] state", state.toString());

    }

    public void stop() {
        mDriveMotor.set(0);
        mSteerMotor.set(0);
    }
}
