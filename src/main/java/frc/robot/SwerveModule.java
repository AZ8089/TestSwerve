package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
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
    this.mAbsoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.mAbsoluteEncoderReversed = absoluteEncoderReversed;
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
        return mAbsoluteEncoder.getPosition();
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
