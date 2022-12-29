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
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;
    
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;

    private final PIDController steerPidController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int steerMotorId, boolean driveMotorReversed, boolean steerMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    //absolute encoders
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new CANCoder(absoluteEncoderId); //default is degrees per second
    absoluteEncoder.configFeedbackCoefficient(ModuleConstants.kAbsoluteEncoderCountsPerMin2Rad, "rad", SensorTimeBase.PerSecond); //convert to radians per second
    //motors
    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    steerMotor = new CANSparkMax(steerMotorId, MotorType.kBrushless);
    //inverting motors
    driveMotor.setInverted(driveMotorReversed);
    steerMotor.setInverted(steerMotorReversed);
    //motor encoders
    driveEncoder = driveMotor.getEncoder();
    steerEncoder = steerMotor.getEncoder();

    //so that we can work with meters and radians instead of rotations
    driveEncoder.getPosition();
    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRotToMeters);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRpm2Mps);
    steerEncoder.setVelocityConversionFactor(ModuleConstants.kSteerEncoderRot2Rad);
    steerEncoder.setVelocityConversionFactor(ModuleConstants.kSteerEncoderRPM2RadPerSec);
    
    steerPidController = new PIDController(ModuleConstants.kPSteer, ModuleConstants.kISteer, ModuleConstants.kDSteer);
    steerPidController.enableContinuousInput(-Math.PI, Math.PI);

    
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getSteerPosition() {
        return steerEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }
    
    public double getSteerVelocity() {
        return steerEncoder.getVelocity();
    }

    public double getAbsoluteEncoder() {
        return absoluteEncoder.getPosition();
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        steerEncoder.setPosition(getAbsoluteEncoder());
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
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.MAX_SPEED_METERS_PER_SECOND);
        steerMotor.set(steerPidController.calculate(getSteerPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID()+ "] state", state.toString());

    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }
}
