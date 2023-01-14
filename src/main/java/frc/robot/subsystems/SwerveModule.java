package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
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

    //private final PIDController mSteerPidController;

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
       /*
        mDriveMotor.getPIDController().setP(0.002*20.0); // if its jerking then kP is probably too big
        mDriveMotor.getPIDController().setI(0.000005*20.0);
        mDriveMotor.getPIDController().setD(0.0);
        mDriveMotor.getPIDController().setFF(0.0);
*/
        mSteerMotor.getPIDController().setP(ModuleConstants.kPSteer);
        mSteerMotor.getPIDController().setI(ModuleConstants.kISteer);
        mSteerMotor.getPIDController().setD(ModuleConstants.kDSteer);
        mSteerMotor.getPIDController().setFF(ModuleConstants.kFFSteer);

        mDriveMotor.getPIDController().setP(ModuleConstants.kPDrive);
        mDriveMotor.getPIDController().setI(ModuleConstants.kIDrive);
        mDriveMotor.getPIDController().setD(ModuleConstants.kDDrive);
        mDriveMotor.getPIDController().setFF(ModuleConstants.kFFDrive);
        
        //so that we can work with meters and radians instead of rotations
        mDriveEncoder.setPositionConversionFactor(0.102*Math.PI/8.14);//gets distance traveled meters
    mDriveEncoder.setVelocityConversionFactor(0.102*Math.PI/(60.0*8.14)); //0.102*Math.PI/(8.14*60)
   
        mSteerEncoder.setPositionConversionFactor(ModuleConstants.kSteerEncoderRot2Rad);
        mSteerEncoder.setVelocityConversionFactor(ModuleConstants.kSteerEncoderRPM2RadPerSec);
        //mSteerEncoder.setVelocityConversionFactor(ModuleConstants.kSteerEncoderRPM2RadPerSec);
        

       

        /*
        mSteerPidController = new PIDController(ModuleConstants.kPSteer, ModuleConstants.kISteer, ModuleConstants.kDSteer);
        mSteerPidController.enableContinuousInput(-Math.PI, Math.PI);
            */
    
    }

    public double getDrivePosition() {
        return mDriveEncoder.getPosition();
    }

    public double getSteerPosition() {
        return mSteerEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return mDriveEncoder.getVelocity(); //should be in meters per second
    }
    
    public double getSteerVelocity() {
        return mSteerEncoder.getVelocity();
    }

    public double getAbsoluteEncoder() {
        double angle = mAbsoluteEncoder.getPosition()-mAbsoluteEncoderOffsetRad;   
        return angle * (mAbsoluteEncoderReversed ? -1.0 : 1.0);
        
    }

    public void resetEncoders() {
        mDriveEncoder.setPosition(0.0);
        mSteerEncoder.setPosition(getAbsoluteEncoder());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        //if statement allows us to ignore commands that don't have substantial driving velocity
        /*if(Math.abs(state.speedMetersPerSecond) <0.001) {
            stop();
            return;
        }
        */
        //by taking in the desired state and the current angle the wheels are at, change desired state so that the difference between current and desired angle is minimized
        //state = SwerveModuleState.optimize(state, getState().angle);
        //set motors to desired state
        mDriveMotor.getPIDController().setReference(3.0, ControlType.kVoltage);      
        //mSteerMotor.getPIDController().setReference(state.angle.getRadians(), ControlType.kPosition);
        //SmartDashboard.putString("Swerve[" + mAbsoluteEncoder.getDeviceID()+ "] state", state.toString());

    }

    public CANSparkMax getMotor(int id){
        if (id == 0){
            return mDriveMotor;
        }
        else if (id == 1){
            return mSteerMotor;
        }
        return null;        
    }

    public void stop() {
        mDriveMotor.set(0);
        mSteerMotor.set(0);
    }
}
