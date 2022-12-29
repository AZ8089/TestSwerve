// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //TODO: decide whether to move ModuleConstants to SwerveModule class
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = .1016; //in meters //4 inches
        public static final double kDriveMotorGearRatio = 1/5.84628; //TODO: measure drive motor gear ratio
        public static final double kSteerMotorGearRatio = 1/18.0; //TODO: measure steer motor gear ratio
        public static final double kDriveEncoderRotToMeters = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kSteerEncoderRot2Rad = kSteerMotorGearRatio *kDriveEncoderRotToMeters;
        public static final double kDriveEncoderRpm2Mps = kDriveEncoderRotToMeters / 60; //rpm = rotations per minute //mps = meters per second
        public static final double kSteerEncoderRPM2RadPerSec = kSteerEncoderRot2Rad / 60;
        public static final double kCANCoderCounts = 4096.0; // CANCoders have a resolution of 4096 counts per revolution
        public static final double kAbsoluteEncoderCountsPerMin2Rad = 2*Math.PI/kCANCoderCounts; 

        //PID values
        public static final double kPSteer = 0.5; //TODO: Determine actual kPSteer Value
        public static final double kISteer = 0.0; //TODO: Determine actual kISteer Value
        public static final double kDSteer = 0.0; //TODO: Determine actual kDSteer Value
    }

    public static final class DriveConstants {


        // Spark Max IDs
        public static final int kFrontLeftDriveMotorPort = 1; //TODO: Decide what this ID needs to be
        public static final int kFrontRightDriveMotorPort = 2; //TODO: Decide what this  ID needs to be
        public static final int kBackLeftDriveMotorPort = 3; //TODO: Decide what this  ID needs to be
        public static final int kBackRightDriveMotorPort = 4; //TODO: Decide what this  ID needs to be
    
        public static final int kFrontLeftSteerMotorPort = 5; //TODO: Decide what this  ID needs to be
        public static final int kFrontRightSteerMotorPort = 6; //TODO: Decide what this ID needs to be
        public static final int kBackLeftSteerMotorPort = 7; //TODO: Decide what this  ID needs to be
        public static final int kBackRightSteerMotorPort = 8; //TODO: Decide what this ID needs to be
    
        // Measurements //TODO: confirm measurements
        public static final double kWheelBaseMeters = .52613; 
        public static final double kTrackWidthMeters = .52695;
        public static final double kMaxSpeedMps = 3.6576;
            
                
    }
}

