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
        public static final double kWheelBaseMeters = 0.52613; 
        public static final double kTrackWidthMeters = 0.52695;
        public static final double kMaxSpeedMps = 3.6576;
            
                
    }
}

