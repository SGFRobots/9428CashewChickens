// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {        
    public static final class MotorPorts {
        public static final int kFrontLeftDriveMotorPort = 8;
        public static final int kBackLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 6;
        public static final int kBackRightDriveMotorPort = 4;
    
        public static final int kFrontLeftTurningMotorPort = 7;
        public static final int kBackLeftTurningMotorPort = 1;
        public static final int kFrontRightTurningMotorPort = 5;
        public static final int kBackRightTurningMotorPort = 3;
        
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;
    }

    public static final class Reversed {
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;
    
        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;
    
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
    }

    
    public static final class Mechanical {
        public static final double kWheelRadius = .0508;
        public static final double kWheelDiameterMeters = 0.1016;
        public static final double kDriveMotorGearRatio= 6.12;
        public static final double kTurningMotorGearRatio =150/7;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        
        public static final double kTrackWidth = 0.64;
        // Distance between right and left wheels
        public static final double kWheelBase = 0.64;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -4.811;
    
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
    
        public static final double kDeadzone = 0.05;
    
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond / kTrackWidth / 2;
    
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    
        public static final double kDriveEncoderResolution = 2048;
        public static final double kTurningEncoderResolution = 42;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    }

    public static final class Controllers {
        // XBox Controller
        public static final int LeftXPort = 0;
        public static final int LeftYPort = 1;
        public static final int LeftTriggerPort = 2;
        public static final int RightTriggerPort = 3;
        public static final int RightXPort = 4;
        public static final int RightYPort = 5;

        public static final int ButtonAPort = 1;
        public static final int ButtonBPort = 2;
        public static final int ButtonXPort = 3;
        public static final int ButtonYPort = 4;
        public static final int LeftBumper = 5;
        public static final int RightBumper = 6;

        public static final class XBox {
            public static final Button buttonA = XboxController.Button.kA;
            public static final Button buttonB = XboxController.Button.kB;
            public static final Button buttonX = XboxController.Button.kX;
            public static final Button buttonY = XboxController.Button.kY;
            public static final Button leftBumper = XboxController.Button.kLeftBumper;
            public static final Button rightBumper = XboxController.Button.kRightBumper;
        }
  }
}