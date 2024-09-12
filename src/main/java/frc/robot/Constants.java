// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
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
    public static final String ModuleNameSim = "Swerve Modules";
    
    public static final class MotorPorts {
        // CAN IDs of driving motors
        public static final int kFLDriveMotorID = 2;
        public static final int kBLDriveMotorID = 3;
        public static final int kFRDriveMotorID = 1;
        public static final int kBRDriveMotorID = 4;

        // CAN IDs of turning motors
        public static final int kFLTurningMotorID = 10;
        public static final int kBLTurningMotorID = 11;
        public static final int kFRTurningMotorID = 9;
        public static final int kBRTurningMotorID = 12;
        
        // CAN IDs of CANCoders
        public static final int kFLDriveAbsoluteEncoderID = 6;
        public static final int kBLDriveAbsoluteEncoderID = 7;
        public static final int kFRDriveAbsoluteEncoderID = 5;
        public static final int kBRDriveAbsoluteEncoderID = 8;

        // Ports of driving encoders - NEED TO CHANGE!
        public static final int[] kFLDriveEncoderPorts = {9, 10};
        public static final int[] kBLDriveEncoderPorts = {11, 12};
        public static final int[] kFRDriveEncoderPorts = {13, 14};
        public static final int[] kBRDriveEncoderPorts = {15, 16};

        // Ports of turning encoders - NEED TO CHANGE!
        public static final int[] kFLTurnEncoderPorts = {17,18};
        public static final int[] kBLTurnEncoderPorts = {19,20};
        public static final int[] kFRTurnEncoderPorts = {21,22};
        public static final int[] kBRTurnEncoderPorts = {23,24};

        // Gyro
        public static final int kGyroPort = 2;
    }

    // Reversed motors
    public static final class Reversed {
        // Turning motors
        public static final boolean kFLTurningReversed = true;
        public static final boolean kBLTurningReversed = true;
        public static final boolean kFRTurningReversed = true;
        public static final boolean kBRTurningReversed = true;
    
        // Driving motors
        public static final boolean kFLDriveReversed = false;
        public static final boolean kBLDriveReversed = false;
        public static final boolean kFRDriveReversed = true;
        public static final boolean kBRDriveReversed = false;

        // Turning encoders
        public static final boolean kFLTurningEncoderReversed = true;
        public static final boolean kBLTurningEncoderReversed = true;
        public static final boolean kFRTurningEncoderReversed = true;
        public static final boolean kBRTurningEncoderReversed = true;
    
        // Driving encoders
        public static final boolean kFLDriveEncoderReversed = false;
        public static final boolean kBLDriveEncoderReversed = false;
        public static final boolean kFRDriveEncoderReversed = false;
        public static final boolean kBRDriveEncoderReversed = false;
    
        // CANCoders
        public static final boolean kFLDriveAbsoluteEncoderReversed = false;
        public static final boolean kBLDriveAbsoluteEncoderReversed = false;
        public static final boolean kFRDriveAbsoluteEncoderReversed = false;
        public static final boolean kBRDriveAbsoluteEncoderReversed = false;
    }

    // Physical and mechanical variables
    public static final class Mechanical {

        // Robot's physical measurements
        public static final double kWheelRadius = .0508;
        public static final double kWheelDiameterMeters = 0.1016;
        public static final double kDriveMotorGearRatio= 6.12;
        public static final double kTurningMotorGearRatio =150/7;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kDriveEncoderResolution = 2048;
        public static final double kTurningEncoderResolution = 42;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kEncoderCPR = 1024;
        public static final double kDistancePerPulse = (kWheelCircumferenceMeters) / kEncoderCPR;
        // The SysId tool provides a convenient method for obtaining these values for your robot.
        // the values being the volt related lines below 
        public static final double kVoltSecondPerRadian = 3.41;
        public static final double kVoltSecondsSquaredPerRadian = 0.111;
        public static final DCMotor kDriveGearBox = DCMotor.getKrakenX60(1);
        public static final DCMotor kTurnGearBox = DCMotor.getKrakenX60(1);
        
        // Distance between right and left wheels (in meters)
        public static final double kRobotWidth = 0.64;
        // Distance between front and back wheels (in meters)
        public static final double kRobotLength = 0.64;

        // CANCoders' offsets
        public static final double kFLDriveAbsoluteEncoderOffsetRad = -0.211;
        public static final double kBLDriveAbsoluteEncoderOffsetRad = -0.342;
        public static final double kFRDriveAbsoluteEncoderOffsetRad = -0.03;
        public static final double kBRDriveAbsoluteEncoderOffsetRad = -0.297;

        // Module Positions on Robot
        public static final Translation2d[] kModulePositions = {
            new Translation2d(kRobotLength / 2, kRobotWidth / 2),
            new Translation2d(kRobotLength / 2, -kRobotWidth / 2),
            new Translation2d(-kRobotLength / 2, kRobotWidth / 2),
            new Translation2d(-kRobotLength / 2, -kRobotWidth / 2)
        };
        // Kinematics map of robot
        // FL, FR, BL, BR
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            kModulePositions[0],
            kModulePositions[1],
            kModulePositions[2],
            kModulePositions[3]);
        
        // Deadzone
        public static final double kDeadzone = 0.05;
    
        // Speeds and Accelerations
        public static final double kPhysicalMaxSpeedMetersPerSecond = 0.50292;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2.846;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 10;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 10;
    }

    // XBox Controller
    public static final class Controllers {
        // Joysticks and triggers
        public static final int LeftXPort = 0;
        public static final int LeftYPort = 1;
        public static final int LeftTriggerPort = 2;
        public static final int RightTriggerPort = 3;
        public static final int RightXPort = 4;
        public static final int RightYPort = 5;

        // Buttons and bumpers
        public static final int ButtonAPort = 1;
        public static final int ButtonBPort = 2;
        public static final int ButtonXPort = 3;
        public static final int ButtonYPort = 4;
        public static final int LeftBumper = 5;
        public static final int RightBumper = 6;

        // Static buttons and bumpers
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