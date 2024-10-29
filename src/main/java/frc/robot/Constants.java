// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
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
        public static final boolean kFRDriveReversed = false;
        public static final boolean kBRDriveReversed = false;

        // Turning encoders
        public static final boolean kFLTurningEncoderReversed = true;
        public static final boolean kBLTurningEncoderReversed = true;
        public static final boolean kFRTurningEncoderReversed = true;
        public static final boolean kBRTurningEncoderReversed = true;
    
        // Driving encoders
        public static final boolean kFLDriveEncoderReversed = false;
        public static final boolean kBLDriveEncoderReversed = true;
        public static final boolean kFRDriveEncoderReversed = true;
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
        public static final double kWheelRadiusMeters = 0.0508;
        public static final double kWheelDiameterMeters = kWheelRadiusMeters * 2;
        public static final double kDriveMotorGearRatio = 6.12;
        public static final double kTurningMotorGearRatio = 150/7;
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
        public static final double kRobotWidthMeters = 0.64;
        // Distance between front and back wheels (in meters)
        public static final double kRobotLengthMeters = 0.64;

        // CANCoders' offsets
        public static final double kFLDriveAbsoluteEncoderOffset = 0.70996; 
        public static final double kBLDriveAbsoluteEncoderOffset = 0.83764;
        public static final double kFRDriveAbsoluteEncoderOffset = 0.52612; 
        public static final double kBRDriveAbsoluteEncoderOffset = 0.78564; 

        // Module Positions on Robot
        public static final Translation2d[] kModulePositions = {
            new Translation2d(kRobotLengthMeters / 2, kRobotWidthMeters / 2),
            new Translation2d(kRobotLengthMeters / 2, -kRobotWidthMeters / 2),
            new Translation2d(-kRobotLengthMeters / 2, kRobotWidthMeters / 2),
            new Translation2d(-kRobotLengthMeters / 2, -kRobotWidthMeters / 2)
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
        public static final double kPhysicalMaxSpeedMetersPerSecond = 0.3;
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
        public static final int RightXPort = 2;
        public static final int RightYPort = 3;

        // Dials
        public static final int s1DialPort = 4;
        public static final int s2DialPort = 5;

        // Switches
        public static final int SwitchE = 6;
        public static final int SwitchF = 7;
        public static final int UpperB = 1;
        public static final int MiddleB = 2;
        public static final int LowerB = 3;
        public static final int UpperC = 4;
        public static final int MiddleC = 5;
        public static final int LowerC = 6;

        // Buttons
        public static final int ButtonAPort = 7;
        public static final int ButtonDPort = 8;
  }
}