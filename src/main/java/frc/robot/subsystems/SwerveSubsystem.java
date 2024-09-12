package frc.robot.subsystems;

import static frc.robot.Constants.Mechanical.kModulePositions;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;


public class SwerveSubsystem extends SubsystemBase {
    // Make instances of all 4 modules
    private final Module[] modules = {
            // Front Left
            new Module(
                    Constants.MotorPorts.kFLDriveMotorID,
                    Constants.MotorPorts.kFLTurningMotorID,
                    Constants.Reversed.kFLDriveReversed,
                    Constants.Reversed.kFLTurningReversed,
                    Constants.MotorPorts.kFLDriveAbsoluteEncoderID,
                    Constants.Mechanical.kFLDriveAbsoluteEncoderOffsetRad,
                    Constants.Reversed.kFLDriveAbsoluteEncoderReversed,
                    Constants.MotorPorts.kFLDriveEncoderPorts,
                    Constants.MotorPorts.kFLTurnEncoderPorts,
                    Constants.Reversed.kFLDriveEncoderReversed,
                    Constants.Reversed.kFLTurningEncoderReversed),

            // Front Right
            new Module(
                    Constants.MotorPorts.kFRDriveMotorID,
                    Constants.MotorPorts.kFRTurningMotorID,
                    Constants.Reversed.kFRDriveReversed,
                    Constants.Reversed.kFRTurningReversed,
                    Constants.MotorPorts.kFRDriveAbsoluteEncoderID,
                    Constants.Mechanical.kFRDriveAbsoluteEncoderOffsetRad,
                    Constants.Reversed.kFRDriveAbsoluteEncoderReversed,
                    Constants.MotorPorts.kFRDriveEncoderPorts,
                    Constants.MotorPorts.kFRTurnEncoderPorts,
                    Constants.Reversed.kFRDriveEncoderReversed,
                    Constants.Reversed.kFRTurningEncoderReversed),

            // Back Left
            new Module(
                    Constants.MotorPorts.kBLDriveMotorID,
                    Constants.MotorPorts.kBLTurningMotorID,
                    Constants.Reversed.kBLDriveReversed,
                    Constants.Reversed.kBLTurningReversed,
                    Constants.MotorPorts.kBLDriveAbsoluteEncoderID,
                    Constants.Mechanical.kBLDriveAbsoluteEncoderOffsetRad,
                    Constants.Reversed.kBLDriveAbsoluteEncoderReversed,
                    Constants.MotorPorts.kBLDriveEncoderPorts,
                    Constants.MotorPorts.kBLTurnEncoderPorts,
                    Constants.Reversed.kBLDriveEncoderReversed,
                    Constants.Reversed.kBLTurningEncoderReversed),
            // Back Right
            new Module(
                    Constants.MotorPorts.kBRDriveMotorID,
                    Constants.MotorPorts.kBRTurningMotorID,
                    Constants.Reversed.kBRDriveReversed,
                    Constants.Reversed.kBRTurningReversed,
                    Constants.MotorPorts.kBRDriveAbsoluteEncoderID,
                    Constants.Mechanical.kBRDriveAbsoluteEncoderOffsetRad,
                    Constants.Reversed.kBRDriveAbsoluteEncoderReversed,
                    Constants.MotorPorts.kBRDriveEncoderPorts,
                    Constants.MotorPorts.kBRTurnEncoderPorts,
                    Constants.Reversed.kBRDriveEncoderReversed,
                    Constants.Reversed.kBRTurningEncoderReversed),

    };
   
    // Positions stored in gyro and mOdometer
    // private final ADXRS450_GyroSim mGyroSim = new ADXRS450_GyroSim(mGyro);
    // private final AHRS mGyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry mOdometer;

    private double yaw;

    // Simulated field
    public static final Field2d mField2d = new Field2d();
    // Simulated modules
    Pose2d[] mModulePose = {
            new Pose2d(),
            new Pose2d(),
            new Pose2d(),
            new Pose2d()
    };

    // Constructor
    public SwerveSubsystem() {
        // Set up gyro and mOdometer
        // mGyro = new AHRS(SPI.Port.kMXP);
        // mGyro = new AnalogGyro(1);

        mOdometer = new SwerveDriveOdometry(Constants.Mechanical.kDriveKinematics, new Rotation2d(),
                new SwerveModulePosition[] {
                        modules[0].getPosition(),
                        modules[1].getPosition(),
                        modules[2].getPosition(),
                        modules[3].getPosition()
                });

        // Simulated field
        SmartDashboard.putData("Field", mField2d);
        // yaw = 0;

        // Reset gyro
        // mGyro.reset();
    }

    // Get angle robot is facing in degrees
    // public double getHeading() {
    // return mGyro.getAngle();
    // }

    // Get direction robot is facing
    // public Rotation2d getRotation2d() {
    // return Rotation2d.fromDegrees(getHeading());
    // }

    // Get position of robot based on odometer
    public Pose2d getPose() {
        return mOdometer.getPoseMeters();
    }

    // public Rotation2d getGyroRotation2d() {
    // return new Rotation2d(mGyro.getAngle() * (Math.PI / 180));
    // }

    @Override
    public void periodic() {
        // Update modules' positions
        mOdometer.update(new Rotation2d(), new SwerveModulePosition[] {
                modules[0].getPosition(),
                modules[1].getPosition(),
                modules[2].getPosition(),
                modules[3].getPosition() });

        // Update Pose for swerve modules - Position of the rotation and the translation matters
        for (int i = 0; i < modules.length; i++) {
            // No gyro - new Rotation2d() instead
            Translation2d updatedModulePosition = kModulePositions[i].rotateBy(new Rotation2d()).plus(getPose().getTranslation());
            // Module heading is the angle relative to the chasis heading
            mModulePose[i] = new Pose2d(updatedModulePosition, modules[i].getState().angle.plus(getPose().getRotation()));

            modules[i].periodic();
        }

        // Sets robot and modules positions on the field
        mField2d.setRobotPose(getPose());
        mField2d.getObject(Constants.ModuleNameSim).setPoses(mModulePose);

        // Logs in Swerve Tab
        double loggingState[] = {
                modules[0].getState().angle.getDegrees(), modules[0].getState().speedMetersPerSecond,
                modules[1].getState().angle.getDegrees(), modules[1].getState().speedMetersPerSecond,
                modules[2].getState().angle.getDegrees(), modules[2].getState().speedMetersPerSecond,
                modules[3].getState().angle.getDegrees(), modules[3].getState().speedMetersPerSecond
        };

        // Debug telemetry
        // SmartDashboard.putNumber("Robot Heading", mGyro.getAngle());
        // SmartDashboard.putString("Gyro", getGyroRotation2d().toString());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumberArray("SwerveModuleLOGGINGStates", loggingState);

    }

    // Reset odometer
    public void resetOdometry(Pose2d pose) {
        mOdometer.resetPosition(new Rotation2d(), new SwerveModulePosition[] {
                modules[0].getPosition(),
                modules[1].getPosition(),
                modules[2].getPosition(),
                modules[3].getPosition() }, pose);
    }

    // Stop the robot completely
    public void stopModules() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].stop();
        }
    }

    // DRIVE the robot
    public void drive(double xSpeed, double ySpeed, double turningSpeed, boolean fieldRelative) {
        // Set desire chassis speeds based on field or robot relative
        ChassisSpeeds chassisSpeed;
        chassisSpeed = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        SmartDashboard.putNumber("omegaradians", chassisSpeed.omegaRadiansPerSecond);
        SmartDashboard.putNumber("xSpeed", chassisSpeed.vxMetersPerSecond);
        SmartDashboard.putNumber("ySpeed", chassisSpeed.vyMetersPerSecond);

        // Convert chassis speeds to each module states
        SwerveModuleState[] moduleStates = Constants.Mechanical.kDriveKinematics.toSwerveModuleStates(chassisSpeed);
        // SmartDashboard.putNumber("omega radians/second", chassisSpeed.omegaRadiansPerSecond);

        // Cap max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
                Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond);

        // Move each module
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(moduleStates[i]);
        }
    }

    // Test one module at a time
    public void driveIndividualModule(double speed, double rotation) {
        modules[3].driveIndividually(speed, rotation);
    }

    // Reset modules rotations to 0
    public void resetEncoders() {
        for (Module module : modules) {
            module.resetting = true;
        }
    }
    
    // Reset modules rotations to 0
    public void stopReset() {
        for (Module module : modules) {
            module.resetting = false;
        }
    }

    // Check if all modules are done resetting angles
    public boolean checkEncoderResetted() {
        for (Module module : modules) {
            if (module.resetting) {
                return false;
            }
        }
        return true;
    }

    // Simulate robot's heading - No Gyro, no heading
    @Override
    public void simulationPeriodic() {
        for (Module module : modules) {
            module.simulationPeriodic(0.02);
        }

        SwerveModuleState[] moduleStates = {
                modules[0].getState(),
                modules[1].getState(),
                modules[2].getState(),
                modules[3].getState()
        };

        // Robot heading and gyro - No gyro
        // ChassisSpeeds chassisSpeed = Constants.Mechanical.kDriveKinematics.toChassisSpeeds(moduleStates);
        // yaw += chassisSpeed.omegaRadiansPerSecond * 0.02;
        // mGyroSim.setAngle(-Units.radiansToDegrees(yaw));
    }
}
