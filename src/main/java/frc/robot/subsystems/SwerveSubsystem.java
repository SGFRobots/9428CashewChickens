package frc.robot.subsystems;

import static frc.robot.Constants.Mechanical.kModulePositions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


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
                    Constants.Mechanical.kFLDriveAbsoluteEncoderOffset,
                    Constants.Reversed.kFLDriveAbsoluteEncoderReversed),

            // Front Right
            new Module(
                    Constants.MotorPorts.kFRDriveMotorID,
                    Constants.MotorPorts.kFRTurningMotorID,
                    Constants.Reversed.kFRDriveReversed,
                    Constants.Reversed.kFRTurningReversed,
                    Constants.MotorPorts.kFRDriveAbsoluteEncoderID,
                    Constants.Mechanical.kFRDriveAbsoluteEncoderOffset,
                    Constants.Reversed.kFRDriveAbsoluteEncoderReversed),

            // Back Left
            new Module(
                    Constants.MotorPorts.kBLDriveMotorID,
                    Constants.MotorPorts.kBLTurningMotorID,
                    Constants.Reversed.kBLDriveReversed,
                    Constants.Reversed.kBLTurningReversed,
                    Constants.MotorPorts.kBLDriveAbsoluteEncoderID,
                    Constants.Mechanical.kBLDriveAbsoluteEncoderOffset,
                    Constants.Reversed.kBLDriveAbsoluteEncoderReversed),
            
            // Back Right
            new Module(
                    Constants.MotorPorts.kBRDriveMotorID,
                    Constants.MotorPorts.kBRTurningMotorID,
                    Constants.Reversed.kBRDriveReversed,
                    Constants.Reversed.kBRTurningReversed,
                    Constants.MotorPorts.kBRDriveAbsoluteEncoderID,
                    Constants.Mechanical.kBRDriveAbsoluteEncoderOffset,
                    Constants.Reversed.kBRDriveAbsoluteEncoderReversed),
            

    };

    // Positions stored in mOdometer
    private final SwerveDriveOdometry mOdometer;

    private ChassisSpeeds chassisSpeeds;

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

        mOdometer = new SwerveDriveOdometry(Constants.Mechanical.kDriveKinematics, new Rotation2d(),
                new SwerveModulePosition[] {
                        modules[0].getPosition(),
                        modules[1].getPosition(),
                        modules[2].getPosition(),
                        modules[3].getPosition()
                });

        // Simulated field
        SmartDashboard.putData("Field", mField2d);

        
    }

    // Get position of robot based on odometer
    public Pose2d getPose() {
        return mOdometer.getPoseMeters();
    }

    public void resetPose() {}

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
        // SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
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
    public void drive(ChassisSpeeds chassisSpeed) {
        this.chassisSpeeds = chassisSpeed;
        // Convert chassis speeds to each module states
        SwerveModuleState[] moduleStates = Constants.Mechanical.kDriveKinematics.toSwerveModuleStates(chassisSpeed);

        // Cap max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
                Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond);

        // Move each module
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(moduleStates[i]);
        }
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return chassisSpeeds;
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
        System.out.println(true);
        return true;
    }
}
