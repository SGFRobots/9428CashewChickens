package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SwerveSubsystem extends SubsystemBase{
    // Make instances of all 4 modules
    private final SwerveModule[] modules = {
        // Front Left
        new SwerveModule(
            Constants.MotorPorts.kFrontLeftDriveMotorPort,
            Constants.MotorPorts.kFrontLeftTurningMotorPort,
            Constants.Reversed.kFrontLeftDriveEncoderReversed,
            Constants.Reversed.kFrontLeftTurningEncoderReversed,
            Constants.MotorPorts.kFrontLeftDriveAbsoluteEncoderPort,
            Constants.Mechanical.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            Constants.Reversed.kFrontLeftDriveAbsoluteEncoderReversed),
        
        // Front Right
        new SwerveModule(
            Constants.MotorPorts.kFrontRightDriveMotorPort,
            Constants.MotorPorts.kFrontRightTurningMotorPort,
            Constants.Reversed.kFrontRightDriveEncoderReversed,
            Constants.Reversed.kFrontRightTurningEncoderReversed,
            Constants.MotorPorts.kFrontRightDriveAbsoluteEncoderPort,
            Constants.Mechanical.kFrontRightDriveAbsoluteEncoderOffsetRad,
            Constants.Reversed.kFrontRightDriveAbsoluteEncoderReversed),
        
        // Back Left
        new SwerveModule(
            Constants.MotorPorts.kBackLeftDriveMotorPort,
            Constants.MotorPorts.kBackLeftTurningMotorPort,
            Constants.Reversed.kBackLeftDriveEncoderReversed,
            Constants.Reversed.kBackLeftTurningEncoderReversed,
            Constants.MotorPorts.kBackLeftDriveAbsoluteEncoderPort,
            Constants.Mechanical.kBackLeftDriveAbsoluteEncoderOffsetRad,
            Constants.Reversed.kBackLeftDriveAbsoluteEncoderReversed),
        
        // Back Right
        new SwerveModule(
            Constants.MotorPorts.kBackRightDriveMotorPort,
            Constants.MotorPorts.kBackRightTurningMotorPort,
            Constants.Reversed.kBackRightDriveEncoderReversed,
            Constants.Reversed.kBackRightTurningEncoderReversed,
            Constants.MotorPorts.kBackRightDriveAbsoluteEncoderPort,
            Constants.Mechanical.kBackRightDriveAbsoluteEncoderOffsetRad,
            Constants.Reversed.kBackRightDriveAbsoluteEncoderReversed),
        
    };

    // Positions stored in gyro and mOdometer
    private final AHRS mGyro;
    private final SwerveDriveOdometry mOdometer;

    // =)
    public SwerveSubsystem() {
        // Set up gyro and mOdometer
        mGyro = new AHRS(SPI.Port.kMXP);
        mOdometer = new SwerveDriveOdometry(Constants.Mechanical.kDriveKinematics,
        new Rotation2d(0), 
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        }, new Pose2d(5.0, 13.5, new Rotation2d()));

        // Reset gyro
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                // Reset position
                mGyro.reset();
            } catch (Exception e) {
            }
        }).start();
    }

    // Get angle robot is facing
    public double getHeading() {
        return Math.IEEEremainder(mGyro.getAngle(), 360);
    }

    // Get direction robot is facing
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    // Get position of robot
    public Pose2d getPose() {
        return mOdometer.getPoseMeters();
    }

    @Override
    public void periodic() {
        // update position
        mOdometer.update(getRotation2d(), new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()});
        // Update distances for all modules
        for (SwerveModule module : modules) {
            module.updateDistance();
        }

        // Debug telemetry
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    // Stop the robot
    public void stopModules() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].stop();
        }
    }

    // DRIVE the robot
    public void setModuleStates(SwerveModuleState[] pDesiredStates) {
        // Set speed to max if go over max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(pDesiredStates, Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond);
        // Move modules
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(pDesiredStates[i]);
        }
    }
}
