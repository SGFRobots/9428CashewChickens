package frc.robot.subsystems;

// import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SwerveSubsystem extends SubsystemBase{
    // Make instances of all 4 modules
    private final SwerveModule[] modules = {
        // Front Left
        new SwerveModule(
            Constants.MotorPorts.kFLDriveMotorPort,
            Constants.MotorPorts.kFLTurningMotorPort,
            Constants.Reversed.kFLDriveEncoderReversed,
            Constants.Reversed.kFLTurningEncoderReversed,
            Constants.MotorPorts.kFLDriveAbsoluteEncoderPort,
            Constants.Mechanical.kFLDriveAbsoluteEncoderOffsetRad,
            Constants.Reversed.kFLDriveAbsoluteEncoderReversed),
        
        // Front Right
        new SwerveModule(
            Constants.MotorPorts.kFRDriveMotorPort,
            Constants.MotorPorts.kFRTurningMotorPort,
            Constants.Reversed.kFRDriveEncoderReversed,
            Constants.Reversed.kFRTurningEncoderReversed,
            Constants.MotorPorts.kFRDriveAbsoluteEncoderPort,
            Constants.Mechanical.kFRDriveAbsoluteEncoderOffsetRad,
            Constants.Reversed.kFRDriveAbsoluteEncoderReversed),
        
        // Back Left
        new SwerveModule(
            Constants.MotorPorts.kBLDriveMotorPort,
            Constants.MotorPorts.kBLTurningMotorPort,
            Constants.Reversed.kBLDriveEncoderReversed,
            Constants.Reversed.kBLTurningEncoderReversed,
            Constants.MotorPorts.kBLDriveAbsoluteEncoderPort,
            Constants.Mechanical.kBLDriveAbsoluteEncoderOffsetRad,
            Constants.Reversed.kBLDriveAbsoluteEncoderReversed),
        
        // Back Right
        new SwerveModule(
            Constants.MotorPorts.kBRDriveMotorPort,
            Constants.MotorPorts.kBRTurningMotorPort,
            Constants.Reversed.kBRDriveEncoderReversed,
            Constants.Reversed.kBRTurningEncoderReversed,
            Constants.MotorPorts.kBRDriveAbsoluteEncoderPort,
            Constants.Mechanical.kBRDriveAbsoluteEncoderOffsetRad,
            Constants.Reversed.kBRDriveAbsoluteEncoderReversed),
        
    };

    // Positions stored in gyro and mOdometer
    // private final AHRS mGyro;
    private final SwerveDriveOdometry mOdometer;

    public static final Field2d mField2d = new Field2d(); 

    // =) 
    public SwerveSubsystem() {
        // Set up gyro and mOdometer
        // mGyro = new AHRS(SPI.Port.kMXP);
        mOdometer = new SwerveDriveOdometry(Constants.Mechanical.kDriveKinematics,
        new Rotation2d(0), 
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        }, new Pose2d(5.0, 13.5, new Rotation2d()));

        SmartDashboard.putData("Field", mField2d);

        // Reset gyro
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                // Reset position
                // mGyro.reset();
            } catch (Exception e) {
            }
        }).start();
    }

    // Get angle robot is facing
    public double getHeading() {
        return Math.IEEEremainder(90, 360);
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

        double loggingState[] = {
            modules[0].getAbsoluteEncoderRad(), 3,
            modules[1].getAbsoluteEncoderRad(), 3,
            modules[2].getAbsoluteEncoderRad(), 3,
            modules[3].getAbsoluteEncoderRad(), 3
        };

        // Debug telemetry
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumberArray("SwerveModuleStates", loggingState);
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
