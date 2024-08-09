package frc.robot.subsystems;

// import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Constants.Mechanical.kModulePositions;



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
    // private final AnalogGyro mGyro;
    private final ADXRS450_Gyro mGyro = new ADXRS450_Gyro();
    private final ADXRS450_GyroSim mGyroSim = new ADXRS450_GyroSim(mGyro); // this? ? ya its only used in this file yes bc its the gyro yeah i knpow hey what do you call the part of the class before the constructor where you declare all variables? is this a test or do you not know i do not know me neither ok make up a name uhhh Decelorations no a related name yeah thats where you declare your variabbles oh you mean declarations yeah ok
    private final SwerveDriveOdometry mOdometer;
    

    public static final Field2d mField2d = new Field2d(); 

    Pose2d[] mModulePose = {
            new Pose2d(),
            new Pose2d(),
            new Pose2d(),
            new Pose2d()
    };
    // =) 
    public SwerveSubsystem() {
        // Set up gyro and mOdometer
        // mGyro = new AHRS(SPI.Port.kMXP);
        // mGyro = new AnalogGyro(1);

        mOdometer = new SwerveDriveOdometry(Constants.Mechanical.kDriveKinematics,
        mGyro.getRotation2d(), 
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        });
        SmartDashboard.putData("Field", mField2d);

        // Reset gyro
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                // Reset position
                // mGyro.reset();
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        mGyro.reset();
    }

    // Get angle robot is facing
    public double getHeading() {
        return mGyro.getRotation2d().getDegrees();
    } // what gyro they have?

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
        mOdometer.update(mGyro.getRotation2d(), new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()});

        // Update distances for all modules
        for (SwerveModule module : modules) {
            module.updateDistance();
        }
        // Update Pose for swerve modules - Position of the rotation and the translation matters
        for (int i = 0; i < modules.length; i++){
            Translation2d updatedModulePosition = kModulePositions[i].rotateBy(mGyro.getRotation2d()).plus(getPose().getTranslation());
            // Module heading is the angle relative to the chasis heading
            mModulePose[i] = new Pose2d(updatedModulePosition, modules[i].getState().angle.plus(getPose().getRotation()));
        }
        // Sets robot position on the field
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
        SmartDashboard.putNumber("Robot Heading", mGyro.getAngle());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumberArray("SwerveModuleLOGGINGStates", loggingState);

    }
    public void resetOdometry(Pose2d pose) {
        mOdometer.resetPosition(mGyro.getRotation2d(),new SwerveModulePosition[]{
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()}, pose);
        for (SwerveModule module : modules) {
            module.resetEncoders();
        }; // it does just say reset encoders by itself so idk ...
    }

    // Stop the robot
    public void stopModules() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].stop();
        }
    }

     // DRIVE the robot // what ikd its in the other codee and we dont have it  
    public void drive(double xSpeed, double ySpeed, double turningSpeed) { // this is also in their other code these are two seperate thigns. their drive is our setmodulestates. their setmodulestates isnt used really ya it syas its used in their robot container really yea line 106
        // Set desire chassis speeds
        // Field Orientation
        ChassisSpeeds chassisSpeed;
        chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, turningSpeed, getRotation2d());

        // Convert chassis speeds to module states
        SwerveModuleState[] moduleStates = Constants.Mechanical.kDriveKinematics.toSwerveModuleStates(chassisSpeed);
        
        // Set speed to max if go over max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond);
        // Move modulesss
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(moduleStates[i]);
        }
    } 
        // bro what you doiongmy bad shake head i deleted something in accident
        // whats next bud? im gonna add something on accident
    public void resetEncoders() {
        for (SwerveModule module : modules) {
            module.resetEncoders();
        }
    }
    
    //their get heading is different than ours ... why ... our getheading is definitely wrong. i thought that this morning too ok that makes sense ill rewrite it okkk
    // public returnvalue functionname(parameters) {
    //     actions semicolon
    // } // it is the format when defining a methodi know
} ///woah what does this one do wow thats cool