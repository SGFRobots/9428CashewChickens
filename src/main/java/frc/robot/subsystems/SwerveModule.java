package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

    // motors and encoders
    public final CANSparkMax driveMotor;
    public final CANSparkMax turnMotor;
    public final RelativeEncoder driveEncoder;
    public final RelativeEncoder turnEncoder;

    public final PIDController turningPIDController;

    // aBSOLUTE ENCODER - knows where the wheels are facing at all times
    public final AnalogInput absoluteEncoder;
    public final boolean absoluteEncoderReversed;
    public final double absoluteEncoderOffset;


    private double previousPosition; // Store previous position to calculate delta distance
    private double totalDistance; // Track the total distance traveled

    public SwerveModule(
        int drivePort, int turnPort, boolean driveReversed, boolean turnReversed, int absoluteEncoderPort, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

            // Motors
            driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
            turnMotor = new CANSparkMax(turnPort, MotorType.kBrushless);
            driveMotor.setInverted(driveReversed);
            turnMotor.setInverted(turnReversed);

            this.previousPosition = 0.0;
            this.totalDistance = 0.0;

            // Encoders
            driveEncoder = driveMotor.getEncoder();
            turnEncoder = turnMotor.getEncoder();

            // Conversions to meters and radians instead of rotations
            driveEncoder.setPositionConversionFactor(Constants.Mechanical.kDriveEncoderRot2Meter);
            driveEncoder.setVelocityConversionFactor(Constants.Mechanical.kDriveEncoderRPM2MeterPerSec);
            turnEncoder.setPositionConversionFactor(Constants.Mechanical.kTurningEncoderRot2Rad);
            turnEncoder.setVelocityConversionFactor(Constants.Mechanical.kTurningEncoderRPM2RadPerSec);

            // Absolute Encoder
            absoluteEncoder = new AnalogInput(absoluteEncoderPort);
            this.absoluteEncoderReversed = absoluteEncoderReversed;
            this.absoluteEncoderOffset = absoluteEncoderOffset;

            //PID Controller - what is this
            turningPIDController = new PIDController(0.5, 0, 0);
            turningPIDController.enableContinuousInput(-Math.PI, Math.PI); // minimize rotations to 180

            // Reset all position
            driveEncoder.setPosition(0);
            turnEncoder.setPosition(getAbsoluteEncoderRad()); // set to current angle (absolute encoders never loses reading)

    }

    // get current angle in radians
    public double getAbsoluteEncoderRad() {
        // Voltage applied over max voltage returns the percentage of a rotation
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        // convert to radians
        angle *= 2.0 * Math.PI;
        
        angle -= absoluteEncoderOffset;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }
 
    // Return all data of the position of the robot - type SwerveModuleState
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition()));
    }

    // Return all data of the position of the robot - type SwerveModulePosition
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            totalDistance,
            new Rotation2d(getAbsoluteEncoderRad())
        );
    }

    // Update the distance traveled of robot
    public void updateDistance(){
        // Get the current encoder position
        double currentPosition = driveEncoder.getPosition(); // This returns counts

        // Calculate the chang in distance
        double countsChange = currentPosition - previousPosition;
        double distanceChange = (countsChange / Constants.Mechanical.kDriveEncoderResolution) * Constants.Mechanical.kWheelCircumferenceMeters;

        // Update total distance
        totalDistance += distanceChange;

        // Update previous position
        previousPosition = currentPosition;
    }


    // Move
    public void setDesiredState(SwerveModuleState state) {
        // Don't move back to 0 after moving
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        // Optimize angle (turn no more than 90 degrees)
        state = SwerveModuleState.optimize(state, getState().angle); 
        // Set power
        driveMotor.set(state.speedMetersPerSecond / Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond);
        turnMotor.set(turningPIDController.calculate(turnEncoder.getPosition(), state.angle.getRadians()));

        // Telemetry
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());

    }

    // Stop moving
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
