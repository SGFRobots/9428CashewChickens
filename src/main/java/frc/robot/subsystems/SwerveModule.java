package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
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
    public final TalonFX mDriveMotor;
    public final CANSparkMax mTurnMotor;
    public final Encoder mDriveEncoder;
    public final RelativeEncoder mTurnEncoder;

    public final PIDController turningPIDController;

    // aBSOLUTE ENCODER - knows where the wheels are facing at all times
    public final AnalogInput absoluteEncoder;
    public final boolean AbsoluteEncoderReversed;
    public final double absoluteEncoderOffset;


    private double previousPosition; // Store previous position to calculate delta distance
    private double totalDistance; // Track the total distance traveled

    public SwerveModule(
        int pDrivePort, int pTurnPort, boolean pDriveReversed, boolean pTurnReversed, int pAbsoluteEncoderPort, double pAbsoluteEncoderOffset, boolean pAbsoluteEncoderReversed) {

            // Motors
            mDriveMotor = new TalonFX(pDrivePort);
            mTurnMotor = new CANSparkMax(pTurnPort, MotorType.kBrushless);
            mDriveMotor.setInverted(pDriveReversed);
            mTurnMotor.setInverted(pTurnReversed);

            previousPosition = 0.0; 
            totalDistance = 0.0;

            // Encoders
            mDriveEncoder = new Encoder(null, null);
            mTurnEncoder = mTurnMotor.getEncoder();

            // Conversions to meters and radians instead of rotations
            // mDriveMotor.setPositionConversionFactor(Constants.Mechanical.kDriveEncoderRot2Meter);
            // driveEncoder.setVelocityConversionFactor(Constants.Mechanical.kDriveEncoderRPM2MeterPerSec);
            mDriveEncoder.setDistancePerPulse(Constants.Mechanical.kDistancePerPulse);
            mTurnEncoder.setPositionConversionFactor(Constants.Mechanical.kTurningEncoderRot2Rad);
            mTurnEncoder.setVelocityConversionFactor(Constants.Mechanical.kTurningEncoderRPM2RadPerSec);

            // Absolute Encoder
            absoluteEncoder = new AnalogInput(pAbsoluteEncoderPort);
            AbsoluteEncoderReversed = pAbsoluteEncoderReversed;
            absoluteEncoderOffset = pAbsoluteEncoderOffset;

            //PID Controller - what is this
            turningPIDController = new PIDController(0.5, 0, 0);
            turningPIDController.enableContinuousInput(-Math.PI, Math.PI); // minimize rotations to 180

            // Reset all position
            // driveEncoder.setPosition(0);
            mDriveEncoder.reset();
            mTurnEncoder.setPosition(getAbsoluteEncoderRad()); // set to current angle (absolute encoders never loses reading)

    }

    // get current angle in radians
    public double getAbsoluteEncoderRad() {
        // Voltage applied over max voltage returns the percentage of a rotation
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        // convert to radians
        angle *= 2.0 * Math.PI;
        
        angle -= absoluteEncoderOffset;
        return angle * (AbsoluteEncoderReversed ? -1.0 : 1.0);
    }
 
    // Return all data of the position of the robot - type SwerveModuleState
    public SwerveModuleState getState() {
        return new SwerveModuleState(mDriveEncoder.getRate(), new Rotation2d(mTurnEncoder.getPosition()));
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
        double currentPosition = mDriveEncoder.getDistance(); // This returns counts

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
        mDriveMotor.set(state.speedMetersPerSecond / Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond);
        mTurnMotor.set(turningPIDController.calculate(mTurnEncoder.getPosition(), state.angle.getRadians()));

        // Telemetry
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());

    }

    // Stop moving
    public void stop() {
        mDriveMotor.set(0);
        mTurnMotor.set(0);
    }
}
