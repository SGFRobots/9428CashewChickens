package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Module {

    // motors and encoders
    public final TalonFX mDriveMotor;
    public final CANSparkMax mTurnMotor;
    // public final Encoder mDriveEncoder;
    // public final Encoder mTurnEncoder;
    // public final RelativeEncoder mTurnEncoder;

    // PID controllers - feedback method
    public final PIDController turningPID;
    public final PIDController drivingPID;

    // aBSOLUTE ENCODER - knows where the wheels are facing at all times
    public final CANcoder absoluteEncoder;
    public final boolean AbsoluteEncoderReversed;
    public final double absoluteEncoderOffset;

    public boolean resetting;

    public SwerveModuleState currentState;
    // public SwerveModuleState desiredState;

    // turning and driving power/distance
    private double turnOutput;
    private double driveOutput;
    private double driveDistance;
    private double turnDistance;

    // Simulated hardwares
    // public final EncoderSim mDriveEncoderSim;
    // public final EncoderSim mTurnEncoderSim;
    public final FlywheelSim mDriveMotorSim;
    public final FlywheelSim mTurnMotorSim;

    // Constructores
    public Module(
        int pDrivePort, int pTurnPort, boolean pDriveReversed, boolean pTurnReversed,
        int pAbsoluteEncoderPort, double pAbsoluteEncoderOffset, boolean pAbsoluteEncoderReversed,
        int[] pDriveEncoderPorts, int[] pTurnEncoderPorts, boolean pDEncoderReversed, boolean pTEncoderReversed) {

            // Motors
            mDriveMotor = new TalonFX(pDrivePort);
            mTurnMotor = new CANSparkMax(pTurnPort, MotorType.kBrushless);
            mDriveMotor.setInverted(pDriveReversed);
            mTurnMotor.setInverted(pTurnReversed);

            // Encoders
            // mDriveEncoder = new Encoder(pDriveEncoderPorts[0], pDriveEncoderPorts[1]);
            // mTurnEncoder = new Encoder(pTurnEncoderPorts[0], pTurnEncoderPorts[1]);
            // mDriveEncoder.setReverseDirection(pDEncoderReversed);
            // mTurnEncoder.setReverseDirection(pTEncoderReversed);

            // Simulated Hardwarwes
            // mDriveEncoderSim = new EncoderSim(mDriveEncoder);
            // mTurnEncoderSim = new EncoderSim(mTurnEncoder);
            mDriveMotorSim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(Constants.Mechanical.kVoltSecondPerRadian, Constants.Mechanical.kVoltSecondsSquaredPerRadian),
                Constants.Mechanical.kDriveGearBox, Constants.Mechanical.kDriveMotorGearRatio);
            mTurnMotorSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(Constants.Mechanical.kVoltSecondPerRadian, Constants.Mechanical.kVoltSecondsSquaredPerRadian),
                Constants.Mechanical.kTurnGearBox, Constants.Mechanical.kTurningMotorGearRatio);
            driveDistance = 0;
            turnDistance = 0;

            // Convert to meters and radians instead of rotations
            // mDriveEncoder.setDistancePerPulse(Constants.Mechanical.kDistancePerPulse);
            // mTurnEncoder.setDistancePerPulse(Constants.Mechanical.kDistancePerPulse);
            // mDriveMotor.setPositionConversionFactor(Constants.Mechanical.kDriveEncoderRot2Meter);
            // driveEncoder.setVelocityConversionFactor(Constants.Mechanical.kDriveEncoderRPM2MeterPerSec);
            // mTurnEncoder.setPositionConversionFactor(Constants.Mechanical.kTurningEncoderRot2Rad);
            // mTurnEncoder.setVelocityConversionFactor(Constants.Mechanical.kTurningEncoderRPM2RadPerSec);
            
            // Absolute Encoder
            absoluteEncoder = new CANcoder(pAbsoluteEncoderPort);
            AbsoluteEncoderReversed = pAbsoluteEncoderReversed;
            absoluteEncoderOffset = pAbsoluteEncoderOffset;
            absoluteEncoder.getConfigurator().setPosition(absoluteEncoderOffset);
            
            //PID Controller - change PID values when get feedback
            turningPID = new PIDController(0.4, 0, 0.01);
            turningPID.enableContinuousInput(-Math.PI, Math.PI); // minimize rotations to 180
            drivingPID = new PIDController(0.4, 0, 0.01);
            // P = rate of change
            // I = rate of change of D
            // D = rate of change of P (slow when get closer)
            
            // Reset all position
            // driveEncoder.setPosition(0);
            // mDriveEncoder.reset();
            // mTurnEncoder.reset(); // set to current angle (absolute encoders never loses reading)
            
            currentState = new SwerveModuleState(Constants.Mechanical.kPhysicalMaxAngularSpeedRadiansPerSecond, new Rotation2d(getAbsoluteEncoderRad()));
    }

    // get current angle in radians
    public double getAbsoluteEncoderRad() {
        // Voltage applied over max voltage returns the percentage of a rotation
        double angle = absoluteEncoder.getSupplyVoltage().getValue() / RobotController.getVoltage5V();        // convert to radians
        angle *= 2.0 * Math.PI;
        
        angle -= absoluteEncoderOffset;
        return angle * (AbsoluteEncoderReversed ? -1.0 : 1.0);
    }
 
    // Return all data of the position of the robot - type SwerveModuleState
    public SwerveModuleState getState() {
        return currentState;
    }

    // Return all data of the position of the robot - type SwerveModulePosition
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveDistance, new Rotation2d(turnDistance)
        );
    }

    // Move module
    public void setDesiredState(SwerveModuleState pNewState) {
        // If normal
        if (!resetting) {
            // Don't move back to 0 after moving
            if (Math.abs(pNewState.speedMetersPerSecond) < 0.001) {
                stop();
                return;
            }

            // Optimize angle (turn no more than 90 degrees)
            // SmartDashboard.putNumber("module " + mDriveMotor.getDeviceID(), pNewState.angle.getRadians());
            currentState = SwerveModuleState.optimize(pNewState, getState().angle); 
            // Rotation2d optimizedAngle = new Rotation2d(optimize(pNewState.angle.getDegrees()));
            // currentState = new SwerveModuleState(pNewState.speedMetersPerSecond, optimizedAngle);
            // SmartDashboard.putNumber("module " + mDriveMotor.getDeviceID() + " optimized", currentState.angle.getRadians());

            // Set power to motors
            // driveOutput = drivingPID.calculate(mDriveEncoder.getRate(), currentState.speedMetersPerSecond);
            // turnOutput = turningPID.calculate(mTurnEncoder.getDistance(), currentState.angle.getRadians());
            // System.out.println(roundToMeters(mDriveMotor.getVelocity().getValueAsDouble()));
            driveOutput = drivingPID.calculate(roundToMeters(mDriveMotor.getVelocity().getValueAsDouble()), currentState.speedMetersPerSecond);
            // driveOutput = currentState.speedMetersPerSecond / 25;
            turnOutput = turningPID.calculate(signAngle(absoluteEncoder.getAbsolutePosition().getValueAsDouble()), currentState.angle.getDegrees());
            SmartDashboard.putNumber("drive " + mDriveMotor.getDeviceID() + " pid", pNewState.speedMetersPerSecond);
            SmartDashboard.putNumber("turn " + mDriveMotor.getDeviceID() + " pid", pNewState.angle.getDegrees());
            mDriveMotor.set(driveOutput / 50);
            mTurnMotor.set(turnOutput / 50);

            // Telemetry
            SmartDashboard.putNumber("angle", currentState.angle.getRadians());
            SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", currentState.toString());
        } else {
            // Reset wheel rotations
            resetRotation();
        }
    }

    public double signAngle(double deg) {
        return (deg <= 180) ? deg : (deg - 360);
    }
    
    public double roundToMeters(double rps) {
        return Constants.Mechanical.kWheelCircumferenceMeters * rps;
    }

    // Angle optimization - Need work
    public double optimize(double angleRad) {
        if(angleRad <= (Math.PI / 2)) {
            return angleRad;
        } else if (angleRad <= (3*Math.PI / 2)) {
            return angleRad - Math.PI;
        }
        return angleRad - (Math.PI * 2);
    }

    // Test one module at a time
    public void driveIndividually(double speed, double rotation) {
        mDriveMotor.set(speed);
        mTurnMotor.set(rotation);
    }

    // Telemetry
    public void periodic() {
        // SmartDashboard.putNumber("mv: drive motor" + mDriveMotor.getDeviceID(), mDriveMotor.getMotorVoltage().getValue());
        // SmartDashboard.putNumber("sv: drive motor" + mDriveMotor.getDeviceID(), mDriveMotor.getSupplyVoltage().getValue());
        // SmartDashboard.putNumber("mv: turn motor" + mTurnMotor.getDeviceId(), turnOutput);
        SmartDashboard.putNumber("absolute encoder" + absoluteEncoder.getDeviceID(), absoluteEncoder.getAbsolutePosition().getValue());
        // SmartDashboard.putNumber("drive motor" + mDriveMotor.getDeviceID(), mDriveMotor.getPosition().getValue());
    }

    // Turn module back to 0 position
    public void resetRotation() {
        System.out.println(mDriveMotor.getDeviceID());
        turnOutput = turningPID.calculate(absoluteEncoder.getAbsolutePosition().getValueAsDouble(), absoluteEncoderOffset);
        if (Math.abs(turnOutput) < Constants.Mechanical.kDeadzone) {
            resetting = false;
            return;
        }
        mTurnMotor.set(turnOutput);
        mDriveMotor.set(0);
    }

    // Stop all motors
    public void stop() {
        mDriveMotor.set(0);
        mTurnMotor.set(0);
    }

    // Simulate module
    public void simulationPeriodic(double dt) {
        // Simulate motor movemnets
        mDriveMotorSim.setInputVoltage(driveOutput / Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond * RobotController.getBatteryVoltage());
        mTurnMotorSim.setInputVoltage(turnOutput / Constants.Mechanical.kPhysicalMaxAngularSpeedRadiansPerSecond * RobotController.getBatteryVoltage());
        mDriveMotorSim.update(dt);
        mTurnMotorSim.update(dt);

        // Simulate drive encoder
        // driveDistance += mDriveMotorSim.getAngularVelocityRadPerSec() * dt;
        // mDriveEncoderSim.setDistance(driveDistance);
        // mDriveEncoderSim.setRate(mDriveMotorSim.getAngularVelocityRadPerSec());

        // // Simulate turn encoder
        // turnDistance += mTurnMotorSim.getAngularVelocityRadPerSec() * dt;
        // mTurnEncoderSim.setDistance(turnDistance);
        // mTurnEncoderSim.setRate(mTurnMotorSim.getAngularVelocityRadPerSec());
    }
}