package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import swervelib.SwerveDrive;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Module {

    // motors and encoders
    private final TalonFX mDriveMotor;
    private final CANSparkMax mTurnMotor;
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
            System.out.println(mDriveMotor.getDeviceID() + " " + mDriveMotor.getInverted());
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
            
            //PID Controller - change PID values when get feedback
            turningPID = new PIDController(0.05, 0, 0);
            turningPID.enableContinuousInput(-Math.PI, Math.PI); // minimize rotations to 180
            drivingPID = new PIDController(1, 0, 0);
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
            if (pNewState.speedMetersPerSecond < 0.001) {
                // System.out.println("stop" + mDriveMotor.getDeviceID());
                stop();
                return;
            }

            // SmartDashboard.putNumber("speed" + mDriveMotor.getDeviceID(), pNewState.speedMetersPerSecond);
            // Optimize angle (turn no more than 90 degrees)
            // currentState = SwerveModuleState.optimize(pNewState, getState().angle); 
            // SmartDashboard.putNumber("module " + mDriveMotor.getDeviceID() + " before", absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360);
            // SmartDashboard.putNumber("module " + mDriveMotor.getDeviceID() + " optimized", optimizeDeg360(absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360));
            // Rotation2d optimizedAngle = optimizeStateDeg(pNewState, absoluteEncoderOffset)
            // currentState = new SwerveModuleState(pNewState.speedMetersPerSecond, optimizedAngle);
            // SmartDashboard.putNumber("module " + mDriveMotor.getDeviceID() + " optimized", currentState.angle.getDegrees());
            SmartDashboard.putString("setState b4" + mDriveMotor.getDeviceID(), pNewState.toString());
            currentState = optimizeStateDeg(pNewState, getCurrentAngleDeg());
            SmartDashboard.putString("setState after" + mDriveMotor.getDeviceID(), currentState.toString());
        

            // Set power to motorsr
            // turnOutput = turningPID.calculate(mTurnEncoder.getDistance(), currentState.angle.getRadians());
            // SmartDashboard.putNumber("turn " + mDriveMotor.getDeviceID() + " newstate", currentState.angle.getDegrees());
            // SmartDashboard.putNumber("turn " + mDriveMotor.getDeviceID() + " change", (absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360) - currentState.angle.getDegrees());
            driveOutput = (currentState.speedMetersPerSecond * Math.cos(turningPID.getPositionError())) / Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond;
            turnOutput = turningPID.calculate(getCurrentAngleRad(), currentState.angle.getRadians());
            // turnOutput = (Math.abs((optimizeDeg360(absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360)) - currentState.angle.getDegrees()) < 3) ? 5 : 0;
            SmartDashboard.putNumber("turn " + mDriveMotor.getDeviceID() + " output", turnOutput);
            SmartDashboard.putNumber("drive " + mDriveMotor.getDeviceID() + " output", driveOutput);
            
            mDriveMotor.set(driveOutput);
            mTurnMotor.set(turnOutput); 
            // mTurnMotor.set(turnOutput / Constants.Mechanical.kPhysicalMaxAngularSpeedRadiansPerSecond);
            
            // Telemetry
            // SmartDashboard.putNumber("angle", currentState.angle.getRadians());
            // SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", currentState.toString());
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
    
    public double getCurrentAngleDeg() {
        return Math.toDegrees(getCurrentAngleRad());
    }
    
    public double getCurrentAngleRad() {
        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2;
        if (angle > Math.PI) {
            angle -= Math.PI;
        }
        return MathUtil.angleModulus(angle - (absoluteEncoderOffset * Math.PI * 2));
    }
    
    // Angle optimization - Need work
    public double optimizeRad180(double angleRad) {
        if(Math.abs(angleRad) <= (Math.PI / 2)) {
            return angleRad;
        }
        return (Math.abs(angleRad) - (Math.PI)) * ((angleRad < 0) ? -1 : 1);
    }
    
    public double optimizeDeg180(double angleDeg) {
        if(Math.abs(angleDeg) <= 90) {
            return angleDeg;
        }
        return (Math.abs(angleDeg) - (180)) * ((angleDeg < 0) ? -1 : 1);
    }
    
    public SwerveModuleState optimizeStateDeg(SwerveModuleState newState, double currentAngle) {
        double change = Math.abs(currentAngle - newState.angle.getDegrees());
        SmartDashboard.putNumber("change "+mDriveMotor.getDeviceID(), change);
        if ((change > 90) && (change < 270)) {
            return new SwerveModuleState(-newState.speedMetersPerSecond, newState.angle.rotateBy(Rotation2d.fromDegrees(180)));
        }
        return newState;
    }
    
    public SwerveModuleState optimizeStateRad(SwerveModuleState newState, double currentAngle) {
        double change = currentAngle - newState.angle.getRadians();
        if ((change > (Math.PI / 2)) || (change < (3 * Math.PI / 2))) {
            return new SwerveModuleState(-newState.speedMetersPerSecond, newState.angle.rotateBy(Rotation2d.fromDegrees(180)));
        }
        return newState;
    }
    
    public double optimizeRad360(double angleRad) {
        if(angleRad <= (Math.PI / 2)) {
            return angleRad;
        } else if (angleRad <= (3*Math.PI / 2)) {
            return angleRad - Math.PI;
        }
        return angleRad - (Math.PI * 2);
    }
    
    public double optimizeDeg360(double angleDeg) {
        if(angleDeg <= (90)) {
            return angleDeg;
        } else if (angleDeg <= (270)) {
            return angleDeg - 180;
        }
        return angleDeg - (360);
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
        // SmartDashboard.putNumber("drive motor" + mDriveMotor.getDeviceID(), mDriveMotor.getPosition().getValue());
        SmartDashboard.putNumber("absolute encoder" + mDriveMotor.getDeviceID(), absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    }

    // Turn module back to 0 position
    public void resetRotation() {
        // System.out.println(mDriveMotor.getDeviceID());
        // turnOutput = turningPID.calculate(absoluteEncoder.getAbsolutePosition().getValueAsDouble(), absoluteEncoderOffset);
        // if (Math.abs(turnOutput) < Constants.Mechanical.kDeadzone) {
        //     resetting = false;
        //     return;
        // }
        // mTurnMotor.set(turnOutput);
        // mDriveMotor.set(0);

        // absoluteEncoder.getConfigurator().setPosition(0);
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