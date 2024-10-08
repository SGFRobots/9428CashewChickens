package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Module {

    // motors and encoders
    private final TalonFX mDriveMotor;
    private final CANSparkMax mTurnMotor;

    // PID controllers - feedback method
    public final PIDController turningPID;
    public final PIDController drivingPID;

    // aBSOLUTE ENCODER - knows where the wheels are facing at all times
    public final CANcoder absoluteEncoder;
    public final boolean AbsoluteEncoderReversed;
    public final double absoluteEncoderOffset;

    public boolean resetting;

    public SwerveModuleState currentState;

    // turning and driving power/distance
    private double turnOutput;
    private double driveOutput;
    private double driveDistance;
    private double turnDistance;


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
            mTurnMotor.setIdleMode(IdleMode.kCoast);

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

            currentState = new SwerveModuleState(Constants.Mechanical.kPhysicalMaxAngularSpeedRadiansPerSecond, new Rotation2d(getCurrentAngleRad()));
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
                stop();
                return;
            }

            SmartDashboard.putNumber("before" + mDriveMotor.getDeviceID(), pNewState.angle.getDegrees());
            currentState = SwerveModuleState.optimize(pNewState, new Rotation2d(getCurrentAngleRad())); // There is something wrong with this! What is wrong? Who knows!
            // currentState = optimizeStateDeg(pNewState, getCurrentAngleDeg());        
            System.out.println(pNewState.angle.getDegrees());

            // Set power to motorsr
            driveOutput = (currentState.speedMetersPerSecond * Math.cos(turningPID.getPositionError())) / Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond;
            turnOutput = turningPID.calculate(getCurrentAngleRad90(), currentState.angle.getRadians());
            
            mDriveMotor.set(driveOutput);
            mTurnMotor.set(turnOutput); 
            
            // Telemetry
            SmartDashboard.putNumber("turn " + mDriveMotor.getDeviceID() + " output", turnOutput);
            SmartDashboard.putNumber("drive " + mDriveMotor.getDeviceID() + " output", driveOutput);
            SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", currentState.toString());
            
        } else {
            // Reset wheel rotations
            resetRotation();
        }
    }
    
    public double getCurrentAngleDeg() {
        return Math.toDegrees(getCurrentAngleRad());
    }
    
    public double getCurrentAngleRad() {
        double angle = -absoluteEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2;
        return MathUtil.angleModulus(angle - (absoluteEncoderOffset * Math.PI * 2));
    }
    public double getCurrentAngleRad90() {
        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2;
        return MathUtil.inputModulus(angle-(absoluteEncoderOffset*Math.PI*2), -(Math.PI/2), Math.PI/2);
    }
    public SwerveModuleState optimizeStateDeg(SwerveModuleState newState, double currentAngle) {
        double change = Math.abs(currentAngle - newState.angle.getDegrees());
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
        SmartDashboard.putNumber("absolute encoder" + mDriveMotor.getDeviceID(), getCurrentAngleDeg());
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
}