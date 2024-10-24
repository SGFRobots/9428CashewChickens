package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    // private ClosedLoopGeneralConfigs configs;

    // PID controllers - feedback method
    private final PIDController turningPID;

    // aBSOLUTE ENCODER - knows where the wheels are facing at all times
    private final CANcoder absoluteEncoder;
    private final boolean AbsoluteEncoderReversed;
    private final double absoluteEncoderOffset;

    public boolean resetting;

    private SwerveModuleState currentState;

    // turning and driving power/distance
    private double turnOutput;
    private double driveOutput;
    private double driveDistance;
    private double turnDistance;


    // Constructores
    public Module(
        int pDrivePort, int pTurnPort, boolean pDriveReversed, boolean pTurnReversed,
        int pAbsoluteEncoderPort, double pAbsoluteEncoderOffset, boolean pAbsoluteEncoderReversed) {

            // Motors
            mDriveMotor = new TalonFX(pDrivePort);
            mTurnMotor = new CANSparkMax(pTurnPort, MotorType.kBrushless);
            mDriveMotor.setInverted(pDriveReversed);
            mTurnMotor.setInverted(pTurnReversed);
            mTurnMotor.setIdleMode(IdleMode.kCoast);
            mDriveMotor.setNeutralMode(NeutralModeValue.Brake);

            // Absolute Encoder
            absoluteEncoder = new CANcoder(pAbsoluteEncoderPort);
            AbsoluteEncoderReversed = pAbsoluteEncoderReversed;
            absoluteEncoderOffset = pAbsoluteEncoderOffset;
            
            //PID Controller - change PID values when get feedback
            turningPID = new PIDController(0.05, 0, 0);
            turningPID.enableContinuousInput(-Math.PI, Math.PI); // minimize rotations to 180
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

            // Optimize angle
            currentState = SwerveModuleState.optimize(pNewState, new Rotation2d(getCurrentAngleRad()));       
            
            // Set power to motor
            driveOutput = (currentState.speedMetersPerSecond * Math.cos(turningPID.getPositionError())) / Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond * 3;
            turnOutput = turningPID.calculate(getCurrentAngleRad(), currentState.angle.getRadians()) * Constants.Mechanical.kPhysicalMaxAngularSpeedRadiansPerSecond * 2;
            mDriveMotor.set(driveOutput);
            mTurnMotor.set(turnOutput); 
            
            // Telemetry
            SmartDashboard.putNumber("before" + mDriveMotor.getDeviceID(), pNewState.angle.getDegrees());
            SmartDashboard.putNumber("turn " + mDriveMotor.getDeviceID() + " output", turnOutput);
            SmartDashboard.putNumber("drive " + mDriveMotor.getDeviceID() + " output", driveOutput);
            SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", currentState.toString());
            
        } else {
            // Reset wheel rotations
            resetRotation();
        }
    }
    
    // Read current module angle in Radians
    public double getCurrentAngleRad() {
        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2;
        angle *= (AbsoluteEncoderReversed) ? -1 : 1;
        return MathUtil.angleModulus(angle - (absoluteEncoderOffset * Math.PI * 2));
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
        turnOutput = turningPID.calculate(absoluteEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2, absoluteEncoderOffset * Math.PI * 2) * Constants.Mechanical.kPhysicalMaxAngularSpeedRadiansPerSecond;
        if (Math.abs(turnOutput) < 0.01) {
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