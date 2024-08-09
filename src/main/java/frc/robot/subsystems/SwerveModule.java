package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

    // motors and encoders
    public final TalonFX mDriveMotor;
    public final CANSparkMax mTurnMotor;
    public final Encoder mDriveEncoder;
    public final Encoder mTurnEncoder;

    // PID controllers
    public final PIDController turningPID;
    public final PIDController drivingPID;

    // aBSOLUTE ENCODER - knows where the wheels are facing at all times
    public final AnalogInput absoluteEncoder;
    public final boolean AbsoluteEncoderReversed;
    public final double absoluteEncoderOffset;

    public SwerveModuleState currentState;
    // public SwerveModuleState desiredState;

    private double turnOutput;
    private double driveOutput;
    private double driveDistance;
    private double turnDistance;

    // Simulated hardwares
    public final EncoderSim mDriveEncoderSim;
    public final EncoderSim mTurnEncoderSim;
    public final FlywheelSim mDriveMotorSim;
    public final FlywheelSim mTurnMotorSim;

    public SwerveModule(
        int pDrivePort, int pTurnPort, boolean pDriveReversed, boolean pTurnReversed,
        int pAbsoluteEncoderPort, double pAbsoluteEncoderOffset, boolean pAbsoluteEncoderReversed,
        int[] pDriveEncoderPorts, int[] pTurnEncoderPorts, boolean pDEncoderReversed, boolean pTEncoderReversed) {

            // Motors
            // ignore this comment
            mDriveMotor = new TalonFX(pDrivePort);
            mTurnMotor = new CANSparkMax(pTurnPort, MotorType.kBrushless);
            mDriveMotor.setInverted(pDriveReversed);
            mTurnMotor.setInverted(pTurnReversed);

            

            // Encoders
            mDriveEncoder = new Encoder(pDriveEncoderPorts[0], pDriveEncoderPorts[1]);
            mTurnEncoder = new Encoder(pTurnEncoderPorts[0], pTurnEncoderPorts[1]);
            mDriveEncoder.setReverseDirection(pDEncoderReversed);
            mTurnEncoder.setReverseDirection(pTEncoderReversed);

            // Simulated Hardwarwes
            mDriveEncoderSim = new EncoderSim(mDriveEncoder);
            mTurnEncoderSim = new EncoderSim(mTurnEncoder);
            mDriveMotorSim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(Constants.Mechanical.kVoltSecondPerRadian, Constants.Mechanical.kVoltSecondsSquaredPerRadian),
                Constants.Mechanical.kDriveGearBox, Constants.Mechanical.kDriveMotorGearRatio);
            mTurnMotorSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(Constants.Mechanical.kVoltSecondPerRadian, Constants.Mechanical.kVoltSecondsSquaredPerRadian),
                Constants.Mechanical.kTurnGearBox, Constants.Mechanical.kTurningMotorGearRatio);
            driveDistance = 0;
            turnDistance = 0;

            // Conversions to meters and radians instead of rotations
            // mDriveMotor.setPositionConversionFactor(Constants.Mechanical.kDriveEncoderRot2Meter);
            // driveEncoder.setVelocityConversionFactor(Constants.Mechanical.kDriveEncoderRPM2MeterPerSec);
            mDriveEncoder.setDistancePerPulse(Constants.Mechanical.kDistancePerPulse);
            mTurnEncoder.setDistancePerPulse(Constants.Mechanical.kDistancePerPulse);
            // mTurnEncoder.setPositionConversionFactor(Constants.Mechanical.kTurningEncoderRot2Rad);
            // mTurnEncoder.setVelocityConversionFactor(Constants.Mechanical.kTurningEncoderRPM2RadPerSec);
            
            // Absolute Encoder
            absoluteEncoder = new AnalogInput(pAbsoluteEncoderPort);
            AbsoluteEncoderReversed = pAbsoluteEncoderReversed;
            absoluteEncoderOffset = pAbsoluteEncoderOffset;
            
            //PID Controller - what is this
            turningPID = new PIDController(0.5, 0, 0);
            turningPID.enableContinuousInput(-Math.PI, Math.PI); // minimize rotations to 180
            drivingPID = new PIDController(0.5, 0, 0);
            
            // Reset all position
            // driveEncoder.setPosition(0);
            mDriveEncoder.reset();
            mTurnEncoder.reset(); // set to current angle (absolute encoders never loses reading)
            
            currentState = new SwerveModuleState(0, new Rotation2d(getAbsoluteEncoderRad()));
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
        return currentState;
    }

    // Return all data of the position of the robot - type SwerveModulePosition
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveDistance, new Rotation2d(turnDistance)
        );
    }

    // Move
    public void setDesiredState(SwerveModuleState pNewState) {
        // Don't move back to 0 after moving
        if (Math.abs(pNewState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        // Optimize angle (turn no more than 90 degrees)
        currentState = SwerveModuleState.optimize(pNewState, getState().angle); 
        // Set power
        driveOutput = drivingPID.calculate(mDriveEncoder.getDistance(), currentState.speedMetersPerSecond);
        turnOutput = turningPID.calculate(mTurnEncoder.getDistance(), currentState.angle.getRadians());
        mDriveMotor.set(driveOutput);
        mTurnMotor.set(turnOutput);

        // Telemetry
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", currentState.toString());
    }

    //ok we need this to reset the encoders you do realize it has a built in chat function wjhattttttttttttt reallu? yeah click on the bar below you should find it
    public void resetEncoders() {
        mDriveEncoder.reset();
        mTurnEncoder.reset(); //does the turn encoder need to be reset? it will lose angle position; thats what the other team has... if we have the kraken motor for turning, we need to change it. Should we do that now or when if its harder to change it later then we should do it now ok what morot did the other team have? can or talon? i can give you a link to their files if you want to look at it ok
    }

    // https://github.com/4201VitruvianBots/2023SwerveSim/blob/adc5d029f32def359702915bc607c4a7733bbedb/2023SwerveControllerCommand/src/main/java/frc/robot/subsystems/SwerveModule.java#L157
    // Stop moving wjat are uoutalking about  ?? idk theu have a spark motor but 2 Encoder s as in ecoder class
    // they made three different examples for different motors https://github.com/4201VitruvianBots/2023SwerveSim/tree/adc5d029f32def359702915bc607c4a7733bbedb that might help...? falcon has talonfx yeah i should have been looking at that huh it not so different so it doesnt matter ok i might have to give up calculus :oh( no!! we;ll see why?? because calculus is a junior/senior class. i have more chance of getting any other ap classes than calculus oh that sucks yeah :( but anyways why ios there a )?? the example code created a freaking hashmap for the modules cry-emoji hey i didnt make it they are more advanced than us yeah but its complicated so i have no intention of replicateing that part so what to do ??everything els ok e
    public void stop() {
        mDriveMotor.set(0);
        mTurnMotor.set(0);
    }

    public void simulationPeriodic(double dt) {
        mDriveMotorSim.setInputVoltage(driveOutput / Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond * RobotController.getBatteryVoltage());
        mTurnMotorSim.setInputVoltage(turnOutput / Constants.Mechanical.kPhysicalMaxAngularSpeedRadiansPerSecond * RobotController.getBatteryVoltage());

        mDriveMotorSim.update(dt);
        mTurnMotorSim.update(dt);

        driveDistance += mDriveMotorSim.getAngularVelocityRadPerSec() * dt;
        mDriveEncoderSim.setDistance(driveDistance);
        mDriveEncoderSim.setRate(mDriveMotorSim.getAngularVelocityRadPerSec());

        turnDistance += mTurnMotorSim.getAngularVelocityRadPerSec() * dt;
        mTurnEncoderSim.setDistance(turnDistance);
        mTurnEncoderSim.setRate(mTurnMotorSim.getAngularVelocityRadPerSec());
    }
}// should i take form the one i was using or the falcon one 
// idk what are you diong? keep on doing what i was doing in the swerve subsystem what are you doing there? what are you trying to implememtnt well i was seeing what they have that we dont and implementing it over here. 
//well obviously make sure to check what those things are for, then we casee if we really need it rodger rodger but that didnt answer my question good point uhhh where you were looking before the swerve controller command yeah that one ok thumbsup
// you want to do subsystem or module first? or what =ever you were doing before