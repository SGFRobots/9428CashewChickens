package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;


import edu.wpi.first.wpilibj.XboxController;

public class SwerveJoystick extends Command {
    
    private final SwerveSubsystem swerveSubsystem;
    // private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final XboxController controller;

    public SwerveJoystick(SwerveSubsystem swerveSubsystem,
    XboxController controller) {
        this.swerveSubsystem = swerveSubsystem;
        this.controller = controller;
        // this.fieldOrientedFunction = controller.;
        this.xLimiter = new SlewRateLimiter(Constants.Mechanical.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(Constants.Mechanical.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(Constants.Mechanical.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Get joystick inputs
        double xSpeed = controller.getLeftX();
        double ySpeed = controller.getLeftY();
        double turningSpeed = controller.getRightX();
        
        // Apply Deadzone
        xSpeed = Math.abs(xSpeed) > Constants.Mechanical.kDeadzone ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.Mechanical.kDeadzone ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > Constants.Mechanical.kDeadzone ? turningSpeed : 0.0;

        // Make Driving Smoother (No Wheelies)
        xSpeed = xLimiter.calculate(xSpeed) * Constants.Mechanical.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.Mechanical.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * Constants.Mechanical.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // Set desire chassis speeds
        ChassisSpeeds chassisSpeed;
        // if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        // } else {
            // Relative to robot
            // chassisSpeed = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        // }

        // Convert chassis speeds to module states
        SwerveModuleState[] moduleStates = Constants.Mechanical.kDriveKinematics.toSwerveModuleStates(chassisSpeed);

        // Drive
        swerveSubsystem.setModuleStates(moduleStates);
        
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
