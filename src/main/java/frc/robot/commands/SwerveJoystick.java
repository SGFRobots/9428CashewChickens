package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystick extends Command {
    private final SwerveSubsystem mSwerveSubsystem;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final XboxController mController;

    // Constructor
    public SwerveJoystick(SwerveSubsystem pSwerveSubsystem, XboxController pController) {
        // Subsystem and controller instances
        mSwerveSubsystem = pSwerveSubsystem;
        mController = pController;

        // Slew Rate Limiter smooths the robot's accelerations
        xLimiter = new SlewRateLimiter(Constants.Mechanical.kTeleDriveMaxAccelerationUnitsPerSecond);
        yLimiter = new SlewRateLimiter(Constants.Mechanical.kTeleDriveMaxAccelerationUnitsPerSecond);
        turningLimiter = new SlewRateLimiter(Constants.Mechanical.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(mSwerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // Get joystick inputs
        double xSpeed = mController.getRawAxis(Constants.Controllers.LeftYPort);
        double ySpeed = mController.getRawAxis(Constants.Controllers.LeftXPort);
        double turningSpeed = mController.getRawAxis(Constants.Controllers.RightXPort); 
        
        // Apply Deadzone
        xSpeed = Math.abs(xSpeed) > Constants.Mechanical.kDeadzone ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.Mechanical.kDeadzone ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > Constants.Mechanical.kDeadzone ? turningSpeed : 0.0;
        
        // Make Driving Smoother using Slew Rate Limiter - less jerky by accelerating slowly
        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        turningSpeed = turningLimiter.calculate(turningSpeed);

        // Calculate speed in m/s
        xSpeed *= Constants.Mechanical.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed *= Constants.Mechanical.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed *= Constants.Mechanical.kTeleDriveMaxSpeedMetersPerSecond;

        // SmartDashboard.putNumber("xSpeed", xSpeed);
        // SmartDashboard.putNumber("ySpeed", ySpeed);
        // SmartDashboard.putNumber("turningSpeed", turningSpeed);

        // Drive
        mSwerveSubsystem.drive(xSpeed, ySpeed, turningSpeed, false);

        // Test
        // mSwerveSubsystem.driveIndividualModule(xSpeed, turningSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        mSwerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
