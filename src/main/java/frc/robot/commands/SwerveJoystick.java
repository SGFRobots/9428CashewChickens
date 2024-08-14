package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveJoystick extends Command {
    
    private final SwerveSubsystem mSwerveSubsystem;
    // private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final XboxController mController;

    public SwerveJoystick(SwerveSubsystem pSwerveSubsystem,
    XboxController pController) {
        mSwerveSubsystem = pSwerveSubsystem;
        mController = pController;
        // this.fieldOrientedFunction = controller.;
        xLimiter = new SlewRateLimiter(Constants.Mechanical.kTeleDriveMaxAccelerationUnitsPerSecond);
        yLimiter = new SlewRateLimiter(Constants.Mechanical.kTeleDriveMaxAccelerationUnitsPerSecond);
        turningLimiter = new SlewRateLimiter(Constants.Mechanical.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(mSwerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Get joystick inputs
        double xSpeed = -mController.getLeftY();
        double ySpeed = -mController.getLeftX();
        double turningSpeed = -mController.getRightX(); 

        // Apply Deadzone
        xSpeed = Math.abs(xSpeed) > Constants.Mechanical.kDeadzone ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.Mechanical.kDeadzone ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > Constants.Mechanical.kDeadzone ? turningSpeed : 0.0;

        // Make Driving Smoother (No Wheelies) Привіт, мене звати Саша, я тут новий кодер
        xSpeed = xLimiter.calculate(xSpeed) * Constants.Mechanical.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.Mechanical.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * Constants.Mechanical.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        SmartDashboard.putNumberArray("Speeds", new Double[]{xSpeed, ySpeed, turningSpeed});
        // Drive
        mSwerveSubsystem.drive(xSpeed, ySpeed, turningSpeed, false);
        
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
