package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoDrive extends Command{
    private SwerveSubsystem mSubsystem;
    private Timer timer;
    private double xSpeed, ySpeed, turningSpeed, time;
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public AutoDrive(SwerveSubsystem subsystem, double xSpeed, double ySpeed, double turningSpeed, double time) {
        mSubsystem = subsystem;
        timer = new Timer();
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.turningSpeed = turningSpeed;
        this.time = time;
        
        // Slew Rate Limiter smooths the robot's accelerations
        xLimiter = new SlewRateLimiter(Constants.Mechanical.kTeleDriveMaxAccelerationUnitsPerSecond);
        yLimiter = new SlewRateLimiter(Constants.Mechanical.kTeleDriveMaxAccelerationUnitsPerSecond);
        turningLimiter = new SlewRateLimiter(Constants.Mechanical.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    }
    
    @Override
    public void initialize() {
        // Make Driving Smoother using Slew Rate Limiter - less jerky by accelerating slowly
        timer.restart();
    }
    
    @Override
    public void execute() {
        System.out.println(turningSpeed);
        double xSpeedNew = xLimiter.calculate(xSpeed);
        double ySpeedNew = yLimiter.calculate(ySpeed);
        double turningSpeedNew = turningLimiter.calculate(turningSpeed);
        System.out.println(turningSpeed);
    
        // Calculate speed in m/s
        xSpeedNew *= Constants.Mechanical.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeedNew *= Constants.Mechanical.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeedNew *= Constants.Mechanical.kTeleDriveMaxSpeedMetersPerSecond;
        System.out.println(turningSpeed);
        // System.out.println(turningSpeed);

        // Set desire chassis speeds based on field or robot relative
        ChassisSpeeds chassisSpeed;
        chassisSpeed = new ChassisSpeeds(xSpeedNew, ySpeedNew, turningSpeedNew);
        chassisSpeed = ChassisSpeeds.discretize(chassisSpeed, 0.02);

        mSubsystem.drive(chassisSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        mSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        if (timer.get() >= time) {
            return true;
        }
        return false;
    }
}
