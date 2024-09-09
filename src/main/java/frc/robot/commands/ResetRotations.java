package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetRotations extends Command {
    SwerveSubsystem mSubsystem;

    public ResetRotations(SwerveSubsystem pSubsystem) {
        // Swerve Subsystem
        mSubsystem = pSubsystem;
    }

    @Override
    public void initialize() {
        // Start resetting process
        mSubsystem.resetEncoders();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean isFinished) {}

    @Override
    public boolean isFinished() {
        // Check if all modules are done resetting
        return mSubsystem.checkEncoderResetted();
    }
    
}
