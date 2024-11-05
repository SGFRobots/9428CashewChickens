package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class Auto extends SequentialCommandGroup{
    // private SwerveSubsystem mSubsystem;
    public Auto(SwerveSubsystem subsystem) {
        // addCommands(null);
        addCommands(new AutoDrive(subsystem, 0, 0, 5, 5));
    }
}
