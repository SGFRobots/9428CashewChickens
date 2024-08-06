// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.Auto;
// Subsystems and commands
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.SwerveJoystick;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  

  // Instances of controllers
  public final XboxController mController;

  // Swerve subsystem
  private final SwerveSubsystem mSwerveSubsystem;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    mController = new XboxController(0);

    // Subsystems and commands
    mSwerveSubsystem = new SwerveSubsystem();
    mSwerveSubsystem.setDefaultCommand(new SwerveJoystick(mSwerveSubsystem, mController));


    // Bind buttons and commands
    configureButtonBindings();
  }

  // Assign buttons to commands
  private void configureButtonBindings() {
    new JoystickButton(mController, 0);
  }


  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Auto();
  }
}
