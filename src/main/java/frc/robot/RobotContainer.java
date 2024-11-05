// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Auto;

// Subsystems and commands
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.ResetRotations;


public class RobotContainer {

  // Instances of controllers
  // public final XboxController mController;
  public final GenericHID mController;

  // Subsystems and commands
  private final SwerveSubsystem mSwerveSubsystem;
  private final ResetRotations mResetRotations;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // mController = new XboxController(0);
    mController = new GenericHID(0);

    // Subsystems and commands
    mSwerveSubsystem = new SwerveSubsystem();
    mSwerveSubsystem.setDefaultCommand(new SwerveJoystick(mSwerveSubsystem, mController));
    mResetRotations = new ResetRotations(mSwerveSubsystem);

    // Bind buttons and commands
    configureButtonBindings();
  }

  // Assign buttons to commands
  private void configureButtonBindings() {
    new JoystickButton(mController, Constants.Controllers.ButtonAPort).onTrue(mResetRotations);
  }


  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Auto(mSwerveSubsystem);
  }
}
