// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer
{

  private final Drivetrain drivetrain;
  private final CommandXboxController controller;

  private final DefaultDrive defaultdrive;

  public RobotContainer()
  {

    drivetrain = new Drivetrain();
    controller = new CommandXboxController(0);

    defaultdrive = new DefaultDrive(drivetrain, controller);

    drivetrain.setDefaultCommand(defaultdrive);

    configureBindings();

  }

  private void configureBindings()
  {}
}