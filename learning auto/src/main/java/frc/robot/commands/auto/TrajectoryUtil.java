// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TrajectoryUtil extends CommandBase {
  /** Creates a new TrajectoryUtil. */
  public TrajectoryUtil() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

String trajectoryJSON = "paths/YourPath.wpilib.json";
Trajectory trajectory = new Trajectory();
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  }

  private static Trajectory fromPathweaverJson(Path trajectoryPath) {
    return null;
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
