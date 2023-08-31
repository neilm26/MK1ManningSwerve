// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoCommands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Subsystems.Drivetrains.SwerveDrivetrain;

import static frc.robot.Constants.SwerveConstants.*;

public class AutoPath {
  /** Creates a new AutoPath. */
  private SwerveDrivetrain drivetrain;
  private PathPlannerTrajectory trajectory = PathPlanner.loadPath("TestPath", new PathConstraints(0.4, 0.12));
  
  public SwerveControllerCommand autoCommand;

  public AutoPath(SwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    autoCommand =  new SwerveControllerCommand(
      trajectory, 
      () -> drivetrain.getOdometry().getPoseMeters(), 
      drivetrain.getKinematics(), 
      SWERVE_PID_CONTROLLER, 
      drivetrain::setStates, drivetrain);
  }
}
