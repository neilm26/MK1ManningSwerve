// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoCommands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Drivetrains.SwerveDrivetrain;

public class AutoTrajectory extends CommandBase {
  /** Creates a new AutoPathPathPlanner. */
  private PathPlannerTrajectory  trajectory;
  private Timer timer = new Timer();

  private boolean hasReachedEndOfTrajectory = false;

  private SwerveDrivetrain drivetrain;
  public AutoTrajectory(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;

    trajectory = PathPlanner.loadPath("TestPath", new PathConstraints(0.5, 0.1));

    Transform2d offset = trajectory.getInitialHolonomicPose().minus(drivetrain.getOdometry().getPoseMeters());

    trajectory.transformBy(offset);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    Pose2d startingPos = trajectory.getInitialHolonomicPose();

    drivetrain.resetModuleHeadings();
    drivetrain.resetOdometry(startingPos);

    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PathPlannerState state = (PathPlannerState) trajectory.sample(timer.get());
    
    drivetrain.updateSkew(false);

    ChassisSpeeds chassisSpeeds = SwerveConstants.SWERVE_PID_CONTROLLER.calculate(
      drivetrain.getOdometry().getPoseMeters(), 
      state, state.holonomicRotation);
    
    SmartDashboard.putNumber("turnspeed", chassisSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("target holo heading:", state.holonomicRotation.getDegrees());
    SmartDashboard.putNumber("drivetrain heading: ", drivetrain.getPigeonRotation2d().getDegrees());

    drivetrain.setCentralMotion(chassisSpeeds, null);

    hasReachedEndOfTrajectory = state.equals(trajectory.getEndState());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.updateSkew(false);
    drivetrain.reface(drivetrain.getPigeonRotation2d().getDegrees(), trajectory.getEndState().holonomicRotation.getDegrees());
    timer.stop();
  }

  // Returns true when the command should end.
  @Override 
  public boolean isFinished() {
    return timer.get()-trajectory.getTotalTimeSeconds()>=.25 && hasReachedEndOfTrajectory;
  }
}
