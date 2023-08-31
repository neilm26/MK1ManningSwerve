// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoCommands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Drivetrains.SwerveDrivetrain;

public class TestTrajJson extends CommandBase {
  /** Creates a new TestTrajJson. */
  private SwerveDrivetrain drivetrain;

  private Trajectory trajectory = new Trajectory();

  private Timer timer = new Timer();

  public TestTrajJson(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      Path path = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/TestPath.wpilib.json");
      trajectory = TrajectoryUtil.fromPathweaverJson(path);

      Pose2d startingPos = trajectory.getInitialPose();

      drivetrain.getPigeon().setYaw(startingPos.getRotation().getDegrees());
      drivetrain.reface(drivetrain.getPigeonRotation2d().getDegrees(), startingPos.getRotation().getDegrees());

      drivetrain.resetModuleHeadings();
      drivetrain.resetOdometry(startingPos);

      timer.restart();
    } catch (IOException e) {
      DriverStation.reportError("cannot load path!", null);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State idealRobotState = trajectory.sample(timer.get());

    drivetrain.updateSkew(false);

    ChassisSpeeds compensatedSpeeds = SwerveConstants.SWERVE_PID_CONTROLLER.calculate(
        drivetrain.getOdometry().getPoseMeters(),
        idealRobotState, idealRobotState.poseMeters.getRotation());

    SmartDashboard.putNumber("turnspeed", compensatedSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("target holo heading:", idealRobotState.poseMeters.getRotation().getDegrees());
    SmartDashboard.putNumber("drivetrain heading: ", drivetrain.getPigeonRotation2d().getDegrees());
    

    drivetrain.setCentralMotion(compensatedSpeeds, null);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.updateSkew(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds() + .25);
  }
}
