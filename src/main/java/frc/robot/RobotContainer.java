// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.Map;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AutoCommands.AutoForward;
import frc.robot.Commands.AutoCommands.AutoPath;
import frc.robot.Commands.AutoCommands.AutoTrajectory;
import frc.robot.Commands.AutoCommands.TestTrajJson;
import frc.robot.Commands.TeleOpCommands.ChassisControl;
import frc.robot.Commands.TeleOpCommands.OrbitControl;
import frc.robot.Commands.TeleOpCommands.PointTo;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Networking.NetworkTableContainer;

public class RobotContainer {
  private CommandXboxController driverController = new CommandXboxController(0);
  private GenericHID hid = new GenericHID(0);
  private Drivetrain drivetrain = new Drivetrain();

  private final EventLoop eventLoop = new EventLoop();
  // Commands

  private AutoForward autoForward = new AutoForward(drivetrain);
  private AutoPath autoPath = new AutoPath(drivetrain);
  private AutoTrajectory autoPathPathPlanner = new AutoTrajectory(drivetrain);
  private TestTrajJson testTrajJson = new TestTrajJson(drivetrain);

  private SendableChooser<Command> autoPathChooser = new SendableChooser<>();
  private SendableChooser<String> autoPathCustomizer = new SendableChooser<>();
  private SendableChooser<Map<String, Double>> autoPathConfig = new SendableChooser<>();
  
  private ChassisControl chassisControl = new ChassisControl(drivetrain,
      driverController::getLeftY,
      driverController::getLeftX,
      () -> driverController.getRawAxis(3),
      () -> driverController.getRawAxis(4), 
      () -> driverController.x().getAsBoolean(),
      () -> true);

  private OrbitControl orbitControl = new OrbitControl(drivetrain,
      driverController::getRightX,
      driverController::getRightTriggerAxis);
  
  private PointTo pointControl = new PointTo(
    () -> driverController.getRawAxis(2),
    () -> driverController.getRawAxis(5),
    drivetrain);

  public RobotContainer() {
    NetworkTableContainer.insertGlobalEntries();
    setUpPathOptions();
    configureBindings();
  }


  public void setUpPathOptions() {
    autoPathChooser.addOption("TestTurn", autoPath.autoCommand.andThen(
      () -> drivetrain.setCentralMotion(new ChassisSpeeds(0, 0, 
      0), null)).alongWith(new InstantCommand(() ->drivetrain.updateSkew(false))));

    autoPathChooser.setDefaultOption("Auto Drive Timed", autoForward);

    autoPathChooser.addOption("Turnpath", autoPathPathPlanner);

    autoPathChooser.addOption("testTraj", testTrajJson);

    SmartDashboard.putData(autoPathChooser);
  }

  public void configureZeroHeading() {
    drivetrain.reface(0);
    drivetrain.getPigeon().setYaw(0);
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(chassisControl);

    BooleanEvent pointToEvent = new BooleanEvent(eventLoop,
        () -> (hid.getPOV() != -1));

    Trigger pointToTrigger = pointToEvent.castTo(Trigger::new);

    driverController.leftBumper()
        .whileTrue(pointControl);

    driverController.a().onTrue(new InstantCommand(() ->  configureZeroHeading(), drivetrain));
  }

  public Command getAutonomousCommand() {
    return autoPathChooser.getSelected();
  }

  public void pollEvent() {
    eventLoop.poll();
  }
}
