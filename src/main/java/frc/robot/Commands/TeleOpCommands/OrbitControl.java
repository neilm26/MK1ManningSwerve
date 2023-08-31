// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleOpCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Drivetrains.SwerveDrivetrain;
import frc.robot.Subsystems.SwerveModule.SwerveMath;

public class OrbitControl extends CommandBase {
  /** Creates a new OrbitControl. */
  private Supplier<Double> angularVelocity, distanceFromRobotCenterToOrbitCenter;
  private SwerveDrivetrain drivetrain;
  public OrbitControl(SwerveDrivetrain drivetrain, Supplier<Double> angularVelocity, Supplier<Double> distanceFromRobotCenterToOrbitCenter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angularVelocity = angularVelocity;
    this.distanceFromRobotCenterToOrbitCenter = distanceFromRobotCenterToOrbitCenter;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /*circular motion can be best defined through polar coordinates.
      Given an angular velocity and radius, we can calculate the period it takes to complete one full revolution
      of 2 * pi * r. 
      To find the velocity (of equal magnitude at all points on an orbit), use the r * angVelocity.
      This velocity will always remain constant (however, due to tangency, direction always changes).
      In order to apply this understanding, execute() should always handle the error compensation for the speeds tangent
      to the circle, using setDesiredState().
    */



    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 
            -angularVelocity.get()*distanceFromRobotCenterToOrbitCenter.get(), 2*Math.PI*angularVelocity.get());

    drivetrain.setCentralMotion(
      SwerveMath.getFieldRelativeChassisSpeeds(
        chassisSpeeds,
        drivetrain.getPigeonRotation2d()), null);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
