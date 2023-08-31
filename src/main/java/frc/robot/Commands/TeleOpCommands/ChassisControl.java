// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleOpCommands;

import java.util.Map.Entry;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.SwerveConstants.*;

import frc.robot.Subsystems.Drivetrains.SwerveDrivetrain;
import frc.robot.Subsystems.SwerveModule.SwerveMath;

public class ChassisControl extends CommandBase {
  /** Creates a new ChassisControl. */
  private SwerveDrivetrain drivetrain;
  private Supplier<Double> leftXAxis, leftYAxis, leftTrigger, rightTrigger;
  private Supplier<Boolean> fieldCentric, pivotToggle;

  public ChassisControl(SwerveDrivetrain drivetrain,
      Supplier<Double> leftXAxis,
      Supplier<Double> leftYAxis,
      Supplier<Double> leftTrigger,
      Supplier<Double> rightTrigger,
      Supplier<Boolean> pivotToggle,
      Supplier<Boolean> fieldCentric) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.leftXAxis = leftXAxis;
    this.leftYAxis = leftYAxis;
    this.leftTrigger = leftTrigger;
    this.rightTrigger = rightTrigger;
    this.pivotToggle = pivotToggle;
    this.fieldCentric = fieldCentric;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d pivot = (pivotToggle.get()) ? BACK_LEFT_OFFSET : null;

    if (fieldCentric.get()) {

      //CHASSIS SPEEDS AFTER NOTHER FOOKING CHASSIS SPEEDS
      //MY GOD THIS NEEDS TO BE CHANGED ONCE I FIND THE SOLUTION

      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(-leftXAxis.get(), leftYAxis.get(),
          Math.pow(rightTrigger.get() - leftTrigger.get(), 3));

      ChassisSpeeds fieldCentricSpeeds = SwerveMath.getFieldRelativeChassisSpeeds(chassisSpeeds,
          drivetrain.getPigeonRotation2d());

      //fieldCentricSpeeds are percentage based
      ChassisSpeeds scaledSpeeds = new ChassisSpeeds(fieldCentricSpeeds.vxMetersPerSecond * MAX_SPEED,
          fieldCentricSpeeds.vyMetersPerSecond * MAX_SPEED, fieldCentricSpeeds.omegaRadiansPerSecond * MAX_TURN_SPEED_SCALE);
   
      SmartDashboard.putNumber("distance travelled X: ", drivetrain.getOdometry().getPoseMeters().getX());
      SmartDashboard.putNumber("distance travelled Y: ", drivetrain.getOdometry().getPoseMeters().getY());

      drivetrain.updateSkew(SwerveMath.canBeginSkewCompensation(scaledSpeeds));

      drivetrain.setCentralMotion(scaledSpeeds, pivot);

    } else {
      drivetrain.setCentralMotion(
          new ChassisSpeeds(leftXAxis.get(), leftYAxis.get(), (leftTrigger.get() - rightTrigger.get())), pivot);
    }
    for (Entry<ModuleNames, SwerveModuleState> state : SwerveDrivetrain.stateMap.entrySet()) {
      SmartDashboard.putNumberArray(state.getKey().toString(),
          new Double[] { state.getValue().angle.getDegrees(), state.getValue().speedMetersPerSecond });
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.reface(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
