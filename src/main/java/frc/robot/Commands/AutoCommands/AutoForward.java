// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Drivetrains.SwerveDrivetrain;
import frc.robot.Subsystems.SwerveModule.SwerveMath;

public class AutoForward extends CommandBase {
  /** Creates a new AutoForward. */
  private SwerveDrivetrain drivetrain;
  
  private Timer timer = new Timer();

  public AutoForward(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.getPigeon().setYaw(0);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0.1,0.0);

    if (timer.get() <= 10) {
      drivetrain.setCentralMotion(SwerveMath.getFieldRelativeChassisSpeeds(chassisSpeeds, drivetrain.getPigeonRotation2d()), null);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 10;
  }
}
