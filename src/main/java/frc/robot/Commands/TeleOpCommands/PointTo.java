// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleOpCommands;

import java.util.Map.Entry;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants.ModuleNames;
import frc.robot.Subsystems.Drivetrains.SwerveDrivetrain;

public class PointTo extends CommandBase {
  /** Creates a new PointTo. */
  private SwerveDrivetrain drivetrain;
  private Supplier<Double> rightXAxis, rightYAxis;

  public PointTo(Supplier<Double> rightXAxis,
                Supplier<Double> rightYAxis,
                SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.rightXAxis = rightXAxis;
    this.rightYAxis = rightYAxis;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    drivetrain.setCentralMotion(new ChassisSpeeds(-rightXAxis.get(), -rightYAxis.get(), 0), null);
    
    for (Entry<ModuleNames, SwerveModuleState> state: SwerveDrivetrain.stateMap.entrySet()) {
      SmartDashboard.putNumberArray(state.getKey().toString(), new Double[] {state.getValue().angle.getDegrees(), state.getValue().speedMetersPerSecond});
    }
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
