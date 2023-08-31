// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrains;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveConstants.ModuleNames;
import frc.robot.Subsystems.SwerveModule.SwerveMath;
import frc.robot.Subsystems.SwerveModule.SwerveModuleBase;
import frc.robot.Subsystems.SwerveModule.SwerveModuleFactory;

import static frc.robot.Constants.SensorConstants.*;
import static frc.robot.Constants.SwerveConstants.*;

public abstract class SwerveDrivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  protected static SwerveModuleBase frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule,
      backRightSwerveModule;

  public static List<SwerveModuleBase> preAssignedModules = new ArrayList<SwerveModuleBase>();

  public static Map<ModuleNames, SwerveModuleState> stateMap = new HashMap<>();
  public static Map<ModuleNames, SwerveModulePosition> moduleWheelPos = new HashMap<ModuleNames, SwerveModulePosition>();

  private SwerveDriveKinematics driveKinematics;
  private SwerveDriveOdometry driveOdometry;

  private ChassisSpeeds chassisSpeeds;

  private WPI_Pigeon2 pigeon2;

  private static Field2d field = new Field2d();

  private double targetHeading;
  private double currentHeading;

  private boolean isSkewing = false;

  protected void initialize() {
    pigeon2 = new WPI_Pigeon2(PIGEON_ID);

    SmartDashboard.putData(field);

    field.setRobotPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    SmartDashboard.putData("angularPID", MODULE_HEADING_PID_CONTROLLER);
    SmartDashboard.putData("drivePID", DRIVE_PID_CONTROLLER);
    SmartDashboard.putData("compensationPID", COMPENSATION_PID_CONTROLLER);
    SmartDashboard.putData("holonomicPID", SWERVE_PID_CONTROLLER.getThetaController());
    initializeAllModules();
  }

  private void initializeAllModules() {
    try {
      backLeftSwerveModule = SwerveModuleFactory.createHybridMk1Mode(ModuleNames.BACK_LEFT,
          0.7666666666666667, false, false,
          5, 4, 
          2);

      frontRightSwerveModule = SwerveModuleFactory.createHybridMk1Mode(ModuleNames.FRONT_RIGHT,
          0.8291666666666667, false, false,
          8, 1, 1);

      frontLeftSwerveModule = SwerveModuleFactory.createHybridMk1Mode(ModuleNames.FRONT_LEFT,
          0.1611111111111111, false, false,
          7, 3, 0);

      backRightSwerveModule = SwerveModuleFactory.createHybridMk1Mode(ModuleNames.BACK_RIGHT,
          0.8611111111111111, false, false,
          9, 6, 3);

      driveKinematics = new SwerveDriveKinematics(OFFSET_ARRAY);
      driveOdometry = new SwerveDriveOdometry(driveKinematics,
          getPigeonRotation2d(), getModulePositionsNoKey());

    } catch (NullPointerException e) {
      DriverStation.reportError("Cannot initialize modules! Please verify that module name(s) exist!", true);
    }
  }

  public SwerveDriveKinematics getKinematics() {
    return driveKinematics;
  }

  public SwerveDriveOdometry getOdometry() {
    return driveOdometry;
  }

  public double getAccelerometerRateX() {
    return pigeon2.getRate();
  }

  public Field2d getField() {
    return field;
  }

  private SwerveModulePosition[] getModulePositionsNoKey() {
    SwerveModulePosition[] wheelPosNoKey = moduleWheelPos.values()
        .toArray(new SwerveModulePosition[moduleWheelPos.size()]);

    return wheelPosNoKey;
  }

  public List<SwerveModuleBase> getModules() {
    return preAssignedModules;
  }

  public SwerveModuleBase getModule(ModuleNames name) {
    switch (name) {

      case FRONT_LEFT:
        return frontLeftSwerveModule;
      case FRONT_RIGHT:
        return frontRightSwerveModule;
      case BACK_LEFT:
        return backLeftSwerveModule;
      case BACK_RIGHT:
        return backRightSwerveModule;

      default:
        break;
    }
    return null;
  }

  public Pigeon2 getPigeon() {
    return pigeon2;
  }

  public Rotation2d getPigeonRotation2d() {
    return pigeon2.getRotation2d();
  }

  public SwerveModuleState getModuleState(ModuleNames moduleName) {
    return stateMap.getOrDefault(moduleName, null);
  }

  public ChassisSpeeds getRobotMotion() {
    return driveKinematics.toChassisSpeeds(stateMap.values().toArray(new SwerveModuleState[stateMap.size()]));
  }

  public void resetOdometry(Pose2d startPos) {
    driveOdometry.resetPosition(getPigeonRotation2d(), getModulePositionsNoKey(), startPos);
  }

  public void resetModuleHeadings() {
    for (SwerveModuleBase swerveModule : getModules()) {
      swerveModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(180)));
      swerveModule.updateEntries();
    }
  }

  public void updateSkew(boolean isSkewing) {
    this.isSkewing = isSkewing;
  }

  public void setStates(SwerveModuleState[] states) {
    for (int i = 0; i < states.length; i++) {
      getModules().get(i).setDesiredState(states[i]);
    }
  }

  public void setCentralMotion(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation) {
    this.chassisSpeeds = chassisSpeeds;

    currentHeading = getPigeon().getYaw();

    SmartDashboard.putBoolean("isSkewing", isSkewing);
    SmartDashboard.putNumber("current heading", currentHeading);
    SmartDashboard.putNumber("target heading", targetHeading);

    if (isSkewing) {
      double correction = SwerveMath.compensateForSkewAngular(TIMEOUT_MS, currentHeading, targetHeading,
          COMPENSATION_PID_CONTROLLER);
      this.chassisSpeeds = new ChassisSpeeds(
          chassisSpeeds.vxMetersPerSecond,
          chassisSpeeds.vyMetersPerSecond,
          chassisSpeeds.omegaRadiansPerSecond + correction);
    } else {
      targetHeading = currentHeading;
    }

    SwerveModuleState[] states = (centerOfRotation != null)
        ? driveKinematics.toSwerveModuleStates(this.chassisSpeeds, centerOfRotation)
        : driveKinematics.toSwerveModuleStates(this.chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED);

    stateMap = IntStream.range(0, ModuleNames.values().length).boxed().collect(Collectors.toMap(
        i -> ModuleNames.values()[i],
        i -> states[i]));
    for (SwerveModuleBase module : getModules()) {
      module.setDesiredState(getModuleState(module.getModuleName()));
    }
  }

  public void reface(double override) {
    targetHeading = currentHeading = override;
  }

  public void reface(double currentHeadingOverride, double targetHeadingOverride) {
    currentHeading = currentHeadingOverride;
    targetHeading = targetHeadingOverride;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    driveOdometry.update(getPigeonRotation2d(), getModulePositionsNoKey());

    for (SwerveModuleBase module : getModules()) {
      // try and update each module's state unless it cannot be found, then return an
      // empty module state.
      module.updateDrivePIDs(DRIVE_PID_CONTROLLER, MODULE_HEADING_PID_CONTROLLER);
      module.updateEntries();
    }
  }
}
