// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.AutoLib;

import java.sql.Driver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public abstract class AutoBase {
    protected abstract void coroutine() throws AutoTerminatedException;

    protected abstract void calibrate() throws AutoFailedCalibrationException;

    private boolean isRunning = false;

    protected double getTimerTimeStamp() {
        return Timer.getFPGATimestamp();
    }

    protected void run() {
        isRunning = true;

        try {
            coroutine();
        }
        catch (AutoTerminatedException e) {
            DriverStation.reportWarning("Autonomous has ended pre-emptively!", true);
            return;
        }
        stop();
    }

    protected void resetOdometry() {

    }

    protected void stop() {
        isRunning = false;
    }

    public abstract Pose2d getRobotPosition();

    public boolean checkCorountineStatus() {
        return isRunning;
    }
}
