// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.AutoLib;

import java.util.function.Supplier;

/** Add your docs here. */
public abstract class AutoTerminatedException extends Exception {
    public AutoTerminatedException(Supplier<Boolean> terminated) {
        super("Autonomous has been terminated! " + Boolean.toString(terminated.get()));
    }
}
