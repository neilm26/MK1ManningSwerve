package frc.robot.Subsystems.SwerveModule;

import frc.robot.Constants.SwerveConstants.ModuleNames;

public class SwerveModuleFactory {

    public static SwerveModuleBase createNeoMk4invertedModule(ModuleNames moduleName, boolean isReversedDrive, 
                    boolean isReversedTurn, int canCoderIndex, int driveId, int turnId) {return null;}

    public static SwerveModuleBase createHybridMk1Mode(ModuleNames moduleName, double encoderAbsOffset,
                                                boolean isReversedDrive, boolean isReversedTurn, int driveId, int turnId, int analogId) {
        SwerveModuleHybridMK1 hybridMK1 = new SwerveModuleHybridMK1();
        
        hybridMK1.initalize(isReversedDrive, isReversedTurn, driveId, turnId, analogId);
        hybridMK1.calibrate(moduleName, ()->encoderAbsOffset);
        hybridMK1.configureSettings();
    
        return hybridMK1;
    }
}
