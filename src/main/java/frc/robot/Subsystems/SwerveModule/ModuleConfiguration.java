package frc.robot.Subsystems.SwerveModule;

public interface ModuleConfiguration {

    //any swerve version should extend off of SwerveModuleBase which
    //implements ModuleConfig.
    //these methods are 'universal' in the sense that each module type MUST INCLUDE
    //these, but the return/methods contain different motors/encoders.
    double getAbsPosition();
    double getModuleVelocity();
    double getDistanceTravelled();
    void configureSettings();
    void initalize(boolean isReversedDrive, boolean isReversedTurn, int driveId, int turnId, int analogId);
    void easyMotion(double drive, double turn);
    
}
