package frc.robot.Subsystems.SwerveModule;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Utilities;
import frc.robot.Subsystems.Drivetrains.SwerveDrivetrain;
import frc.robot.Subsystems.Networking.NetworkTableContainer;

import static frc.robot.Constants.SwerveConstants.*;

public class SwerveModuleHybridMK1 extends SwerveModuleBase {

    private TalonSRXConfiguration turnConfiguration = new TalonSRXConfiguration();

    //should have an interface for motor types as well.
    private CANSparkMax driveMotor;
    private TalonSRX turnMotor;
    private AnalogEncoder analogEncoder;

    @Override
    public double getAbsPosition() {
        return analogEncoder.getAbsolutePosition();
    }

    @Override
    public double getModuleVelocity() {
        return driveMotor.getEncoder(Type.kHallSensor, 42).getVelocity();
    }

    @Override
    public double getDistanceTravelled() {
        return Units.inchesToMeters(driveMotor.getEncoder(Type.kHallSensor, 42).getPosition() * Math.PI * WHEEL_DIAMETER * GEAR_RATIO); }

    @Override
    public void configureSettings() {
        SwerveDrivetrain.preAssignedModules.add(this);

        Utilities.attemptToConfigureThrow(driveMotor.restoreFactoryDefaults(), "cannot factory reset spark max!");
        turnMotor.configFactoryDefault();

        //I call this: the config flood
        //seriously though, ugly.
        driveMotor.setSmartCurrentLimit(100, 20);
        driveMotor.setControlFramePeriodMs(100);

        driveMotor.getPIDController().setP(1e-7);
        driveMotor.getPIDController().setI(1e-11);
        driveMotor.getPIDController().setD(0.01);
        driveMotor.getPIDController().setFF(0.001);
        driveMotor.getPIDController().setIZone(0.009);

        driveMotor.getEncoder(Type.kHallSensor, 42)
            .setVelocityConversionFactor(Units.inchesToMeters(GEAR_RATIO * Math.PI * WHEEL_DIAMETER) / 60);

        turnConfiguration.peakCurrentLimit = 30;
        turnConfiguration.peakCurrentDuration = 250;

        turnConfiguration.feedbackNotContinuous = false;

        Utilities.attemptToConfigure(turnMotor.configAllSettings(turnConfiguration),
                "Cannot calibrate initial turn settings");
    }

    @Override
    public void easyMotion(double drive, double turn) { 
        driveMotor.getPIDController().setReference((drive/MAX_SPEED)*1000, ControlType.kVelocity);
        turnMotor.set(TalonSRXControlMode.PercentOutput, turn);
    }

    public void networkTableDrive() {
        driveMotor.getPIDController().setReference((Double) NetworkTableContainer.entries.get("Auto Forward Power").getNetworkTblValue(), ControlType.kVelocity);
        turnMotor.set(TalonSRXControlMode.PercentOutput,0);
    }

    @Override
    public void initalize(boolean isReversedDrive, boolean isReversedTurn, int driveId, int turnId, int analogEncoderId) {
        driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);
        turnMotor = new TalonSRX(turnId);
        analogEncoder = new AnalogEncoder(analogEncoderId);

        driveMotor.setInverted(isReversedDrive);
        turnMotor.setInverted(isReversedTurn);

        configureSettings();

        driveMotor.burnFlash();
    }
}
