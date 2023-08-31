package frc.robot.Subsystems.SwerveModule;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Utilities;
import frc.robot.Constants.SwerveConstants;

public class SwerveMath {
    public static Debouncer debouncer = new Debouncer(0.5, DebounceType.kRising);

    public static Trajectory trajectoryBuilder(Pose2d start, Pose2d end, TrajectoryConfig config, List<Translation2d> waypoints) {
        return TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
    } 

    public static double ticksToAngle(double ticks, double gearRatio) {
        double angle = (ticks % SwerveConstants.TICKS_PER_REV_ANALOG_CODER) / gearRatio;

        double result = (angle / (SwerveConstants.TICKS_PER_REV_ANALOG_CODER / 2)) * 180;

        if (result > 180) {
            result -= 360;
        }

        return result;
    }

    public static double absolutePositionToAngle(double absPos) {
        return absPos * 360;
    }

    public static double[] findFastestTurnDirection(double currAngle, double targetAngle, double speed) {
        double turnAmount = targetAngle - currAngle; // only applies if angles are not clamped between 0-360
        double clampedSpeed = Utilities.constrain(speed, -1, 1);
        if (Math.abs(turnAmount) >= 180) { // if conditional is valid, we inverse control
            // dir = 180 + (-(Math.signum(dir) * 360) + dir); //the inverse of the direction
            // (signum)
            turnAmount = -(Math.signum(turnAmount) * 360) + turnAmount;
            clampedSpeed *= -1; // inverse speed as well so it remains relative to field.
        }
        return new double[] { turnAmount, clampedSpeed };
    }

    public static double[] calculateFastestTurn(double currAngle, double targetAngle, double driveSpeed) {
        double originalHeading = targetAngle - currAngle;
        if (Math.abs(originalHeading) > 90 && Math.abs(originalHeading) < 270) {
            originalHeading = Math.IEEEremainder(originalHeading, 180);
            driveSpeed *= -1;
        }
        return new double[] { originalHeading, driveSpeed };
    }

    public static boolean canBeginSkewCompensation(ChassisSpeeds chassisSpeeds) {
        return debouncer.calculate(chassisSpeeds.omegaRadiansPerSecond==0);
    }

    public static double compensateForSkewAngular(double cycleMs, double currHeading, double targetHeading, PIDController compensationPID) {
        double err = Math.IEEEremainder(Math.toRadians(targetHeading), 2*Math.PI) * cycleMs - 
                Math.IEEEremainder(Math.toRadians(currHeading), 2*Math.PI)  * cycleMs;
    
        double compensate = compensationPID.calculate(err);
    
        double new_vel = compensate / cycleMs;
    
        double radius = Math.sqrt(Math.pow(SwerveConstants.TRACK_WIDTH, 2)+Math.pow(SwerveConstants.WHEEL_BASE, 2));
        return Utilities.notWithin(new_vel / radius, -.03, .03) ? new_vel / radius : 0;
    }


    public static double clamp(double encPos) {
        if (encPos > 0.5) {
            return encPos - 1;
        } else if (encPos < -0.5) {
            return encPos + 1;
        }
        return encPos;
    }

    public static ChassisSpeeds getFieldRelativeChassisSpeeds(ChassisSpeeds robotCentricSpeeds, Rotation2d robotAngle) {
        return new ChassisSpeeds(
            robotCentricSpeeds.vxMetersPerSecond * robotAngle.getCos()
                        - robotCentricSpeeds.vyMetersPerSecond * robotAngle.getSin(),
            robotCentricSpeeds.vyMetersPerSecond * robotAngle.getCos()
                        + robotCentricSpeeds.vxMetersPerSecond * robotAngle.getSin(),
            robotCentricSpeeds.omegaRadiansPerSecond);
    }

    public static ChassisSpeeds getActualRobotSpeeds(SwerveDriveKinematics kinematics, SwerveModuleState ... states) {
        return kinematics.toChassisSpeeds(states);
    }
}
