package frc.robot.Constants;


import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveConstants {
    public static final int MODULE_COUNT = 4;
    public static final String CANIVORENAME = "canivore1";

    public static final double WHEEL_BASE = 0.5207; //cm to m, from one module adjacent to another.
    public static final double TRACK_WIDTH = 0.5207;

    public static final double WHEEL_DIAMETER = 3.36; // (inches)
    public static final double TICKS_PER_REV_CIM_CODER = 1024;
    public static final double TICKS_PER_REV_ANALOG_CODER = 4096;


    public static final double HOME_ANALOG_ENC_POS_FRONT_RIGHT = -0.06944444444;
    public static final double HOME_ANALOG_ENC_POS_FRONT_LEFT = 0.1111111; //offsets necessary to zero out encoders
    public static final double HOME_ANALOG_ENC_POS_BACK_LEFT = -0.07777777777;
    public static final double HOME_ANALOG_ENC_POS_BACK_RIGHT = -0.0388888888888889;

    public static final double MAX_SPEED = 3.30708/3; //10.85 ft/sec to m/s
    public static final double MAX_ACCEL = 3.30708/3;

    public static final double MAX_DRIVE_RPM = 5700;

    public static final double MAX_POINT_SPEED = 0.25;
    public static final double MAX_TURN_SPEED_SCALE = 0.25;

    public static final double TIMEOUT_MS = 5;

    public static final double GEAR_RATIO = 1/4.94;

    //Driving encoder gear of module MK1 is 48 teeth.

 

    public static final Translation2d FRONT_RIGHT_OFFSET = new Translation2d(WHEEL_BASE/2, TRACK_WIDTH/2);
    public static final Translation2d FRONT_LEFT_OFFSET = new Translation2d(WHEEL_BASE/2, -TRACK_WIDTH/2);
    public static final Translation2d BACK_RIGHT_OFFSET = new Translation2d(-WHEEL_BASE/2, -TRACK_WIDTH/2);
    public static final Translation2d BACK_LEFT_OFFSET = new Translation2d(-WHEEL_BASE/2, TRACK_WIDTH/2);


    public static final Translation2d[] OFFSET_ARRAY = new Translation2d[] {FRONT_LEFT_OFFSET, FRONT_RIGHT_OFFSET, BACK_LEFT_OFFSET, BACK_RIGHT_OFFSET};

    public static PIDController DRIVE_PID_CONTROLLER = new PIDController(1.5, 0.009, 0.02);
    public static ProfiledPIDController ANGULAR_PID_CONTROLLER = new ProfiledPIDController(0.087, 0, 0, new TrapezoidProfile.Constraints(MAX_SPEED, MAX_ACCEL));

    public static PIDController COMPENSATION_PID_CONTROLLER = new PIDController(1, 0.04, 0.06);
    public static ProfiledPIDController MODULE_HEADING_PID_CONTROLLER = new ProfiledPIDController(5, 0, 0, 
                            new TrapezoidProfile.Constraints(MAX_SPEED, MAX_ACCEL));

    
    public static SimpleMotorFeedforward TURN_FEEDFORWARD = new SimpleMotorFeedforward(0.06, 0.02);
    public static SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(1, 0.5);

    public static HolonomicDriveController SWERVE_PID_CONTROLLER = new HolonomicDriveController(DRIVE_PID_CONTROLLER, 
                                                                    DRIVE_PID_CONTROLLER, 
                                                                    ANGULAR_PID_CONTROLLER);    


    public static enum ModuleNames {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_RIGHT,
        BACK_LEFT
    }    
}