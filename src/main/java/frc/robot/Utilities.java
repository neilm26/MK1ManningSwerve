package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.SensorConstants;

public class Utilities {
    public static void attemptToConfigure(ErrorCode errorCode, String errorMsg) {
        if (errorCode != ErrorCode.OK) {
            DriverStation.reportError(errorMsg + " " + errorCode, false);
        }
    }

    public static void attemptToConfigureThrow(ErrorCode errorCode, String errorMsg) {
        if (errorCode != ErrorCode.OK) {
            throw new RuntimeException(errorMsg + " " + errorCode);
        }
    }

	public static void attemptToConfigureThrow(REVLibError errorCode, String errorMsg) {
		if (errorCode != REVLibError.kOk) {
			throw new RuntimeException(errorMsg + " " + errorCode);
		}
	}

    public static Object convertAxesToDegrees(double xAxis, double yAxis) {
        if (Math.abs(xAxis) < SensorConstants.JOYSTICK_DEAD_ZONE && 
            Math.abs(yAxis) < SensorConstants.JOYSTICK_DEAD_ZONE) {
            return null;
        }
        double angle = Math.toDegrees(Math.atan2(yAxis, xAxis))*-1;

        return angle = angle < 0 ? angle+=360 : angle;
    }
    
    public static double constrain(double value, double min, double max) {
		if(value < min) {
			return min;
		}else if(value > max) {
			return max;
		}


		return value;
	}

	public static boolean allWithinConstraints(double min, double max, double... values) {
		for (double value: values) {
			if (value < min || value > max) {
				return false;
			}
		}
		return true;
	}
	
	public static double constrain(double value, double max) {
		if(value < 0) {
			return 0;
		}else if(value > max) {
			return max;
		}
		return value;
	}
	
	public static double withinValue(double value, double min, double max) {
		if(value < max && value > min) {
			return value;
		}else {
			return 0;
		}
	}
	
	public static double notWithinValue(double value, double min, double max) {
		if(value < max && value > min) {
			return 0;
		}else {
			return value;
		}
	}
	
	public static boolean within(double value, double min, double max) {
		return (value < max && value > min);
	}
	
	public static boolean notWithin(double value, double min, double max) {
		return (value > max || value < min);
	}
	
}