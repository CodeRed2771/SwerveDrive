package frc.robot;

import java.math.BigDecimal;
import java.math.RoundingMode;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain implements PIDOutput {

	private static DriveTrain instance;
	public static Module moduleA, moduleB, moduleC, moduleD;
	private static PIDController pidControllerRot;
	private static boolean inPIDRotationMode = true;

	public static DriveTrain getInstance() {
		if (instance == null) {
			try {
				instance = new DriveTrain();
			} catch (Exception ex) {
				System.out.println("Drive train could not be initialized due to the following error: ");
				System.out.println(ex.getMessage());
				System.out.println(ex.getStackTrace());
				return null;
			}
		}
		return instance;
	}

	// define robot dimensions. L=wheel base W=track width
	private static final double l = 22, w = 21, r = Math.sqrt((l * l) + (w * w));

	private DriveTrain() {

		moduleA = new Module(Calibration.DT_A_DRIVE_TALON_ID, Calibration.DT_A_TURN_TALON_ID, Calibration.AUTO_DRIVE_P,
				Calibration.AUTO_DRIVE_I, Calibration.AUTO_DRIVE_D, Calibration.AUTO_DRIVE_IZONE, Calibration.TURN_P,
				Calibration.TURN_I, Calibration.TURN_D, 200, Calibration.GET_DT_A_ABS_ZERO(),'A'); // Front right
		moduleB = new Module(Calibration.DT_B_DRIVE_TALON_ID, Calibration.DT_B_TURN_TALON_ID, Calibration.AUTO_DRIVE_P,
				Calibration.AUTO_DRIVE_I, Calibration.AUTO_DRIVE_D, Calibration.AUTO_DRIVE_IZONE, Calibration.TURN_P,
				Calibration.TURN_I, Calibration.TURN_D, 200, Calibration.GET_DT_B_ABS_ZERO(),'B'); // Back left
		moduleC = new Module(Calibration.DT_C_DRIVE_TALON_ID, Calibration.DT_C_TURN_TALON_ID, Calibration.AUTO_DRIVE_P,
				Calibration.AUTO_DRIVE_I, Calibration.AUTO_DRIVE_D, Calibration.AUTO_DRIVE_IZONE, Calibration.TURN_P,
				Calibration.TURN_I, Calibration.TURN_D, 200, Calibration.GET_DT_C_ABS_ZERO(),'C'); // Back right
		moduleD = new Module(Calibration.DT_D_DRIVE_TALON_ID, Calibration.DT_D_TURN_TALON_ID, Calibration.AUTO_DRIVE_P,
				Calibration.AUTO_DRIVE_I, Calibration.AUTO_DRIVE_D, Calibration.AUTO_DRIVE_IZONE, Calibration.TURN_P,
				Calibration.TURN_I, Calibration.TURN_D, 200, Calibration.GET_DT_D_ABS_ZERO(),'D'); // Front left

		// PID is for PID drive not for the modules
		// DVV - I don't believe we're using a mode that uses this
		pidControllerRot = new PIDController(Calibration.DT_ROT_PID_P, Calibration.DT_ROT_PID_I,
				Calibration.DT_ROT_PID_D, RobotGyro.getGyro(), this);
		pidControllerRot.setInputRange(-180.0f, 180.0f);
		pidControllerRot.setOutputRange(-1.0, 1.0);
		pidControllerRot.setContinuous(true);
	}

	public static void setFollowerFix(boolean enable) {
		if (enable)
			moduleB.setFollower(Calibration.DT_D_DRIVE_TALON_ID);
		else
			moduleB.setFollower(0);
	}

	public static void setDrivePower(double modAPower, double modBPower, double modCPower, double modDPower) {
		if (getInstance() == null)
			return;

		moduleA.setDrivePower(modAPower);
		moduleB.setDrivePower(modBPower);
		moduleC.setDrivePower(modCPower);
		moduleD.setDrivePower(modDPower);
	}

	public static void setDriveMMAccel(int accel) {
		moduleA.setDriveMMAccel(accel);
		moduleB.setDriveMMAccel(accel);
		moduleC.setDriveMMAccel(accel);
		moduleD.setDriveMMAccel(accel);
	}

	public static void setDriveMMVelocity(int velocity) {
		moduleA.setDriveMMVelocity(velocity);
		moduleB.setDriveMMVelocity(velocity);
		moduleC.setDriveMMVelocity(velocity);
		moduleD.setDriveMMVelocity(velocity);
	}

	public static int getDriveVelocity() {
		return (moduleA.getDriveVelocity() + moduleB.getDriveVelocity() + moduleC.getDriveVelocity()
				+ moduleD.getDriveVelocity()) / 4;
	}

	public static boolean hasDriveCompleted(int allowedError) {
		// just checking two of the modules to see if they are done moving
		return moduleB.hasDriveCompleted(allowedError) && moduleA.hasDriveCompleted(allowedError);
	}

	public static boolean hasDriveCompleted() {
		return hasDriveCompleted(0);
	}

	public static void setTurnPower(double modAPower, double modBPower, double modCPower, double modDPower) {
		if (getInstance() == null)
			return;

		moduleA.setTurnPower(modAPower);
		moduleB.setTurnPower(modBPower);
		moduleC.setTurnPower(modCPower);
		moduleD.setTurnPower(modDPower);
	}

	public static void setTurnOrientation(double modAPosition, double modBPosition, double modCPosition,
			double modDPosition) {
		setTurnOrientation(modAPosition, modBPosition, modCPosition, modDPosition, true);
	}

	public static void setTurnOrientation(double modAPosition, double modBPosition, double modCPosition,
			double modDPosition, boolean optimizeTurn) {
		if (getInstance() == null)
			return;

		// position is a value from 0 to 1 that indicates
		// where in the rotation of the module the wheel should be set.
		// e.g. a value of .5 indicates a half turn from the zero position

		moduleA.setTurnOrientation(modAPosition, optimizeTurn);
		moduleB.setTurnOrientation(modBPosition, optimizeTurn);
		moduleC.setTurnOrientation(modCPosition, optimizeTurn);
		moduleD.setTurnOrientation(modDPosition, optimizeTurn);

	}

	public static void setAllTurnOrientation(double position) {
		if (getInstance() == null)
			return;
		setTurnOrientation(position, position, position, position, true);
	}

	/**
	 * @param position     - position, 0 to 1, to turn to.
	 * @param optimizeTurn - allow turn optimization
	 */
	public static void setAllTurnOrientation(double position, boolean optimizeTurn) {
		if (getInstance() == null)
			return;
		setTurnOrientation(position, position, position, position, optimizeTurn);
	}

	public static void setAllDrivePosition(int position) {
		if (getInstance() == null)
			return;
		setDrivePosition(position, position, position, position);
	}

	public static void setDrivePosition(int modAPosition, int modBPosition, int modCPosition, int modDPosition) {
		if (getInstance() == null)
			return;

		SmartDashboard.putNumber("Drive A Setpoint", modAPosition);
		SmartDashboard.putNumber("Drive B Setpoint", modBPosition);
		SmartDashboard.putNumber("Drive C Setpoint", modCPosition);
		SmartDashboard.putNumber("Drive D Setpoint", modDPosition);

		moduleA.setDrivePIDToSetPoint(modAPosition);
		moduleB.setDrivePIDToSetPoint(modBPosition);
		moduleC.setDrivePIDToSetPoint(modCPosition);
		moduleD.setDrivePIDToSetPoint(modDPosition);

	}

	public static void addToAllDrivePositions(int ticks) {
		if (getInstance() == null)
			return;

		setDrivePosition(moduleA.getDriveEncoder() + ((moduleA.getIsModuleReversed() ? -1 : 1) * ticks),
				moduleB.getDriveEncoder() + ((moduleB.getIsModuleReversed() ? -1 : 1) * ticks),
				moduleC.getDriveEncoder() + ((moduleC.getIsModuleReversed() ? -1 : 1) * ticks),
				moduleD.getDriveEncoder() + ((moduleD.getIsModuleReversed() ? -1 : 1) * ticks));
	}

	public static int getDriveEnc() {
		if (getInstance() == null)
			return 0;
		return (moduleA.getDriveEncoder() + moduleB.getDriveEncoder() + moduleC.getDriveEncoder() + moduleD.getDriveEncoder()) / 4;
	}

	public static void autoSetRot(double rot) {
		if (getInstance() == null)
			return;
		swerveDrive(0, 0, rot);
	}

	public static void setAllTurnPower(double power) {
		if (getInstance() == null)
			return;
		setTurnPower(power, power, power, power);
	}

	public static void setAllDrivePower(double power) {
		if (getInstance() == null)
			return;
		setDrivePower(power, power, power, power);
	}

	public static boolean isModuleATurnEncConnected() {
		if (getInstance() == null)
			return false;
		return moduleA.isTurnEncConnected();
	}

	public static boolean isModuleBTurnEncConnected() {
		if (getInstance() == null)
			return false;
		return moduleB.isTurnEncConnected();
	}

	public static boolean isModuleCTurnEncConnected() {
		if (getInstance() == null)
			return false;
		return moduleC.isTurnEncConnected();
	}

	public static boolean isModuleDTurnEncConnected() {
		if (getInstance() == null)
			return false;
		return moduleD.isTurnEncConnected();
	}

	public static void resetDriveEncoders() {
		if (getInstance() == null)
			return;

		moduleA.resetDriveEncoder();
		moduleB.resetDriveEncoder();
		moduleC.resetDriveEncoder();
		moduleD.resetDriveEncoder();
	}

	public static void stopDriveAndTurnMotors() {
		if (getInstance() == null)
			return;

		moduleA.stopDriveAndTurnMotors();
		moduleB.stopDriveAndTurnMotors();
		moduleC.stopDriveAndTurnMotors();
		moduleD.stopDriveAndTurnMotors();
	}

	public static void stopDrive() {
		if (getInstance() == null)
			return;

		moduleA.stopDrive();
		moduleB.stopDrive();
		moduleC.stopDrive();
		moduleD.stopDrive();
	}

	public static double angleToPosition(double angle) {
		if (angle < 0) {
			return .5d + ((180d - Math.abs(angle)) / 360d);
		} else {
			return angle / 360d;
		}
	}

	private static boolean allowTurnEncoderReset = false;

	public static void allowTurnEncoderReset() {
		allowTurnEncoderReset = true;
	}

	/*
	 * Resets the turn encoder values relative to what we've determined to be the
	 * "zero" position. 
	 * 
	 */
	public static void resetTurnEncoders() {
		if (getInstance() == null)
			return;

		// if (allowTurnEncoderReset) {
			double modAOff = 0, modBOff = 0, modCOff = 0, modDOff = 0;

			moduleA.setTurnPower(0);
			moduleC.setTurnPower(0);
			moduleB.setTurnPower(0);
			moduleD.setTurnPower(0);

			Timer.delay(1);

			// first find the current absolute position of the turn encoders
			modAOff = DriveTrain.moduleA.getTurnAbsolutePosition();
			modBOff = DriveTrain.moduleB.getTurnAbsolutePosition();
			modCOff = DriveTrain.moduleC.getTurnAbsolutePosition();
			modDOff = DriveTrain.moduleD.getTurnAbsolutePosition();

			// now use the difference between the current position and the
			// calibration zero
			// position
			// to tell the encoder what the current relative position is
			// (relative to the
			// zero pos)
			moduleA.setTurnEncoderValue((int) (calculatePositionDifference(modAOff, Calibration.GET_DT_A_ABS_ZERO()) * 4096d));
			moduleB.setTurnEncoderValue((int) (calculatePositionDifference(modBOff, Calibration.GET_DT_B_ABS_ZERO()) * 4096d));
			moduleC.setTurnEncoderValue((int) (calculatePositionDifference(modCOff, Calibration.GET_DT_C_ABS_ZERO()) * 4096d));
			moduleD.setTurnEncoderValue((int) (calculatePositionDifference(modDOff, Calibration.GET_DT_D_ABS_ZERO()) * 4096d));

			
			System.out.println("Turn encoders have been reset");

		// 	allowTurnEncoderReset = false;
		// }
	}

	private static double calculatePositionDifference(double currentPosition, double calibrationZeroPosition) {
		if (currentPosition - calibrationZeroPosition >= 0) {
			return currentPosition - calibrationZeroPosition;
		} else {
			return (1 - calibrationZeroPosition) + currentPosition;
		}
	}

	public static void setDriveBrakeMode(boolean b) {
		if (getInstance() == null)
			return;

		moduleA.setBrakeMode(b);
		moduleB.setBrakeMode(b);
		moduleC.setBrakeMode(b);
		moduleD.setBrakeMode(b);
	}

	public static double getAverageTurnError() {
		if (getInstance() == null)
			return 0.0;

		return (Math.abs(moduleA.getTurnError()) + Math.abs(moduleB.getTurnError()) + Math.abs(moduleC.getTurnError())
				+ Math.abs(moduleD.getTurnError())) / 4d;
	}

	public static double getAverageDriveError() {
		if (getInstance() == null)
			return 0.0;

		return (Math.abs(moduleA.getDriveError()) + Math.abs(moduleB.getDriveError())
				+ Math.abs(moduleC.getDriveError()) + Math.abs(moduleD.getDriveError())) / 4d;
	}

	public static double getDriveError() {
		if (getInstance() == null)
			return 0.0;
		return moduleA.getDriveError();
	}

	/*
	 * 
	 * Drive methods
	 */
	public static void swerveDrive(double fwd, double strafe, double rot) {
		if (getInstance() == null)
			return;

		double a = strafe - (rot * (l / r));
		double b = strafe + (rot * (l / r));
		double c = fwd - (rot * (w / r));
		double d = fwd + (rot * (w / r));

		double ws1 = Math.sqrt((b * b) + (c * c)); // front_right (CHECK THESE
													// AGAINST OUR BOT)
		double ws2 = Math.sqrt((b * b) + (d * d)); // front_left
		double ws3 = Math.sqrt((a * a) + (d * d)); // rear_left
		double ws4 = Math.sqrt((a * a) + (c * c)); // rear_right

		double wa1 = Math.atan2(b, c) * 180 / Math.PI;
		double wa2 = Math.atan2(b, d) * 180 / Math.PI;
		double wa3 = Math.atan2(a, d) * 180 / Math.PI;
		double wa4 = Math.atan2(a, c) * 180 / Math.PI;

		double max = ws1;
		max = Math.max(max, ws2);
		max = Math.max(max, ws3);
		max = Math.max(max, ws4);
		if (max > 1) {
			ws1 /= max;
			ws2 /= max;
			ws3 /= max;
			ws4 /= max;
		}

		// SmartDashboard.putNumber("swerve a", a);
		// SmartDashboard.putNumber("swerve b", b);
		// SmartDashboard.putNumber("swerve c", c);
		// SmartDashboard.putNumber("swerve d", d);
		// SmartDashboard.putNumber("swerve wa1", wa1);
		// SmartDashboard.putNumber("swerve wa2", wa2);
		// SmartDashboard.putNumber("swerve wa3", wa3);
		// SmartDashboard.putNumber("swerve wa4", wa4);
		// SmartDashboard.putNumber("swerve rot", rot);
		// SmartDashboard.putNumber("swerve fwd", fwd);
		// SmartDashboard.putNumber("ws1", ws1);

		DriveTrain.setTurnOrientation(angleToPosition(wa4), angleToPosition(wa2), angleToPosition(wa1), angleToPosition(wa3), false);
		DriveTrain.setDrivePower(ws4, ws2, ws1, ws3);
	}

	public static void showDriveEncodersOnDash() {
		SmartDashboard.putNumber("Mod A Drive Enc", moduleA.getDriveEncoder());
		SmartDashboard.putNumber("Mod B Drive Enc", moduleB.getDriveEncoder());
		SmartDashboard.putNumber("Mod C Drive Enc", moduleC.getDriveEncoder());
		SmartDashboard.putNumber("Mod D Drive Enc", moduleD.getDriveEncoder());

	}

	public static void showTurnEncodersOnDash() {
		SmartDashboard.putNumber("TURN A RAW", round(moduleA.getTurnAbsolutePosition(), 3));
		SmartDashboard.putNumber("TURN B RAW", round(moduleB.getTurnAbsolutePosition(), 3));
		SmartDashboard.putNumber("TURN C RAW", round(moduleC.getTurnAbsolutePosition(), 3));
		SmartDashboard.putNumber("TURN D RAW", round(moduleD.getTurnAbsolutePosition(), 3));

		SmartDashboard.putNumber("TURN A ENC", round(moduleA.getTurnRelativePosition(), 3));
		SmartDashboard.putNumber("TURN B ENC", round(moduleB.getTurnRelativePosition(), 3));
		SmartDashboard.putNumber("TURN C ENC", round(moduleC.getTurnRelativePosition(), 3));
		SmartDashboard.putNumber("TURN D ENC", round(moduleD.getTurnRelativePosition(), 3));

		SmartDashboard.putNumber("TURN A POS", moduleA.getTurnEncoderValue());
		SmartDashboard.putNumber("TURN B POS", moduleB.getTurnEncoderValue());
		SmartDashboard.putNumber("TURN C POS", moduleC.getTurnEncoderValue());
		SmartDashboard.putNumber("TURN D POS", moduleD.getTurnEncoderValue());

		SmartDashboard.putNumber("TURN A ANGLE", round(moduleA.getTurnAngle(), 0));
		SmartDashboard.putNumber("TURN B ANGLE", round(moduleB.getTurnAngle(), 0));
		SmartDashboard.putNumber("TURN C ANGLE", round(moduleC.getTurnAngle(), 0));
		SmartDashboard.putNumber("TURN D ANGLE", round(moduleD.getTurnAngle(), 0));

		SmartDashboard.putNumber("TURN A ERR", moduleA.getTurnError());
		SmartDashboard.putNumber("TURN B ERR", moduleB.getTurnError());
		SmartDashboard.putNumber("TURN C ERR", moduleC.getTurnError());
		SmartDashboard.putNumber("TURN D ERR", moduleD.getTurnError());
	}

	public static void humanDrive(double fwd, double str, double rot) {
		if (getInstance() == null)
			return;

		if (Math.abs(rot) < 0.01)
			rot = 0;

		if (Math.abs(fwd) < .15 && Math.abs(str) < .15 && Math.abs(rot) < 0.01) {
			// setOffSets();
			setDriveBrakeMode(true);
			stopDrive();
		} else {
			setDriveBrakeMode(false);
			swerveDrive(fwd, str, rot);
			// resetOffSet();
		}
	}

	public static void pidDrive(double fwd, double strafe, double angle) {
		if (getInstance() == null)
			return;

		double temp = (fwd * Math.cos(RobotGyro.getGyroAngleInRad()))
				+ (strafe * Math.sin(RobotGyro.getGyroAngleInRad()));
		strafe = (-fwd * Math.sin(RobotGyro.getGyroAngleInRad())) + (strafe * Math.cos(RobotGyro.getGyroAngleInRad()));
		fwd = temp;
		if (!pidControllerRot.isEnabled())
			pidControllerRot.enable();
		if (Math.abs(fwd) < .15 && Math.abs(strafe) < .15) {
			pidFWD = 0;
			pidSTR = 0;
		} else {
			setDriveBrakeMode(false);
			pidFWD = fwd;
			pidSTR = strafe;
		}
		pidControllerRot.setSetpoint(angle);
	}

	public static void fieldCentricDrive(double fwd, double strafe, double rot) {
		if (getInstance() == null)
			return;

		double temp = (fwd * Math.cos(RobotGyro.getGyroAngleInRad()))
				+ (strafe * Math.sin(RobotGyro.getGyroAngleInRad()));
		strafe = (-fwd * Math.sin(RobotGyro.getGyroAngleInRad())) + (strafe * Math.cos(RobotGyro.getGyroAngleInRad()));
		fwd = temp;
		humanDrive(fwd, strafe, rot);
	}

	public static void tankDrive(double left, double right) {
		if (getInstance() == null)
			return;
		setAllTurnOrientation(0);
		setDrivePower(right, left, right, left);
	}

	/*
	 * 
	 * PID Stuff
	 */

	@Override
	public void pidWrite(double output) {
		if (getInstance() == null)
			return;

		if (Math.abs(pidControllerRot.getError()) < Calibration.DT_ROT_PID_IZONE) {
			pidControllerRot.setPID(Calibration.DT_ROT_PID_P, Calibration.DT_ROT_PID_I, Calibration.DT_ROT_PID_D);
		} else {
			// I Zone
			pidControllerRot.setPID(Calibration.DT_ROT_PID_P, 0, Calibration.DT_ROT_PID_D);
			pidControllerRot.setContinuous(true);
		}
		if (inPIDRotationMode) {
			swerveDrive(pidFWD, pidSTR, output);
		} else {
			swerveDrive(-output, pidSTR, 0);
		}
	}

	public static void disablePID() {
		if (getInstance() == null)
			return;
		pidControllerRot.disable();
	}

	public static void setDistancePIDMode() {
		inPIDRotationMode = false;
	}

	public static void setRotationPIDMode() {
		inPIDRotationMode = true;
	}

	public static double[] getAllAbsoluteTurnOrientations() {
		if (getInstance() == null)
			return new double[4];
		return new double[] { moduleA.getTurnAbsolutePosition(), moduleB.getTurnAbsolutePosition(),
				moduleC.getTurnAbsolutePosition(), moduleD.getTurnAbsolutePosition() };
	}

	public static void setDrivePIDValues(double p, double i, double d) {
		if (getInstance() == null)
			return;

		moduleA.setDrivePIDValues(p, i, d);
		moduleB.setDrivePIDValues(p, i, d);
		moduleC.setDrivePIDValues(p, i, d);
		moduleD.setDrivePIDValues(p, i, d);
	}

	public static void setTurnPIDValues(double p, double i, double d) {
		if (getInstance() == null)
			return;

		moduleA.setTurnPIDValues(p, i, d);
		moduleB.setTurnPIDValues(p, i, d);
		moduleC.setTurnPIDValues(p, i, d);
		moduleD.setTurnPIDValues(p, i, d);
	}

	private static volatile double pidFWD = 0, pidSTR = 0;

	public static Double round(Double val, int scale) {
		return new BigDecimal(val.toString()).setScale(scale, RoundingMode.HALF_UP).doubleValue();
	}

}