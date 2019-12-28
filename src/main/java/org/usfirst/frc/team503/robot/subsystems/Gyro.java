package org.usfirst.frc.team503.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.utils.FFDashboard;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/***********************************************
 * Class Gyro.java Description Provides all services for the Navx Gyro.
 * 
 * @author Frogforce 503 Programming team
 * @version 0.1 Change History 0.0 Initial Load
 * 
 ***********************************************/

public class Gyro extends Subsystem {

	private FFDashboard table = new FFDashboard("Gyro");

	private static AHRS gyro;
	private boolean calibration_complete = false;
	private double angle;
	private final static String kSource = "Gyro";

	/*********************************************************
	 * Constructor
	 **********************************************************/
	public Gyro() {
		try {
			gyro = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
			// FrogLogger.logMsg("ERROR GETTING GYRO" ,kSource,true);
		}

		/* NavX calibration completed ~15 seconds after it is powered on */
		/* as long as the robot is still. Hold off using Yaw value until */
		/* calibration is complete */

		while (!calibration_complete) {
			calibration_complete = !gyro.isCalibrating();
			if (!calibration_complete) {
				SmartDashboard.putString("NavX", "Calibration in Progress");
			} else {
				SmartDashboard.putString("NavX", "Calibration Completed!");
				// force reset of yaw value
				gyro.zeroYaw();
			}
		}
	}

	private static Gyro instance = new Gyro();

	/********************************************************************
	 * Public Methods
	 *******************************************************************/

	public static Gyro getInstance() {
		return instance;
	}

	public void resetGyro() {
		// System.out.println("Gyro yaw reset");
		gyro.reset();
	}

	public double getAngle() {
		return boundTo360(gyro.getAngle() + RobotState.getInstance().getGyroOffset());
	}

	public double getTranslatedAngle() {
		return 90.0 - getAngle();
	}

	public double getHeading() {
		return gyro.getFusedHeading();
	}

	public double getYaw() {
		return gyro.getYaw();
	}
	public double getPitch(){
		return gyro.getPitch();
	}
	public void sendDashboardData() {
		// SmartDashboard.putBoolean( "NavX Connected", gyro.isConnected());
		// SmartDashboard.putBoolean( "NavX Calibrating", gyro.isCalibrating());
		SmartDashboard.putNumber("NavX Angle", gyro.getAngle());
		SmartDashboard.putNumber("NavX Yaw", gyro.getYaw());
		SmartDashboard.putNumber("NavX FusedHeading", gyro.getFusedHeading());
		// SmartDashboard.putNumber( "NavX CompassHeading", gyro.getCompassHeading());
		// SmartDashboard.putNumber("NavX Pitch", gyro.getPitch());

		table.putNumber("NavX Angle", getAngle());
		table.putNumber("Pitch", getPitch());
		table.putNumber("Gyro Offset", RobotState.getInstance().getGyroOffset());
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	private double boundTo360(double angle_degrees) {
		while (angle_degrees >= 360.0)
			angle_degrees -= 360.0;
		while (angle_degrees < 0.0)
			angle_degrees += 360.0;
		return angle_degrees;
	}
}