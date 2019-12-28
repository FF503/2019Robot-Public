package org.usfirst.frc.team503.robot.utils;

import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.Gyro;

import edu.wpi.first.wpilibj.Notifier;

public class GPS {
	private double GPS_DT = 0.0075;// 0.0025;
	private volatile double x, y, theta;
	private double encLeft = 0.0;
	private double lastEncLeft = Drive.getInstance().getLeftDistanceInches();
	private double encRight = 0.0;
	private double lastEncRight = Drive.getInstance().getRightDistanceInches();
	private double lastTheta = getTranslatedTheta();
	private Pose currentPose;
	private static GPS instance = new GPS();
	private double count = 0;

	// runnable class
	public static GPS getInstance() {
		return instance;
	}

	private class PeriodicRunnable implements Runnable {
		// code inside this method runs every GPS_DT seconds
		@Override
		public void run() {

			double dx, dy;

			theta = getTranslatedTheta();
			encLeft = Drive.getInstance().getLeftDistanceInches();
			encRight = Drive.getInstance().getRightDistanceInches();
			double calculationTheta = Math.toRadians(theta - lastTheta);

			double dArc = ((encLeft - lastEncLeft) + (encRight - lastEncRight)) / 2;

			if (Math.abs(calculationTheta) >= Math.toRadians(10)) {
				dArc = (2 * dArc * Math.sin(calculationTheta / 2) / (calculationTheta));
			}

			double avgTheta = (theta + lastTheta) / 2;
			dx = Math.cos(Math.toRadians(avgTheta)) * dArc;
			dy = Math.sin(Math.toRadians(avgTheta)) * dArc;
			x += dx;
			y += dy;
			lastEncLeft = encLeft;
			lastEncRight = encRight;
			lastTheta = theta;

			currentPose.update(x, y, theta);
			RobotState.getInstance().setPose(currentPose);

			// System.out.println(x + "," + y);

		}
	}

	private Notifier runner = new Notifier(new PeriodicRunnable());

	public GPS() {
		lastEncLeft = Drive.getInstance().getLeftDistanceInches();
		lastEncRight = Drive.getInstance().getRightDistanceInches();
		lastTheta = getTranslatedTheta();
	}

	public synchronized void startTracking() {
		resetPositionData();
		runner.startPeriodic(GPS_DT);
	}

	public synchronized double getX() {
		return x;
	}

	public synchronized double getY() {
		return y;
	}

	public synchronized double getTheta() {
		return theta;
	}

	public synchronized void UpdatePositionData(Pose pose) {
		x = pose.getX();
		y = pose.getY();
		theta = pose.getTheta();
	}

	public synchronized void resetPositionData() {
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		lastEncLeft = Drive.getInstance().getLeftDistanceInches();
		lastEncRight = Drive.getInstance().getRightDistanceInches();
		lastTheta = getTranslatedTheta();
		currentPose = new Pose(x, y, theta);
		RobotState.getInstance().setPose(currentPose);
	}

	private double getTranslatedTheta() {
		return Gyro.getInstance().getTranslatedAngle();
	}

	public synchronized void stop() {
		runner.stop();
	}
}