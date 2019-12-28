package org.usfirst.frc.team503.robot.vision;

import org.usfirst.frc.team503.robot.subsystems.Gyro;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public enum TargetData {
	LEFTROCKETLEFT(-28.75, 0, 1, 17.86, 222.71), LEFTROCKETCENTER(-90, 2, 3, 27.5, 228),
	LEFTROCKETRIGHT(-151.25, 4, 5, 17.86, 233.29), LEFTCARGOLEFT(90, 0, 1, 134.125, 303.25),
	LEFTCARGOCENTER(90, 2, 3, 134.125, 281.25), LEFTCARGORIGHT(90, 4, 5, 134.125, 259.25),
	CENTERCARGOLEFT(0, 0, 1, 151, 220.875), CENTERCARGOCENTER(0, 2, 3, 173, 0), CENTERCARGORIGHT(0, 4, 5, 173, 220.875),
	RIGHTCARGOLEFT(-90, 0, 1, 189.875, 259.25), RIGHTCARGOCENTER(-90, 2, 3, 189.875, 281.25),
	RIGHTCARGORIGHT(-90, 4, 5, 189.875, 303.25), RIGHTROCKETLEFT(151.25, 0, 1, 306.14, 233.29),
	RIGHTROCKETCENTER(90, 2, 3, 296.5, 228), RIGHTROCKETRIGHT(28.75, 4, 5, 306.14, 233.29),
	HUMANPLAYERLEFT(-180, 2, 3, 27.5, 0), HUMANPLAYERRIGHT(-180, 2, 3, 296.5, 0), DEFAULT(0, 3, 4, 0, 0), SIMPLETARGET(0,5,5,0,0);

	private final double angleAdjustment;
	private final int individualPipeline;
	private final int groupedPipeline;
	private final double x;
	private final double y;

	private TargetData(double angleAdjustment, int individualPipeline, int groupedPipeline, double x, double y) {
		this.angleAdjustment = angleAdjustment;
		this.individualPipeline = individualPipeline;
		this.groupedPipeline = groupedPipeline;
		this.x = x;
		this.y = y;
	}

	/**
	 * Calculates the angle of the robot relative to the predicted target
	 * 
	 * @return Angle, in degrees
	 */
	public double getAdjustedAngle() {
		double gyroAngle = Gyro.getInstance().getAngle();
		SmartDashboard.putNumber("Raw gyro", gyroAngle);
		double correctedGyro = VisionLocalizer.getInstance().cameraDirectionCorrectGyro(gyroAngle);

		correctedGyro -= this.getTargetFieldAngle();
		SmartDashboard.putNumber("Target Field Angle", this.getTargetFieldAngle());
		while (correctedGyro < -180) {
			correctedGyro += 360;
		}

		while (correctedGyro > 180) {
			correctedGyro -= 360;
		}

		return correctedGyro;

	}

	public double getTargetFieldAngle() {
		return this.angleAdjustment;
	}

	public double getIndividualPipeline() {
		return this.individualPipeline;
	}

	public double getGroupedPipeline() {
		return this.groupedPipeline;
	}

	public double getTargetFieldX() {
		return this.x;
	}

	public double getTargetFieldY() {
		return this.y;
	}

}
