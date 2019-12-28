package org.usfirst.frc.team503.robot.motionProfiling;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.Drive.DriveMotorOutput;
import org.usfirst.frc.team503.robot.subsystems.Gyro;
import org.usfirst.frc.team503.robot.utils.DataEntry;
import org.usfirst.frc.team503.robot.utils.FFDashboard;
import org.usfirst.frc.team503.robot.utils.Pose;
import org.usfirst.frc.team503.robot.utils.UniversalGrapher;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import motionProfiling.Config;
import motionProfiling.PathGenerator;
import motionProfiling.Points;
import motionProfiling.Trajectory;

public class PathFollowerPure {
	// private variables go here
	private FFPureEncoderFollower follower;
	private double max_velocity, max_acceleration, max_jerk;
	private FFDashboard table = new FFDashboard("PurePursuit");
	private boolean isReversed;
	private Drive mDrive;
	private Gyro mGyro;
	private BufferedWriter fileWriter;
	private boolean highGear;
	private boolean runPath = false;
	private int i = 0;
	private double initAngle;
	private Pose lastPose;
	private Trajectory trajectory;
	private double lastTime;
	private boolean useCamera = false;
	private boolean dontStop = false;

	private double lastAngleDifference;
	DataEntry angle_difference = new DataEntry("Angle Difference");

	private double kDriveP, kDriveI, kDriveD, kDriveV, kDriveA;
	private double kTurnP, kTurnI, kTurnD;
	private double lookahead;

	private class PeriodicRunnable implements Runnable {

		private FFPureEncoderFollower follower;

		public void run() {
			/*
			 * loop to follow path calculating motor settings on each loop
			 */
			// System.out.println(runPath);
			if (runPath) {
				if (i == 0) {
					follower = new FFPureEncoderFollower(trajectory, isReversed, lookahead);
				}
				i++;

				Pose robotPose = RobotState.getInstance().getPoseOdometry();

				DriveMotorOutput motorOutput = follower.calculate(robotPose);

				double leftVelocity = motorOutput.getLeftVelocity(), rightVelocity = motorOutput.getRightVelocity();

				double leftTargetAccel = motorOutput.getLeftAccel(), rightTargetAccel = motorOutput.getRightAccel();

				double leftPower = 0.0, rightPower = 0.0;
				if (leftTargetAccel < 0) {
					leftPower = leftVelocity;// + leftTargetAccel;
				} else {
					leftPower = leftVelocity + leftTargetAccel;
				}
				if (rightTargetAccel < 0) {
					rightPower = rightVelocity;// + rightTargetAccel;
				} else {
					rightPower = rightVelocity + rightTargetAccel;
				}

				// FeedForward
				// leftPower = (leftVelocity * kDriveV) + (kDriveA * leftTargetAccel);
				// rightPower = (rightVelocity * kDriveV) + (kDriveA * rightTargetAccel);

				// Feedback
				double actualLeftVel = Drive.getInstance().getLeftVelocityInches();
				double actualRightVel = Drive.getInstance().getRightVelocityInches();
				if (isReversed) {
					actualLeftVel *= -1;
					actualRightVel *= -1;
				}
				DriveMotorOutput target = follower.calculatePureParameterizedDesiredVelocities();
				// System.out.println("TARGET L:" + target.getLeftPercent());
				// System.out.println("TARGET R:" + target.getRightPercent());
				// System.out.println("ACTUAL L:" + actualLeftVel);
				// System.out.println("ACTUAL R:" + actualRightVel);
				double leftError = target.getLeftPercent() - actualLeftVel;
				double rightError = target.getRightPercent() - actualRightVel;
				// if (isReversed) {
				// kDriveP = Robot.bot.KP_PurePursuitReverse;
				// }
				// if (!isReversed){
				leftPower += leftError * kDriveP;
				rightPower += rightError * kDriveP;
				// }

				double gyro_heading = (robotPose.getTheta()); // mGyro.getHeading(); //should be in degrees
				
				double desired_heading = follower.getSegment().getHeadingInDegrees(); // should be in degrees
				double angleDifference = boundHalfDegrees(gyro_heading - desired_heading);
				
				double leftRaw = leftPower;
				double rightRaw = rightPower;

				double turn;
				// System.out.println("actual:" + gyro_heading);
				// System.out.println("desired:" + desired_heading);
				// System.out.println("ang" + angleDifference);

				// if (isReversed) {
				// turn = -0.03 * angleDifference;
				// } else {
				turn = kTurnP * angleDifference;
				// }
				if (isReversed) {
					// leftPower -= turn;
					// rightPower += turn;
				} else {
					leftPower += turn;
					// rightPower -= turn;
				}
				System.out.println("t:" + turn);
				System.out.println("pre lv:" + leftPower);
				System.out.println("pre rv:" + rightPower);
				// Normalize wheel speeds
				double maxMagnitude = Math.max(Math.abs(leftPower), Math.abs(rightPower));
				if (maxMagnitude > 1.0) {
					System.out.println("over max");
					leftPower /= maxMagnitude;
					rightPower /= maxMagnitude;
				}
				// leftPower = Math.abs(leftPower);
				// rightPower = Math.abs(rightPower);

				if (i <= 10) {
					if (leftPower < 0.0) {
						System.out.println("LEFT POWER LESS THAN ZERO fix");

					}
					if (rightPower < 0.0) {
						System.out.println("RIGHT POWER LESS THAN ZERO fix");
					}
					leftPower = Math.abs(leftPower);
					rightPower = Math.abs(rightPower);
				}
				if (leftPower < 0.0) {
					System.out.println("LEFT POWER LESS THAN ZERO non fix");

				}
				if (rightPower < 0.0) {
					System.out.println("RIGHT POWER LESS THAN ZERO non fix");
				}
				motorOutput.setPercentPowers(leftPower, rightPower);
				System.out.println("IS REVERSED VAR:" + isReversed);
				motorOutput.setDriveDirection(isReversed);
				System.out.println("Left:" + leftPower + " Right:" + rightPower);
				mDrive.tankDrive(motorOutput);

				// Test for end of path
				if (follower.isFinished(useCamera)) {
					System.out.println("finshed");
					runPath = false;
					// stop motors
					mDrive.tankDrive(0, 0);
					UniversalGrapher.addEntries(follower.time);
					UniversalGrapher.addEntries(follower.error);
					UniversalGrapher.addEntries(follower.power);
					UniversalGrapher.addEntries(angle_difference);
					// try{
					// // fileWriter.close();
					// }
					// catch(IOException e){
					// e.printStackTrace();
					// }

				}

				SmartDashboard.putNumber("Initial Heading", initAngle);

				SmartDashboard.putBoolean("Motion profile reversed", isReversed);

				table.putNumber("Left Motor Power", motorOutput.getLeftPercent());
				table.putNumber("Right Motor Power", motorOutput.getRightPercent());

				SmartDashboard.putNumber("Left Motor Power", motorOutput.getLeftPercent());
				SmartDashboard.putNumber("Right Motor Power", motorOutput.getRightPercent());
				System.out.println("left motor:" + motorOutput.getLeftPercent());
				System.out.println("right motor:" + motorOutput.getRightPercent());
				SmartDashboard.putBoolean("Pathfinder End-of-Path", endOfPath());
				SmartDashboard.putNumber("Step", i);
				lastPose = robotPose;
				lastTime = Timer.getFPGATimestamp();
				//try {
					/*
					fileWriter.write("Gyro Heading: " + gyro_heading);
					fileWriter.write("\n");
					fileWriter.write("Desired Heading: " + desired_heading);
					fileWriter.write("\n");
					fileWriter.write("Initial Heading: "+ initAngle);
					fileWriter.write("\n");
					fileWriter.write("Angle Difference: "+ angleDifference);
					fileWriter.write("\n");
					fileWriter.write("Motion profile reversed: "+ isReversed);
					fileWriter.write("\n");
					fileWriter.write("X: "+ robotPose.getX());
					fileWriter.write("\n");
					fileWriter.write("Y: "+ robotPose.getY());
					fileWriter.write("\n");
					fileWriter.write("Look Ahead X: "+ follower.getLookAheadPose().i());
					fileWriter.write("\n");
					fileWriter.write("Look Ahead Y: "+ follower.getLookAheadPose().j());
					fileWriter.write("\n");
					fileWriter.write("Right Motor Requested Vel: " + target.getRightPercent());
					fileWriter.write("\n");
					fileWriter.write("Left Motor Requested Vel: " + target.getLeftPercent());
					fileWriter.write("\n");
					fileWriter.write("Right Motor Power pre turn: " + rightRaw);
					fileWriter.write("\n");
					fileWriter.write("Left Motor Power pre turn: " + leftRaw);
					fileWriter.write("\n");
					fileWriter.write("Right Motor Power after turn: " + rightPower);
					fileWriter.write("\n");
					fileWriter.write("Left Motor Power after turn: " + leftPower);
					fileWriter.write("\n");
					fileWriter.write("Turn Power: "+ turn);
					fileWriter.write("\n");
					fileWriter.write("Get Calculated Curvature: "+ follower.getCalculatedCurvature());
					fileWriter.write("\n");
					fileWriter.write("Pathfinder End-of-Path: "+ endOfPath());
					fileWriter.write("Step"+ i);
					fileWriter.write("\n");
					fileWriter.write("\n");
					fileWriter.write("\n");
					*/
				// } catch (IOException e) {
				// e.printStackTrace();
				// }

			} // end if runPath
		} // end run loop
	} // end Runnable class

	private Notifier notifier = new Notifier(new PeriodicRunnable());

	// Constructor
	public PathFollowerPure() {
		// get pointer to Drive Subsystem
		mDrive = Drive.getInstance();

		// get pointer to Gyro
		mGyro = Gyro.getInstance();

		// Setup constants for Path creation, in Inches/second
		max_velocity = Robot.bot.kMaxVelocityInchesPerSec;
		max_acceleration = Robot.bot.kMaxAccelerationInchesPerSec;
		max_jerk = Robot.bot.kMaxJerkInchesPerSec;
	//	try{
			/*
			fileWriter = new BufferedWriter(new FileWriter("/home/lvuser/pathTest1.txt"));
			fileWriter.write("Opening");
			fileWriter.write("max vel:"+Robot.bot.kMaxVelocityInchesPerSec);
			fileWriter.write("\n");
			fileWriter.write("Kp: "+Robot.bot.kP_PurePursuit);
			fileWriter.write("\n");
			fileWriter.write("Ki: " +Robot.bot.kI_PurePursuit);
			fileWriter.write("\n");
			fileWriter.write("Kd: " +Robot.bot.kD_PurePursuit);
			fileWriter.write("\n");
			fileWriter.write("curve fudge: " +Robot.bot.curvatureFudge);
			fileWriter.write("\n");
			fileWriter.write("look ahead distance: " + Robot.bot.lookAheadDistance);
			fileWriter.write("\n");
			fileWriter.write("kA:" +Robot.bot.kA_PurePursuit);
			fileWriter.write("\n");
			*/
	//	}
	//	catch (IOException e){
	//		e.printStackTrace();
	//	}

	}

	private static PathFollowerPure instance = new PathFollowerPure();

	public static PathFollowerPure getInstance() {
		return instance;
	}

	public void followPath(double initAngle, boolean useCamera) {
		// Setup regular execution of periodic runnable

		notifier.startPeriodic(Robot.bot.DT);
		i = 0;
		this.useCamera = useCamera;
		runPath = true;
		System.out.println("setting run path to true: " + runPath);
		this.initAngle = initAngle;
	}

	/*
	 * Convert list of Waypoints into robot trajectory *
	 */
	public Trajectory createTrajectory(String file) {
		Trajectory trajectory = readFromFile(file);
		return trajectory;
	}

	public Trajectory createTrajectory(Points points) {
		Config config = new Config(0.05, max_velocity, max_acceleration, max_jerk);
		Trajectory trajectory = PathGenerator.generateCentralTrajectory(points, config);
		return trajectory;
	}

	/*
	 * 
	 * Based on a given robot trajectory initialize encoder followers
	 */
	public void configFollowers(Trajectory trajectory, int resetPoint, double lookAhead) {
		this.highGear = false;
		this.trajectory = trajectory;
		// this.isReversed = false;
		this.lookahead = lookAhead;
		follower = new FFPureEncoderFollower(trajectory, isReversed, lookAhead);
	}

	/*
	 * 
	 * Based on a given robot trajectory initialize encoder followers
	 */
	public void configFollowers(Trajectory trajectory, int resetPoint, boolean reversed, double lookahead) {
		this.trajectory = trajectory;
		this.lookahead = lookahead;
		this.isReversed = reversed;
		follower = new FFPureEncoderFollower(trajectory, isReversed, lookahead);
	}

	/*
	 * Test for end of path - when both encoder followers report finished set true
	 * to end of path
	 */
	public synchronized boolean endOfPath() {
		boolean ret = false;

		if (follower.isFinished(useCamera)) {
			runPath = false;
			ret = true;
			reset();
		}
		return ret;
	}

	public synchronized boolean velocityZero(){
		return follower.velocityZero();
	}

	public synchronized void reset() {
		lastAngleDifference = 0.0;
		i = 0;
	}

	public synchronized void kill() {
		runPath = false;
		reset();
		// notifier.stop();
	}

	public static double d2r(double degrees) {
		return Math.toRadians(degrees);
	}

	/**
	 * Convert radians to degrees. This is included here for static imports.
	 */
	public static double r2d(double radians) {
		return Math.toDegrees(radians);
	}

	/**
	 * Bound an angle (in degrees) to -180 to 180 degrees.
	 */
	public static double boundHalfDegrees(double angle_degrees) {
		while (angle_degrees >= 180.0)
			angle_degrees -= 360.0;
		while (angle_degrees < -180.0)
			angle_degrees += 360.0;
		return angle_degrees;
	}

	public Trajectory readFromFile(String file) {
		if (!file.endsWith(".txt")) {
			file = file + ".txt";
		}
		if (!file.startsWith("/home/lvuser/MotionProfiles/")) {
			file = "/home/lvuser/MotionProfiles/" + file;
		}
		Trajectory trajectories = null;
		try (BufferedReader in = new BufferedReader(new FileReader(file))) {
			int length = Integer.parseInt(in.readLine());
			trajectories = new Trajectory(length);
			String[] data;
			for (int i = 0; i < length; i++) {
				String line = in.readLine();
				data = line.split(",");
				Trajectory.Segment seg = new Trajectory.Segment();
				seg.dt = Double.parseDouble(data[0]);
				seg.x = Double.parseDouble(data[1]);
				seg.y = Double.parseDouble(data[2]);
				seg.pos = Double.parseDouble(data[3]);
				seg.vel = Double.parseDouble(data[4]);
				seg.acc = Double.parseDouble(data[5]);
				seg.jerk = Double.parseDouble(data[6]);
				seg.heading = Double.parseDouble(data[7]);
				seg.curvature = Double.parseDouble(data[8]);
				trajectories.setSegment(i, seg);
			}
			isReversed = Boolean.parseBoolean(in.readLine());
			System.out.println("REVERSED: " + isReversed);
			while (in.readLine().equals("end"));
        	this.lookahead = Robot.bot.lookAheadDistance;
            try {
                this.lookahead = Double.parseDouble(in.readLine());
            } catch (Exception e) {
                e.printStackTrace();
            }
           
		} catch (Exception e) {
			e.printStackTrace();
		}
		return trajectories;
	}

	public boolean getReversed() {
		return isReversed;
	}

	public void configDrivePIDF(double p, double i, double d, double v, double a) {
		this.kDriveP = p;
		this.kDriveI = i;
		this.kDriveD = d;
		this.kDriveV = v;
		this.kDriveA = a;
	}

	public void configTurnPID(double p, double i, double d) {
		this.kTurnP = p;
		this.kTurnI = i;
		this.kTurnD = d;
	}

	public synchronized void setRunPath(boolean b) {
		runPath = b;
	}

}
