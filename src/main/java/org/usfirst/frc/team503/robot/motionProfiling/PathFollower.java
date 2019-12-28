package org.usfirst.frc.team503.robot.motionProfiling;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.Gyro;
import org.usfirst.frc.team503.robot.utils.DataEntry;
import org.usfirst.frc.team503.robot.utils.FFDashboard;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.FitMethod;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class PathFollower {
	// private variables go here
	private FFEncoderFollower leftEncFollower, rightEncFollower;
	private double max_velocity, max_acceleration, max_jerk;

	private static boolean isReversed;
	private static int pathNum = 0;
	private Drive mDrive;
	private Gyro mGyro;
	private boolean highGear;
	private boolean runPath = false;
	private int i = 0;
	private double initAngle;
	private double lastAngleDifference;
	DataEntry angle_difference = new DataEntry("Angle Difference");
	private FFDashboard motionProfilingTable = new FFDashboard("MotionProfiling");
	private BufferedWriter fileWriter;

	private class PeriodicRunnable implements Runnable {
		public void run() {
			/*
			 * loop to follow path calculating motor settings on each loop
			 */
			if (runPath) {

				
				if (i==0){
					try {
						fileWriter = new BufferedWriter(new FileWriter("/home/lvuser/pathDiagnositics"+pathNum+".txt"));
						fileWriter.write("Opening");
						fileWriter.write("max vel:"+Robot.bot.kMaxVelocityInchesPerSec);
						fileWriter.write("\n");
						fileWriter.write("Kp: "+Robot.bot.kDrivePositionKp);
						fileWriter.write("\n");
						fileWriter.write("Ki: " +Robot.bot.kDrivePositionKi);
						fileWriter.write("\n");
						fileWriter.write("Kd: " +Robot.bot.kDrivePositionKd);
						fileWriter.write("\n");
						fileWriter.write("turn p: " +Robot.bot.turnMP_P_Forward);
						fileWriter.write("\n");
						fileWriter.write("turn d: " +Robot.bot.turnMP_D_Forward);
						fileWriter.write("\n");
						fileWriter.write("kA:" +Robot.bot.kA);
						fileWriter.write("\n");
						
					} catch (IOException e) {
						e.printStackTrace();
					}
				}
				i++;
				int lp, rp;

				lp = (int) mDrive.getLeftPosition();
				rp = (int) mDrive.getRightPosition();

				// ????????????????????
				// lp=lp*-1;
				// rp=rp*-1;

				double l, r;

				motionProfilingTable.putNumber("Left Velocity", leftEncFollower.getSegment().velocity);
				motionProfilingTable.putNumber("Right Velocity", rightEncFollower.getSegment().velocity);

				l = leftEncFollower.calculate(lp);
				r = rightEncFollower.calculate(rp);

				double gyro_heading = mGyro.getAngle(); // mGyro.getHeading(); //should be in degrees
				double desired_heading = r2d(leftEncFollower.getHeading()); // should be in degrees
				double angleDifference = boundHalfDegrees(desired_heading - gyro_heading);
				System.out.println("angle diff:" + angleDifference);

				double turn;

				if (isReversed) {
					turn = Robot.bot.turnMP_P_Backward * angleDifference
							+ Robot.bot.turnMP_D_Backward * (angleDifference - lastAngleDifference) / 0.05; // 0.8*
																											// (-1.0/80.0)
																											// *
																											// angleDifference;
				} else {
					turn = Robot.bot.turnMP_P_Forward * angleDifference
							+ Robot.bot.turnMP_D_Forward * (angleDifference - lastAngleDifference) / 0.05; // 0.8*
																											// (-1.0/80.0)
																											// *
																											// angleDifference;
				}

				if (leftEncFollower.getSegNum() > leftEncFollower.getNumSegments() / 2) {
					turn /= 2;
				}
				mDrive.tankDrive((l + turn), (r - turn));

				SmartDashboard.putNumber("Gyro Heading:", gyro_heading);
				SmartDashboard.putNumber("Desired Heading:", desired_heading);
				SmartDashboard.putNumber("Initial Heading", initAngle);
				SmartDashboard.putNumber("Angle Difference:", angleDifference);
				SmartDashboard.putBoolean("Motion profile reversed", isReversed);
				SmartDashboard.putNumber("Left Encoder Position", lp);
				SmartDashboard.putNumber("Right Encoder Position", rp);
				SmartDashboard.putNumber("Left Motor Power", (l + turn));
				SmartDashboard.putNumber("Right Motor Power", (r - turn));
				SmartDashboard.putNumber("Turn Power", turn);
				SmartDashboard.putBoolean("Pathfinder End-of-Path", endOfPath());
				SmartDashboard.putNumber("Step", i);

				try {
					fileWriter.write("Gyro Heading:" + gyro_heading);
					fileWriter.write("\n");
					fileWriter.write("Desired Heading:" + desired_heading);
					fileWriter.write("\n");
					fileWriter.write("Initial Heading: "+ initAngle);
					fileWriter.write("\n");
					fileWriter.write("Angle Difference: "+ angleDifference);
					fileWriter.write("\n");
					fileWriter.write("Motion profile reversed: "+ isReversed);
					fileWriter.write("\n");
					fileWriter.write("Left Encoder Position: "+ lp);
					fileWriter.write("\n");
					fileWriter.write("Right Encoder Position: "+ rp);
					fileWriter.write("\n");
					fileWriter.write("Left Motor Power: "+ (l + turn));
					fileWriter.write("\n");
					fileWriter.write("Right Motor Power: "+ (r - turn));
					fileWriter.write("\n");
					fileWriter.write("Turn Power: "+ turn);
					fileWriter.write("\n");
					fileWriter.write("Pathfinder End-of-Path: "+ endOfPath());
					fileWriter.write("Step"+ i);
					fileWriter.write("\n");
					fileWriter.write("\n");
					fileWriter.write("\n");
				} catch (IOException e) {
					e.printStackTrace();
				}
				
				
				angle_difference.addValue(angleDifference);

				// Test for end of path
				if (leftEncFollower.isFinished() && rightEncFollower.isFinished()) {
					runPath = false;
					// stop motors
					mDrive.tankDrive(0, 0);

				}

				lastAngleDifference = angleDifference;
			} // end if runPath
		} // end run loop
	} // end Runnable class

	private Notifier notifier = new Notifier(new PeriodicRunnable());

	// Constructor
	public PathFollower() {
		// get pointer to Drive Subsystem
		mDrive = Drive.getInstance();

		// get pointer to Gyro
		mGyro = Gyro.getInstance();

		// Setup constants for Path creation, in Inches/second
		max_velocity = Robot.bot.kMaxVelocityInchesPerSec; // was 1.7
		max_acceleration = Robot.bot.kMaxAccelerationInchesPerSec; // was 2.0
		max_jerk = Robot.bot.kMaxJerkInchesPerSec; // was 60.0
		
		
	}	// end constructor
	
	
	private static PathFollower instance = new PathFollower();
		
	public static PathFollower getInstance(){
		return instance;
   }
	
	public void followPath(double initAngle) {
		//Setup regular execution of periodic runnable 
		runPath = true;
		notifier.startPeriodic(leftEncFollower.getSegment().dt);
		this.initAngle = initAngle;
	}
	
	
	
	/* 
	 * Convert list of Waypoints into robot trajectory  * 
	 */
	public Trajectory[] createTrajectory(String file) {    
		//config parms 
		//parm 1 - fit method (fit) 
		//parm 2 - samples (int) 
		//parm 3 - dt (double) - 0.05 
		//parm 4 - max velocity (double) - 1.7 
		//parm 5 - max acceleration (double) - 2.0 
		//parm 6 - max jerk (double) - 60.0 
		
		//Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, max_velocity, max_acceleration, max_jerk);
		
		Trajectory[] trajectory = readFromFile(file);
		//Trajectory trajectory = Pathfinder.generate(points, config);

		return trajectory;
	}
	
	public Trajectory[] createTrajectory(Waypoint[] points) {    
		//config parms 
		//parm 1 - fit method (fit) 
		//parm 2 - samples (int) 
		//parm 3 - dt (double) - 0.05 
		//parm 4 - max velocity (double) - 1.7 
		//parm 5 - max acceleration (double) - 2.0 
		//parm 6 - max jerk (double) - 60.0 
		
		Trajectory.Config config = new Trajectory.Config(FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, max_velocity, max_acceleration, max_jerk);
		Trajectory trajectory = Pathfinder.generate(points, config);
		TankModifier modifier = new TankModifier(trajectory).modify(Robot.bot.WHEEL_BASE_INCHES);
		Trajectory[] returner = {modifier.getLeftTrajectory(),modifier.getRightTrajectory()};
		return returner;
	}
	
	   
	/*
	 *  
	 * Based on a given robot trajectory initialize encoder followers  
	 */
	public void configFollowers(Trajectory[] trajectory, int resetPoint, boolean highGear) { 
		this.highGear = highGear;
		// Wheelbase Width = 0.5m
		//TankModifier modifier = new TankModifier(trajectory).modify(Robot.bot.WHEEL_BASE_INCHES);
		
		//Create Encoder Followers 
		leftEncFollower = new FFEncoderFollower(trajectory[0],isReversed);
		rightEncFollower = new FFEncoderFollower(trajectory[1],isReversed);
		
		//Configure Encoders 
		leftEncFollower.configureEncoder((int)mDrive.getLeftPosition(), Robot.bot.kEncoderUnitsPerRev, Robot.bot.kDriveWheelDiameterInches);
		rightEncFollower.configureEncoder((int)mDrive.getRightPosition(), Robot.bot.kEncoderUnitsPerRev, Robot.bot.kDriveWheelDiameterInches);
	  
		//Configure PID Values
		// parm 1 - proportional gain 
		// parm 2 - integral gain - unused for motion profiling 
		// parm 3 - derivative gain- tweak this is unhappy with tracking trajectory 
		// parm 4 - velocity ratio - this is 1 over the maximum velocity you provided in configure
		//						    (it translates  m/s to a -1 to +1 scale that the motor scan read) 
		// parm 5 - acceleration gain - Tweak this is you want to get to a higher or lower speed quicker
		
		leftEncFollower.configurePIDVA(Robot.bot.kDrivePositionKp, Robot.bot.kDrivePositionKi, Robot.bot.kDrivePositionKd, 1/Robot.bot.kMaxVelocityInchesPerSec, Robot.bot.kA);
		rightEncFollower.configurePIDVA(Robot.bot.kDrivePositionKp, Robot.bot.kDrivePositionKi, Robot.bot.kDrivePositionKd, 1/Robot.bot.kMaxVelocityInchesPerSec, Robot.bot.kA);
	


		leftEncFollower.setResetPoint(resetPoint);
		rightEncFollower.setResetPoint(resetPoint);
	}
	   	   
	  /* 
	   * Test for end of path - when both encoder followers report finished set true to end of path 
	   */
	public synchronized boolean endOfPath() {
		boolean ret = false; 
		
		if(leftEncFollower.isFinished() && rightEncFollower.isFinished() ) {
			runPath = false; 
			ret = true; 
			try {
				fileWriter.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
			reset();
		} 
		  
		return ret;
	}

	public void setReversed(boolean reversed) {
		this.isReversed = reversed;
	}
	
	public synchronized void reset() {
		lastAngleDifference = 0.0;
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
        while (angle_degrees >= 180.0) angle_degrees -= 360.0;
        while (angle_degrees < -180.0) angle_degrees += 360.0;
        return angle_degrees;
    }   
  
    
    public static Trajectory[] readFromFile(String file) {
    	if(!file.endsWith(".txt")) {
    		file = file + ".txt";
    	}
    	if(!file.startsWith("/home/lvuser/MotionProfiles/")) {
    		file = "/home/lvuser/MotionProfiles/" + file;
    	}
    	Trajectory[] trajectories = new Trajectory[2];
    	try(BufferedReader in = new BufferedReader(new FileReader(file))){
    		int length = Integer.parseInt(in.readLine());
    		trajectories[0] = new Trajectory(length);
    		trajectories[1] = new Trajectory(length);
    		String[] data;
    		for(int i=0; i<length; i++) {
    			String line = in.readLine();
    			data = line.split(",");
    			trajectories[0].segments[i] = new Segment(Double.parseDouble(data[0]), Double.parseDouble(data[1]), Double.parseDouble(data[2]), Double.parseDouble(data[3]), 
    												Double.parseDouble(data[4]), Double.parseDouble(data[5]), Double.parseDouble(data[6]), Double.parseDouble(data[7]));
    			}
    		for(int i=0; i<length; i++) {
    			String line = in.readLine();
    			data = line.split(",");
    			trajectories[1].segments[i] = new Segment(Double.parseDouble(data[0]), Double.parseDouble(data[1]), Double.parseDouble(data[2]), Double.parseDouble(data[3]), 
    												Double.parseDouble(data[4]), Double.parseDouble(data[5]), Double.parseDouble(data[6]), Double.parseDouble(data[7]));
    			/*
    			 *
    			trajectories[1].get(i).dt = Double.parseDouble(data[0]);
    			trajectories[1].get(i).x = Double.parseDouble(data[1]);
    			trajectories[1].get(i).y = Double.parseDouble(data[2]);
    			trajectories[1].get(i).position = Double.parseDouble(data[3]);
    			trajectories[1].get(i).velocity = Double.parseDouble(data[4]);
    			trajectories[1].get(i).acceleration = Double.parseDouble(data[5]);
    			trajectories[1].get(i).jerk = Double.parseDouble(data[6]);
    			trajectories[1].get(i).heading = Double.parseDouble(data[7]);*/
    		}
    		isReversed = Boolean.parseBoolean(in.readLine());
    	}
    	catch(Exception e) {
    		e.printStackTrace();
    	}
    	return trajectories;
    }
    
   public boolean getReversed() {
	   return isReversed;
   }

	
}	// End pathFollower