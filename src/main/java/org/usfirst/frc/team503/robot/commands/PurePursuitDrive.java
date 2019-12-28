package org.usfirst.frc.team503.robot.commands;


import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.motionProfiling.PathFollowerPure;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.Gyro;
import org.usfirst.frc.team503.robot.utils.FFDashboard;
import org.usfirst.frc.team503.robot.utils.Pose;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import motionProfiling.Trajectory;

/**
 *
 */
public class PurePursuitDrive extends Command {
	PathFollowerPure mPathFollower;
	Timer time;
	Drive mDrive;
	Trajectory trajectory = null;
	String file;
	boolean done;
	boolean forceReverse;
	boolean highGear;
	int resetPoint;
	double startTime;
	boolean isPreLoaded;
	boolean isReversed;
	boolean useCamera;
	private double lookahead = Robot.bot.lookAheadDistance;

	FFDashboard graphTable = new FFDashboard("Graph");
	FFDashboard purePursuitTable = new FFDashboard("PurePursuit");

	boolean isTest = false;

	public PurePursuitDrive(String file) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.file = file;
		done = false;
		resetPoint = -1;
		time = new Timer();
		highGear = false;
		isPreLoaded = false;
	}

	public PurePursuitDrive(String file, int resetPoint) {
		this.file = file;
		done = false;
		this.resetPoint = resetPoint;
		time = new Timer();
		highGear = false;
		isPreLoaded = false;
	}

	public PurePursuitDrive(String file, boolean highGear) {
		this.file = file;
		done = false;
		resetPoint = -1;
		time = new Timer();
		this.highGear = highGear;
		isPreLoaded = false;
	}

	public PurePursuitDrive(String file, boolean highGear, boolean isTest) {
		this.file = file;
		done = false;
		resetPoint = -1;
		time = new Timer();
		this.highGear = highGear;
		isPreLoaded = false;
		this.isTest = isTest;
	}


	public PurePursuitDrive(Trajectory traj) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this(traj, false, false);
	}

	public PurePursuitDrive(Trajectory traj, boolean reversed, boolean useCamera) {
		this.trajectory = traj;
		this.isReversed = reversed;
		this.useCamera = useCamera;
		done = false;
		resetPoint = -1;
		time = new Timer();
		highGear = false;
		isPreLoaded = true;
	}

	public PurePursuitDrive(Trajectory traj, boolean reversed, double lookahead) {
		this.trajectory = traj;
		this.isReversed = reversed;
		done = false;
		resetPoint = -1;
		time = new Timer();
		highGear = false;
		isPreLoaded = true;
		this.lookahead = lookahead;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		startTime = Timer.getFPGATimestamp();
		mDrive = Drive.getInstance();
		mPathFollower = PathFollowerPure.getInstance();
		mPathFollower.reset();
		// mDrive.resetEncoders();
		RobotState.getInstance().setDriveProfileDone(false);
		double initAngle = Gyro.getInstance().getAngle();
		double currentPathNum = purePursuitTable.getNumber("pathNumber", 1);
		purePursuitTable.putNumber("pathNumber", currentPathNum + 1);
		// initAngle = Gyro.getInstance().getTranslatedAngle();
		// Gyro.getInstance().resetGyro();

		if (!isPreLoaded) {
			trajectory = mPathFollower.createTrajectory(file);
		}

		if (trajectory == null) {
			done = true;
			mPathFollower.kill();
			System.out.println("File doesn't exist on RoboRIO.");
		} else {
			// if (!isTest) {

			if (isPreLoaded) {
				mPathFollower.configFollowers(trajectory, resetPoint, isReversed, lookahead);
			} else {
				mPathFollower.configFollowers(trajectory, resetPoint, lookahead);
			}

			if (mPathFollower.getReversed()) {
				mPathFollower.configDrivePIDF(Robot.bot.KP_PurePursuitReverse, Robot.bot.kI_PurePursuit, Robot.bot.kD_PurePursuit,
					Robot.bot.kV_PurePursuit, Robot.bot.kA_PurePursuit);
				mPathFollower.configTurnPID(Robot.bot.turnPurePursuitKpReverse, Robot.bot.turnPurePursuitKi,
						Robot.bot.turnPurePursuitKd);
			} else {
				mPathFollower.configDrivePIDF(Robot.bot.kP_PurePursuit, Robot.bot.kI_PurePursuit, Robot.bot.kD_PurePursuit,
					Robot.bot.kV_PurePursuit, Robot.bot.kA_PurePursuit);
				mPathFollower.configTurnPID(Robot.bot.turnPurePursuitKp, Robot.bot.turnPurePursuitKi,
						Robot.bot.turnPurePursuitKd);
			}
			// } else {
			// double[] pidf = PIDFDataHandler.getInstance().getCurrentPIDF();
			// switch (PIDFDataHandler.getInstance().getCurrentTest()) {
			// case DRIVE_STRAIGHT_PIDF:
			// mPathFollower.configDrivePIDF(Robot.bot.kP_PurePursuit,
			// Robot.bot.kI_PurePursuit,
			// Robot.bot.kD_PurePursuit, Robot.bot.kV_PurePursuit,
			// Robot.bot.kA_PurePursuit);
			// mPathFollower.configTurnPID(pidf[0], pidf[1], pidf[3]);
			// break;
			// case TURN_PID:
			// mPathFollower.configDrivePIDF(pidf[0], pidf[1], pidf[2], pidf[3], pidf[4]);
			// mPathFollower.configTurnPID(Robot.bot.turnPurePursuitKp,
			// Robot.bot.turnPurePursuitKi,
			// Robot.bot.turnPurePursuitKd);
			// break;
			// default:
			// break;
			// }
			// }

			System.out.println("init to run:" + (Timer.getFPGATimestamp() - startTime));
			if (useCamera){
				Pose cameraEstimate = VisionLocalizer.getInstance().getFieldCoordinates(VisionLocalizer.getInstance().translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles()));
				if (cameraEstimate.toVector().distanceTo(RobotState.getInstance().getPoseOdometry().toVector())<=5.0){
					RobotState.getInstance().setPose(cameraEstimate);
				}
			}
			startTime = Timer.getFPGATimestamp();
			mPathFollower.followPath(initAngle, useCamera);
		}

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return done || mPathFollower.endOfPath() /*|| RobotState.getInstance().lostTarget() *//*|| Timer.getFPGATimestamp()-startTime > 5.0*/|| DriverStation.getInstance().isDisabled() /*|| mPathFollower.velocityZero()*/;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Drive.getInstance().zeroPowerMotors();
		mPathFollower.kill();
		mDrive.zeroPowerMotors();
		RobotState.getInstance().setDriveProfileDone(true);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
