package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.motionProfiling.PathFollower;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.Gyro;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;

/**
 *
 */
public class MPDrive extends Command {
    PathFollower mPathFollower;
    Timer time;
    Drive mDrive;
    Trajectory[] trajectory;
    String file;
    boolean done;
    boolean forceReverse;
    boolean highGear = false;
    int resetPoint;
    double startTime;
    boolean isPreloaded= false;
    boolean isReversed;

    public MPDrive(String file) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        this.file = file;
        done = false;
        resetPoint = -1;
        time = new Timer();
        highGear = false;

    }

    public MPDrive(String file, int resetPoint) {
        this.file = file;
        this.resetPoint = resetPoint;
        highGear = false;
        time = new Timer();
    }

    public MPDrive(String file, boolean highGear) {
        this.file = file;
        resetPoint = -1;
        time = new Timer();
        this.highGear = highGear;

    }

    public MPDrive(Trajectory[] trajectories, boolean isReversed) {
        trajectory = trajectories; 
        done = false;
        resetPoint = -1;
        time = new Timer();
        highGear = false;
        this.isReversed = isReversed;
        isPreloaded = true;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        startTime = Timer.getFPGATimestamp();
        mDrive = Drive.getInstance();
        mPathFollower = PathFollower.getInstance();
        mPathFollower.reset();
        mDrive.resetEncoders();
        Gyro.getInstance().resetGyro();
        RobotState.getInstance().setDriveProfileDone(false);

        //SmartDashboard.putString("Motion profile running", file);

        double initAngle;
        initAngle = Gyro.getInstance().getHeading();
        Gyro.getInstance().resetGyro();
        // trajectory = mPathFollower.createTrajectory(path);
        if(!isPreloaded) {
            trajectory = PathFollower.getInstance().createTrajectory(file);
        } else {
            PathFollower.getInstance().setReversed(isReversed);
        }
        if (trajectory[0] == null) {
            done = true;
            System.out.println("File doesn't exist on RoboRIO.");
        } else {
            mPathFollower.configFollowers(trajectory, resetPoint, highGear);
            System.out.println("init to run:" + (Timer.getFPGATimestamp() - startTime));
            mPathFollower.followPath(initAngle);
        }
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return done || PathFollower.getInstance().endOfPath();
    }

    // Called once after isFinished returns true
    protected void end() {
        // UniversalGrapher.sendValues();
        Drive.getInstance().tankDrive(0, 0);
        PathFollower.getInstance().reset();
        RobotState.getInstance().setDriveProfileDone(true);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}