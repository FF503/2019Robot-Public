package org.usfirst.frc.team503.robot.motionProfiling;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.Drive.DriveMotorOutput;
import org.usfirst.frc.team503.robot.utils.DataEntry;
import org.usfirst.frc.team503.robot.utils.Pose;
import org.usfirst.frc.team503.robot.utils.Vector2D;

import motionProfiling.Trajectory;

/**
 *
 */
public class FFPureEncoderFollower {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    int encoder_offset, encoder_tick_count;
    double wheel_circumference;
    boolean isReversed;
    int resetPoint;
    double resetError;
    Pose pose = RobotState.getInstance().getPoseOdometry();

    double last_error, heading;
    PurePursuit engine;

    int segmentIndex = 0;
    Trajectory trajectory;
    DataEntry error = new DataEntry("Error");
    DataEntry power = new DataEntry("Power");
    DataEntry time = new DataEntry("Time");

    public FFPureEncoderFollower(Trajectory trajectory2, boolean isReversed, double lookAheadDistance) {
        this.trajectory = trajectory2;
        this.isReversed = isReversed;
        engine = new PurePursuit(trajectory, lookAheadDistance, Robot.bot.TRACK_WIDTH);
        engine.setIsReversed(isReversed);
        reset();
    }

    /**
     * Set a new trajectory to follow, and reset the cumulative errors and segment
     * counts
     */
    public void setTrajectory(Trajectory traj) {
        this.trajectory = traj;
        reset();
    }

    public void configureEncoder(int initial_position, int ticks_per_revolution, double wheel_diameter) {
        encoder_offset = initial_position;
        encoder_tick_count = ticks_per_revolution;
        wheel_circumference = Math.PI * wheel_diameter;
    }

    /**
     * Reset the follower to start again. Encoders must be reconfigured.
     */
    public void reset() {
        last_error = 0;
        segmentIndex = 0;
        pose = RobotState.getInstance().getPoseOdometry();
        engine.resetRunCount();

    }

    public DriveMotorOutput calculate(Pose robotPose) {
        this.pose = robotPose;
        DriveMotorOutput temp = engine.calculateVelocities(robotPose);
        segmentIndex = engine.getSegmentIndex();
        return temp;
    }

    public DriveMotorOutput calculatePureParameterizedDesiredVelocities() {
        return engine.getCurrentTargetVelocityParameterized();
    }

    /**
     * @return the desired heading of the current point in the trajectory
     */
    public double getHeading() {
        return heading;
    }

    /**
     * @return the current segment being operated on
     */
    public Trajectory.Segment getSegment() {
        return engine.getSegment();
    }

    public Vector2D getLookAheadPose() {
        return engine.getCalculatedLookAhead();
    }

    public double getCalculatedCurvature() {
        return engine.getCalculatedCurvatures();
    }

    /**
     * @return whether we have finished tracking this trajectory or not.
     */
    public boolean isFinished(boolean useCamera) {
        pose = RobotState.getInstance().getPoseOdometry();
        double dist = Math.hypot(pose.getX() - trajectory.getSegment(trajectory.getNumSegments() - 1).x,
                pose.getY() - trajectory.getSegment(trajectory.getNumSegments() - 1).y);
        System.out.println("x: " + trajectory.getSegment(trajectory.getNumSegments() - 1).x);
        System.out.println("y: " + trajectory.getSegment(trajectory.getNumSegments() - 1).y);
        System.out.println("rx:" + pose.getX());
        System.out.println("ry:" + pose.getY());
        System.out.println("run count:" + engine.getFinalRunCount());
        double tolerance = Robot.bot.PurePursuit_TOLERANCE;
        if (useCamera) {
            tolerance = 2.0;
        }
        int segmentTolerance = 5;
        return (dist < tolerance) || (trajectory.getNumSegments() - engine.getSegmentIndex()) <= segmentTolerance;
        // return segmentIndex >= trajectory.getNumSegments();
    }

    public void setResetPoint(int reset) {
        resetPoint = reset;
    }

    public boolean velocityZero() {
        return (Math.abs(Drive.getInstance().getRightVelocityInches()) < Robot.bot.VELOCITY_ZERO_TOLERANCE
                && Math.abs(Drive.getInstance().getRightVelocityInches()) < Robot.bot.VELOCITY_ZERO_TOLERANCE
                && (engine.getSegmentIndex() * 1.0 / trajectory.getNumSegments()  > 0.70));
    }
}
