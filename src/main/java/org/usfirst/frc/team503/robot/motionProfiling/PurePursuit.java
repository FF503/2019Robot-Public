package org.usfirst.frc.team503.robot.motionProfiling;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.Drive.DriveMotorOutput;
import org.usfirst.frc.team503.robot.utils.FFDashboard;
import org.usfirst.frc.team503.robot.utils.Pose;
import org.usfirst.frc.team503.robot.utils.Vector2D;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import motionProfiling.Trajectory;
import motionProfiling.Trajectory.Segment;

public class PurePursuit {
    private Trajectory traj;
    private double lookAheadDist;
    private double minLookaheadDistance;
    private int runCount = 0;
    private double curAngle;
    private double trackwidth;
    private double curvature;
    private double currentTargetVelocity;
    private int theoreticalSegmentIndex = 0, lookAheadIndex = 0, closestSegmentIndex = 0;
    private boolean isReversed = false;
    private Vector2D lookAheadPoint;
    private static Double lastTime = Timer.getFPGATimestamp();
    private Pose pose;
    private static Pose lastPose = new Pose(0, 0, 0.0);

    FFDashboard graphTable = new FFDashboard("Graph");
    FFDashboard purePursuitTable = new FFDashboard("PurePursuit");

    public PurePursuit(Trajectory traj, double lookAheadDist, double trackwidth) {
        this.traj = traj;
        purePursuitTable.putNumber("Total Segments", traj.getNumSegments());
        this.minLookaheadDistance = lookAheadDist;
        this.lookAheadDist = lookAheadDist;
        purePursuitTable.putNumber("LookAheadDistance", lookAheadDist);
        this.trackwidth = trackwidth;
    }

    public PurePursuit(Vector2D fixedLookAhead, double trackwidth) {
        this.lookAheadPoint = fixedLookAhead;
        this.trackwidth = trackwidth;
    }

    public DriveMotorOutput calculateVelocitesFixedLookAhead(Pose relativeRobotPose, double vel) {
        DriveMotorOutput output;
        this.pose = relativeRobotPose;
        this.curAngle = relativeRobotPose.getTheta();
        this.curvature = getCurvatureFixedLookAhead(lookAheadPoint, relativeRobotPose);
        System.out.println("curv:" + curvature);
        output = getWheelOutput(curvature, Robot.bot.kV_PurePursuit * vel, 0);
        lastPose = relativeRobotPose;
        graphTable.putNumber("lookaheadPointX", lookAheadPoint.x());
        graphTable.putNumber("lookaheadPointY", lookAheadPoint.y());
        return output;
    }

    public DriveMotorOutput calculateVelocities(Pose robotPose) {
        DriveMotorOutput output;
        this.pose = robotPose;
        this.curAngle = robotPose.getTheta();
        // getClosestSegment();
        this.lookAheadDist = getSpeedBasedLookahead(getClosestSegment());
        SmartDashboard.putNumber("Lookahead Distance", lookAheadDist);
        this.lookAheadPoint = getLookAhead(robotPose);
        this.curvature = getCurvature(lookAheadPoint, robotPose);
        output = getWheelOutput(curvature, getVelocityBasedPowerCalculation(), getAccelerationBasedPowerCalculation());
        theoreticalSegmentIndex++;
        // lastTime = Timer.getFPGATimestamp();
        lastPose = robotPose;
        return output;
    }

    private Vector2D getLookAhead(Pose robotPose) {
        double tValue = 0;
        Vector2D startSeg = new Vector2D(traj.getSegment(lookAheadIndex).x, traj.getSegment(lookAheadIndex).y);
        Vector2D dSegmentVector = new Vector2D(0, 0);
        for (int i = closestSegmentIndex; i < traj.getNumSegments() - 1; i++) {
            Vector2D curPathSeg = new Vector2D(traj.getSegment(i).x, traj.getSegment(i).y);
            Vector2D nextPathSeg = new Vector2D(traj.getSegment(i + 1).x, traj.getSegment(i + 1).y);

            Vector2D curToNextSegment = new Vector2D(nextPathSeg.x() - curPathSeg.x(),
                    nextPathSeg.y() - curPathSeg.y());
            Vector2D f = new Vector2D(curPathSeg.x() - robotPose.getX(), curPathSeg.y() - robotPose.getY());

            double a = curToNextSegment.dot(curToNextSegment);
            double b = 2 * f.dot(curToNextSegment);
            double c = f.dot(f) - lookAheadDist * lookAheadDist;
            double discriminant = b * b - 4 * a * c;

            if (discriminant >= 0) {
                discriminant = Math.sqrt(discriminant);

                double t1 = (-b - discriminant) / (2 * a);
                double t2 = (-b + discriminant) / (2 * a);

                if (t1 >= 0 && t1 <= 1) {
                    tValue = t1;
                    startSeg = curPathSeg.copy();
                    dSegmentVector = curToNextSegment.copy();
                    lookAheadIndex = i;
                    break;
                } else if (t2 >= 0 && t2 <= 1) {
                    tValue = t2;
                    startSeg = curPathSeg.copy();
                    dSegmentVector = curToNextSegment.copy();
                    lookAheadIndex = i;
                    break;
                }
                // otherwise, no intersection
            }
        }
        Vector2D lookAhead = startSeg.copy();
        dSegmentVector.scale(tValue);
        lookAhead.translateUp(dSegmentVector);

        Segment finalSegment = traj.getSegment(traj.getNumSegments() - 1);
        if (lookAhead.distanceTo(finalSegment) < lookAheadDist) {
            // lookAhead = new Vector2D(finalSegment.x, finalSegment.y);
            // runCount++;
        } else {
            runCount = 0;
        }

        if (Drive.getInstance().getRobotVelocity() <= 3.0 && lookAhead.distanceTo(finalSegment) <= 50.0) {
            runCount++;
        } else {
            runCount--;
        }
        // Output Dashboard Data
        purePursuitTable.putNumber("Closest Segment Index", closestSegmentIndex);
        purePursuitTable.putNumber("runCount", runCount);
        graphTable.putNumber("lookaheadPointX", lookAhead.x());
        graphTable.putNumber("lookaheadPointY", lookAhead.y());
        return lookAhead;
    }

    public Segment getClosestSegment() {
        Segment closestSeg = traj.getSegment(closestSegmentIndex);
        for (int i = closestSegmentIndex; i < this.traj.getNumSegments(); i++) {
            Segment curSegment = traj.getSegment(i);
            double poseToPrevHypot = Math.hypot(this.pose.getX() - closestSeg.x, this.pose.getY() - closestSeg.y);
            double poseToCurrHypot = Math.hypot(this.pose.getX() - curSegment.x, this.pose.getY() - curSegment.y);
            if (poseToCurrHypot < poseToPrevHypot) {
                closestSegmentIndex = i;
                closestSeg = curSegment;
            }
        }
        return closestSeg;
    }

    public Vector2D getCalculatedLookAhead() {
        return lookAheadPoint;
    }

    public double getCalculatedCurvatures() {
        return curvature;
    }

    public double getCurvature(Vector2D lookAheadPoint, Pose robotPose) {
        double localcurvature = 0;// getSegment().curvature;
        double Lx = lookAheadPoint.x();
        double Ly = lookAheadPoint.y();
        double Rx = robotPose.getX();
        double Ry = robotPose.getY();
        double Rtheta = Math.toRadians(robotPose.getTheta());

        double a = -Math.tan(Rtheta);
        double b = 1;
        double c = Math.tan(Rtheta) * Rx - Ry;

        double x = Math.abs(a * Lx + b * Ly + c) / Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
        double side = Math.signum(Math.sin(Rtheta) * (Lx - Rx) - Math.cos(Rtheta) * (Ly - Ry));

        localcurvature = side * (2 * x) / (Math.pow(lookAheadDist, 2));

        // Output Curvature
        purePursuitTable.putNumber("Curvature", localcurvature);

        System.out.println("Curvature " + localcurvature);
        return localcurvature;
    }

    public double getCurvatureFixedLookAhead(Vector2D lookAheadPoint, Pose robotPose) {
        double localcurvature = 0;// getSegment().curvature;
        double Lx = lookAheadPoint.x();
        double Ly = lookAheadPoint.y();
        double Rx = robotPose.getX();
        double Ry = robotPose.getY();
        double Rtheta = Math.toRadians(robotPose.getTheta());

        double a = -Math.tan(Rtheta);
        double b = 1;
        double c = Math.tan(Rtheta) * Rx - Ry;

        double x = Math.abs(a * Lx + b * Ly + c) / Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
        double side = Math.signum(Math.sin(Rtheta) * (Lx - Rx) - Math.cos(Rtheta) * (Ly - Ry));

        localcurvature = side * (2 * x) / (Math.pow(lookAheadPoint.distanceTo(pose.toVector()), 2));

        // Output Curvature
        purePursuitTable.putNumber("Curvature", localcurvature);

        return localcurvature;
    }

    public int getFinalRunCount() {
        return runCount;
    }

    public void resetRunCount() {
        runCount = 0;
    }

    public DriveMotorOutput getCurrentTargetVelocityParameterized() {
        DriveMotorOutput ret = new DriveMotorOutput();
        // if (isReversed){
        // currentTargetVelocity*=-1;
        // }
        double l = currentTargetVelocity * (2 + (curvature) * trackwidth) / 2;
        double r = currentTargetVelocity * (2 - (curvature) * trackwidth) / 2;
        ret.setPercentPowers(l, r);
        return ret;
    }

    public DriveMotorOutput getWheelOutput(double curvature, double targetVel, double targetAccel) {
        DriveMotorOutput outputs = new DriveMotorOutput();
        double[] velocities = getWheelVelocities(curvature, targetVel);
        double[] accel = getWheelAccels(curvature, targetAccel);
        outputs.setAccelerations(accel[0], accel[1]);
        outputs.setVelocity(velocities[0], velocities[1]);

        purePursuitTable.putNumber("Target Velocity", getSegment().vel);
        purePursuitTable.putNumber("Target Acceleration", getSegment().acc);
        return outputs;
    }

    public double[] getWheelVelocities(double curvature, double targetVel) {
        double fudge = Robot.bot.curvatureFudge;
        // if (runCount>0){
        // fudge = 1.0;
        // }
        double mult = 1.0;
        // if (isReversed) {
        // mult = -1.0;
        // fudge = Robot.bot.curvatureFudgeRev;
        // }
        double l = mult * targetVel * (2 + (curvature * fudge) * trackwidth) / 2;
        double r = mult * targetVel * (2 - (curvature * fudge) * trackwidth) / 2;
        // Output Wheel Velocities
        System.out.println("l:" + l);
        System.out.println("r:" + r);
        purePursuitTable.putNumber("Left Velocity", l);
        purePursuitTable.putNumber("Right Velocity", r);
        return new double[] { l, r };
    }

    public double[] getWheelAccels(double curvature, double targetAccel) {
        double mult = 1.0;
        // if (isReversed){
        // mult = -1.0;
        // }
        double l = mult * targetAccel * (2 + curvature * trackwidth) / 2;
        double r = mult * targetAccel * (2 - curvature * trackwidth) / 2;
        System.out.println("l a:" + l);
        System.out.println("r a" + r);
        // Output Wheel Acceleration
        purePursuitTable.putNumber("Left Acceleration", l);
        purePursuitTable.putNumber("Right Acceleration", r);
        return new double[] { l, r };
    }

    private double lastError = 0.0;

    public double getVelocityBasedPowerCalculation() {
        double targetVel = getSegment().vel;
        currentTargetVelocity = targetVel;
        double distanceTraveled = pose.toVector().distanceTo(lastPose.toVector());
        double currentTime = Timer.getFPGATimestamp();
        double velocityMagnitude = distanceTraveled / (currentTime - lastTime);
        double velError = targetVel - velocityMagnitude;
        double errorDerivative = (velError - lastError) / (currentTime - lastTime);
        lastError = velError;
        lastTime = currentTime;
        double kv;
        // if (isReversed){
        // kv = Robot.bot.kV_PurePursuitReverse;
        // System.out.println("using reversed");
        // }
        // else{
        kv = Robot.bot.kV_PurePursuit;
        // }
        return (kv * targetVel) /*+(Robot.bot.kP_PurePursuit * velError) + (Robot.bot.kD_PurePursuit * errorDerivative)*/;
    }

    public void setIsReversed(boolean rev) {
        isReversed = rev;
    }

    public double getAccelerationBasedPowerCalculation() {
        return getSegment().acc * Robot.bot.kA_PurePursuit;
    }

    public Segment getSegment() {
        return traj.getSegment(getSegmentIndex());
    }

    public int getSegmentIndex() {
        return closestSegmentIndex;
    }

    public int getTheoreticalSegmentIndex() {
        return theoreticalSegmentIndex;
    }

    public double getSpeedBasedLookahead(Segment seg) {
        return getSpeedBasedLookahead(seg.vel);
    }

    public double getSpeedBasedLookahead(double speed) {
        // Change Parameters below as nessesary
        double desiredMaxLookahead = isReversed ? 70 : 40;
        final double minLookaheadDistance = this.minLookaheadDistance, maxLookaheadDistance = desiredMaxLookahead;
        final double min_speed = 0, max_speed = Robot.bot.kMaxVelocityInchesPerSec;
        /**********************************************************/

        final double delta_distance = maxLookaheadDistance - minLookaheadDistance;
        final double delta_speed = max_speed - min_speed;
        double lookahead = delta_distance * (speed - min_speed) / delta_speed + minLookaheadDistance;
        return Double.isNaN(lookahead) ? minLookaheadDistance
                : Math.max(minLookaheadDistance, Math.min(maxLookaheadDistance, lookahead));
    }

}