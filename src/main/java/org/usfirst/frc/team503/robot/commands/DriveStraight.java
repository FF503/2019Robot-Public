package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.utils.SynchronousPID;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This command uses encoders to drive the robot forward a certain distance
 */
public class DriveStraight extends Command {
    SynchronousPID straightPID = new SynchronousPID();
    double distanceForDrive;
    double finalPt[];
    double initLeftDist, initRightDist;
    double[] pidValues, outputRange;
    double tolerance;

    public DriveStraight() {
        pidValues = Robot.bot.getDSPID();
        outputRange = Robot.bot.getDSOutputRange();
        tolerance = Robot.bot.getDSTolerance();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        // System.out.println("Drive straight initialized");
        initLeftDist = Drive.getInstance().getLeftDistanceInches();
        initRightDist = Drive.getInstance().getRightDistanceInches();
        // System.out.println("Init left distance: " + initLeftDist);
        // System.out.println("Init right distance: " + initRightDist);

        // These were working before on the programming bot
        // straightPID.setPID(.035, 0, 0.015);
        // straightPID.setOutputRange(-0.6, 0.6);
        straightPID.setPID(pidValues[0], pidValues[1], pidValues[2]);
        straightPID.setOutputRange(outputRange[0], outputRange[1]);

        double angle = RobotState.getInstance().getTargetData().getAdjustedAngle();
        // System.out.println("Angle: " + angle);

        double[] initPt = VisionLocalizer.getInstance()
                .translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles());

        // These were working on the programming bot
        // if (initPt[0] < 0) {
        // finalPt = new double[] { 6.5, 27 }; // 27
        // } else {
        // finalPt = new double[] { -6.5, 27 }; // 27
        // }
        finalPt = Robot.bot.getDSSetpoint(initPt[0]);

        // System.out.println("DriveStraight setpoint: (" + finalPt[0] + ", " + finalPt[1] + ")");

        distanceForDrive = Math.hypot((finalPt[0] - initPt[0]), (finalPt[1] - initPt[1]));
        // System.out.println("Attempting to drive: " + distanceForDrive);

        SmartDashboard.putNumber("distance to drive", distanceForDrive);

        straightPID.setSetpoint(distanceForDrive);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double distanceTraveled = ((Drive.getInstance().getLeftDistanceInches() - initLeftDist)
                + (Drive.getInstance().getRightDistanceInches() - initRightDist)) / 2;

        // System.out.println("Traveled: " + distanceTraveled);
        if (!straightPID.onTarget(tolerance)) {
            double power = straightPID.calculate(distanceTraveled);
            // System.out.println("Power: " + power);
            Drive.getInstance().tankDrive(power, power);
        } else {
            Drive.getInstance().zeroPowerMotors();
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return straightPID.onTarget(tolerance);
    }

    // Called once after isFinished returns true
    protected void end() {
        Drive.getInstance().zeroPowerMotors();
        double distanceTraveled = ((Drive.getInstance().getLeftDistanceInches() - initLeftDist)
                + (Drive.getInstance().getRightDistanceInches() - initRightDist)) / 2;

        // System.out.println("Traveled: " + distanceTraveled);
        // System.out.println("On target with tolerance of " + tolerance);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
