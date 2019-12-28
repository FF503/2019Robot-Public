package org.usfirst.frc.team503.robot.vision.commands;

import org.usfirst.frc.team503.robot.OI;
import org.usfirst.frc.team503.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveForwardTimePlace extends Command {
    double power, time;
    double start;

    boolean desiredButton;

    public DriveForwardTimePlace(double power, double time) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        this.power = power;
        this.time = time;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        start = Timer.getFPGATimestamp();
        Drive.getInstance().tankDrive(power, power);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        System.out.println("setting power of: " + power);
        Drive.getInstance().tankDrive(power, power);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        // if (!RobotState.getInstance().getClimbState()) {
        //     return ((Timer.getFPGATimestamp() - start) > time) || !OI.getClimbButton();
        // } else {
            System.out.println("Finished");
            return (Timer.getFPGATimestamp() - start) > time;
        // }
    }

    // Called once after isFinished returns true
    protected void end() {
        Drive.getInstance().zeroPowerMotors();
        System.out.println("Ended erive");
        // new AssistedArcadeDriveCommand().start();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
