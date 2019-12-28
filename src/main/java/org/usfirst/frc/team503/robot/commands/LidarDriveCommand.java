/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.command.Command;

public class LidarDriveCommand extends Command {
  private int angle;
  private double targetDistance;
  private double tolerance;
  private double curDistance;
  private final double kLidarDriveP;
  private final double isReversed;
  // private SynchronousPID pidControlller;

  public LidarDriveCommand(int angle, double targetDistance, double tolerance, boolean isReversed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.angle = angle;
    this.tolerance = tolerance;
    this.targetDistance = targetDistance;
    this.kLidarDriveP = Robot.bot.kLidarDriveP;
    this.isReversed = isReversed ? -1.0 : 1.0;
    // pidControlller = new SynchronousPID(Robot.bot.kLidarDriveP, 0.0, 0.0);
  }

  public LidarDriveCommand(int angle, double targetDistance, boolean isReversed) {
    this(angle, targetDistance, Robot.bot.lidarTolerance, isReversed);
  }


  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // pidControlller.setOutputRange(Robot.bot.kMinOutput, Robot.bot.kMaxOutput);
    // pidControlller.setSetpoint(targetDistance);
    curDistance = RobotState.getInstance().getLidarDistance(angle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    curDistance = RobotState.getInstance().getLidarDistance(angle);
    // double power = pidControlller.calculate(curDistance);
    double error = getError(targetDistance, curDistance);
    double power = kLidarDriveP * error;
    power *= -isReversed;
    
    // System.out.println("Power " + power);
    // System.out.println("LIDAR Pid Error: " + error);
    
    Drive.getInstance().tankDrive(power, power);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    if(RobotState.getInstance().getState().equals(RobotState.State.TELEOP)) {
			if(!RobotState.getInstance().isVisionFollowerRunning()) {
				return true;
			}
		}  
    return Math.abs(targetDistance - curDistance) < tolerance;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Drive.getInstance().zeroPowerMotors();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  private double getError(double target, double current) {
    return target - current;
  }

}
