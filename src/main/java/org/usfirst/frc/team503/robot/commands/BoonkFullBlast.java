/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class BoonkFullBlast extends Command {
  double timeOut, startTime;
  boolean arc;
  boolean reverse;
  public BoonkFullBlast(double timeOut, boolean reverse, boolean arc) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    //requires(Drive.getInstance());
    this.timeOut = timeOut;
    this.arc = arc;
    this.reverse = reverse;
    startTime = Timer.getFPGATimestamp();
  }

  public BoonkFullBlast(boolean reverse) {
    this.timeOut = 0.0;
    this.arc = false;
    this.reverse = reverse;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    double arcPower = 0.2;
    double fullPower = 1.0;
    if (reverse){
      arcPower *= -1;
      fullPower *= -1;
    }
    if (arc){
      Drive.getInstance().setMotorOutputs(arcPower, fullPower);
    }
    else{
      Drive.getInstance().setMotorOutputs(fullPower, fullPower);
    }
   
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println("running");
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;//Timer.getFPGATimestamp() - startTime > timeOut; 
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
}
