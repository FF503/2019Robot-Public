/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.RobotState.LedColors;
import org.usfirst.frc.team503.robot.subsystems.LED;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class LedSetCommand extends Command {

  private LedColors mLedColor, mStateOn, mStateOff;
  private double mPatternTime = 0.10, mLastSetTime = 0.;
  private int mFrequency, mIdx = 0;
  private boolean seqFinished = false;


  // This consrtuctor is just for killing all other possible sequences
  public LedSetCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(LED.getInstance());
    mLedColor = RobotState.getInstance().getLedColor();
  }

  public LedSetCommand(LedColors color) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(LED.getInstance());
    this.mLedColor = color;
  }

  public LedSetCommand(LedColors stateOn, LedColors stateOff, int frequency) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.mStateOn = stateOn;
    this.mStateOff = stateOff;
    requires(LED.getInstance());
    this.mFrequency = frequency;
  }

  public LedSetCommand(LedColors stateOn, LedColors stateOff, double patternTime, int frequency) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(LED.getInstance());
    this.mStateOn = stateOn;
    this.mStateOff = stateOff;
    this.mPatternTime = patternTime;
    this.mFrequency = frequency;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (mStateOn != null) {
      mLedColor = mStateOff;
    } else {
      seqFinished = true;
    }

    RobotState.getInstance().setLedColor(mLedColor);
    mLastSetTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!seqFinished && Timer.getFPGATimestamp() - mLastSetTime >= mPatternTime) {
      // mLedColor = (mLedColor == mStateOff) ? mStateOn : mStateOff;
      if (mLedColor == mStateOff) {
        mLedColor = mStateOn;
        mIdx++;
      } else {
        mLedColor = mStateOff;
      }
      mLastSetTime = Timer.getFPGATimestamp();

    }
    RobotState.getInstance().setLedColor(mLedColor);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return seqFinished || (mIdx == mFrequency && mFrequency != 0);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
