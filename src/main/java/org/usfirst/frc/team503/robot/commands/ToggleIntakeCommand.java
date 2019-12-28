/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.OI;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.RobotState.ArmDirection;
import org.usfirst.frc.team503.robot.RobotState.GameElement;
import org.usfirst.frc.team503.robot.RobotState.LedColors;
import org.usfirst.frc.team503.robot.RobotState.TargetHeight;
import org.usfirst.frc.team503.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class ToggleIntakeCommand extends Command {

  private double startTime;
  private boolean hasCargo;
  private boolean irFlag = true, currentFlag = true;
  private double irStartTime, currentStartTime;

  public ToggleIntakeCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // requires(Intake.getInstance());

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // if (!RobotState.getInstance().getIntakeRunning()) {
    // gameElement = RobotState.getInstance().getGameElement();
    startTime = Timer.getFPGATimestamp();
    // // switch (gameElement) {

    // // case CARGO:
    // // Intake.getInstance().intakeCargo();
    // // break;
    // // case HATCH_R:
    // // Intake.getInstance().intakeHatch();
    // // break;
    // // default:

    // // Intake.getInstance().stopIntake();
    // // }
    // Intake.getInstance().intakeCargo();
    // RobotState.getInstance().setIntakeRunning(true);
    // } else {
    // Intake.getInstance().stopIn take();
    // RobotState.getInstance().setIntakeRunning(false);
    // finish = true;
    // }
    Intake.getInstance().intakeCargo();
    RobotState.getInstance().setHatchDependence(false);
    new MoveArmCommand(ArmDirection.FRONT, GameElement.CARGO, TargetHeight.INTAKE).start();
    new LedSetCommand(LedColors.GREEN).start();
    // Intake.getInstance().stopVacuum();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Intake.getInstance().hasCargo() && irFlag){
      irFlag = false;
      irStartTime = Timer.getFPGATimestamp();
    }
    else if(!Intake.getInstance().hasCargo()){
      irFlag = true;
    }

    if (Intake.getInstance().hasCargoOld() && currentFlag){
      currentFlag = false;
      currentStartTime = Timer.getFPGATimestamp();
    }
    else if  (!Intake.getInstance().hasCargoOld()){
      currentFlag = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    hasCargo = (Timer.getFPGATimestamp() - startTime > 0.75/* && hasCargoForSomeTime()*/
        && Intake.getInstance().hasCargoOld()) || OI.getRunOuttake();
    return hasCargo;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Intake.getInstance().stopIntake();
    // Intake.getInstance().startVacuum();
    if (hasCargo) {
      RobotState.getInstance().setHasElement(true);
      new MoveArmCommand(ArmDirection.FRONT, TargetHeight.HOME).start();
      new LedIntakeSequenceCommand().start();

    } else {
      new LedSetCommand(LedColors.BLACK).start();

    }
  }

  public boolean hasCargoForSomeTime(){
    return (!irFlag && Timer.getFPGATimestamp() - irStartTime > 0.1); 
  }

  public boolean hasCargoForSomeTimeOld(){
    return (!currentFlag && Timer.getFPGATimestamp() - currentStartTime > 0.1); 
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
