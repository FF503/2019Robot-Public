/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.utils.FFDashboard;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private CANSparkMax rightHand, leftHand, leg;
  private CANEncoder  elevEncoder;
  private FFDashboard table = new FFDashboard("Climber");

  private double integral, derivative, currError, lastError, currTime, lastTime = 0.0;
  

  public static Climber instance = new Climber();

  public static Climber getInstance() {
    return instance;
  }

  public Climber() {
    if (!Robot.bot.getName().equals("ProgrammingBot")) {
      rightHand = new CANSparkMax(Robot.bot.rightHandID, MotorType.kBrushless);
      leftHand = new CANSparkMax(Robot.bot.leftHandID, MotorType.kBrushless);
      leg = new CANSparkMax(Robot.bot.legID, MotorType.kBrushless);
      leftHand.follow(rightHand);
      elevEncoder = leg.getEncoder();
      elevEncoder.setPosition(0.0);
      //setBrake();

    }
  }

  public void runLegMotor(double power) {
    if(Robot.bot.hasClimber()) {
      leg.set(power);
      //System.out.println("Leg id" + leg.getDeviceId());
    }
  }

  public void runHandMotors(double power) {
    if(Robot.bot.hasClimber()) {
      rightHand.set(power);
      // System.out.println("Right Hand id" + rightHand.getDeviceId());
      // System.out.println("leg power: " + leg.get());
      // System.out.println("leg follow: " + leg.isFollower());
    }
  }

  public void stopMotors() {
    if(Robot.bot.hasClimber()) {
      runLegMotor(0.0);
      runHandMotors(0.0);
    }
  }

  public double getLegCounts() {
    if(Robot.bot.hasClimber()) {
      return leg.get();
    } else {
      return 0;
    }
  }

  public double getHandCounts() {
    if(Robot.bot.hasClimber()) {
      return rightHand.get();
    } else {
      return 0;
    }

  }

  public void sendDashboardData() {
    table.putNumber("Leg Pwr", getLegCounts());
    table.putNumber("Hand Pwr", getHandCounts());
  }

  public void setBrake() {
    leg.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast() {
    leg.setIdleMode(IdleMode.kCoast);
  }

  public double getCalculatedPitchGain(double tgt) {
    double error = tgt - Gyro.getInstance().getPitch();
    currError = error;
    currTime = Timer.getFPGATimestamp();
    derivative = (currError - lastError)/(currTime-lastTime);
    integral += currError / (currTime - lastTime);

    lastTime = currTime;
    lastError = currError;
    System.out.println("gyro pitch = " + Gyro.getInstance().getPitch());

    return minLim(Robot.bot.climbPitchP * error + Robot.bot.climbPitchI * integral + Robot.bot.climbPitchD * derivative, Robot.bot.climbHandLim);
  }

  public double getCalculatedElevGain(double tgt) {
    double error = tgt - Gyro.getInstance().getPitch();//tgt - elevEncoder.getPosition();
    currError = error;
    currTime = Timer.getFPGATimestamp();
    derivative = (currError - lastError)/(currTime-lastTime);
    integral += currError / (currTime - lastTime);

    lastTime = currTime;
    lastError = currError;

    return minLim(-(Robot.bot.climbElevP * error + Robot.bot.climbElevI * integral + error * Robot.bot.climbElevD), Robot.bot.climbLegLim);
  }

  private double minLim(double value, double limit) {
    return value < limit ? limit : value;
  }

  public double getElevPos() {
    return elevEncoder.getPosition();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

}
