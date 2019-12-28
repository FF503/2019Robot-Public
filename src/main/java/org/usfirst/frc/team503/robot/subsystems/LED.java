/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.subsystems;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.RobotState.LedColors;
import org.usfirst.frc.team503.robot.utils.FFDashboard;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Subsystem for LED condition changing
 */
public class LED extends Subsystem {

  private static Spark LED;
  private FFDashboard lead = new FFDashboard("LEDs");

  public LED() {
    LED = new Spark(Robot.bot.LED_ID);
    lead.putNumber("Power", 0.0);
    lead.getEntry("Power").addListener(event -> {
      set(event.getEntry().getDouble(0.0));
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }

  private static LED instance = new LED();

  public static LED getInstance() {
    return instance;
  }

  public void set(double power) {
    if (Robot.bot.hasLEDs()) {
      LED.set(power);
    }
  }

  public void set(RobotState.LedColors ledColors) {
    set(ledColors.getPower());
  }

  public void setLEDOff() {
    set(LedColors.BLACK);
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

}
