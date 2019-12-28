/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.utils.FFDashboard;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem for cargo intake and hatch intake
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private Spark hatchVac;
  private Spark cargoIntake;
  private Spark hatchVac2;
  private Solenoid releaser;
  private Solenoid releaser2;
  private FFDashboard table = new FFDashboard("Intake");
  private double cCur, lCur = 0.0;
  private double vcCur, vlCur = 0.0;
  private AnalogInput infSensor;
  private AnalogInput pressureSensor;

  private PowerDistributionPanel pdp;
  private static Intake instance = new Intake();

  public Intake() {
    if (Robot.bot.hasIntake()) {
      hatchVac = new Spark(Robot.bot.hatchVacId);
      hatchVac2 = new Spark(Robot.bot.hatchVacId2);
      cargoIntake = new Spark(Robot.bot.rollerIntakeID);
      infSensor = new AnalogInput(Robot.bot.beamBreakID);
      pressureSensor = new AnalogInput(Robot.bot.pressureSensorID);
      pdp = new PowerDistributionPanel(Robot.bot.PDP_ID);
      releaser = new Solenoid(Robot.bot.releaseId);
      releaser2 = new Solenoid(Robot.bot.release2Id);
      // startVacuum();
    }
  }

  public static Intake getInstance() {
    return instance;
  }

  public void setMotorPower(double power) {
    cargoIntake.set(power);
  }

  @Deprecated
  public double getOutputCurrent() {
    if (Robot.bot.hasIntake()) {
      return pdp.getCurrent(Robot.bot.intakePdpChannel);
    } else {
      return 0.0;
    }
  }

  public double getVacuumCurrent() {
     return pdp.getCurrent(Robot.bot.vacuumPdpChannel);
    //return hatchVac.getOutputCurrent();
  }

  public double getVacuumCurrentD() {
    vcCur = getVacuumCurrent();
    double derivative = (vcCur - vlCur) / .02;
    vlCur = vcCur;
    return derivative;
  }

  public double getChannelCurrent(int pdpChannel) {
    return pdp.getCurrent(pdpChannel);
  }

  public double getDeltaCurrent() {
    cCur = getOutputCurrent();
    double delta = cCur - lCur;
    lCur = cCur;
    return delta;
  }

  public boolean hasCargoOld() {
    return getOutputCurrent() > Robot.bot.rollerCurrentThres;
  }

  public boolean hasCargo() {
    return getSensorVoltage() > Robot.bot.sensorVoltageThres;
  }

  public boolean hasHatch() {
    return getPressureGauge() > Robot.bot.pressureThres;
  }

  public double getMotorPower() {
    if (Robot.bot.hasIntake()) {
      return hatchVac.get();
    } else {
      return 0.0;
    }
  }

  public boolean isCargo() {
    return (getMotorPower() > 0.01);
  }

  public boolean isHatch() {
    return (getMotorPower() < 0.01);
  }

  public boolean getIntakeRunning() {
    return Math.abs(getMotorPower()) > 0.0;
  }

  public void intakeCargo() {
    setMotorPower(Robot.bot.intakeBasePower);
  }

  @Deprecated
  public void intakeHatch() {
    setMotorPower(-1.0);
  }

  public void outtakeCargo() {
    setMotorPower(Robot.bot.intakeOutPower);
  }

  // public double getHatchCurrent() {
  // return hatchVac.get
  // }

  public void outtakeHatch() {
    setMotorPower(1.0);
  }

  public void stopIntake() {
    setMotorPower(Robot.bot.intakeStallPower);
  }

  public double getSensorVoltage() {
    return infSensor.getVoltage();
  }

  public double getPressureGauge() {
    return pressureSensor.getVoltage();
  }
  // public void grabHatch() {
  // RobotState.getInstance().setGrabberDeployed(true);
  // }

  // public void setVacuumOutput(boolean state) {
  // hatchVac.set(state ? 1.0 : 0.0);
  // }

  // public void release(boolean state) {
  // releaser.set(state);
  // }

  public void startVacuum() {
    hatchVac.set(Robot.bot.intakeVaccPower);
    hatchVac2.set(Robot.bot.intakeVaccPower);
  }

  public void setVacuumPower(double power) {
    hatchVac.set(power);
    hatchVac2.set(power);
  }

  public void stopVacuum() {
    hatchVac.set(0.0);
    hatchVac2.set(0.0);
  }

  public void releaseHatch() {
    releaser.set(true);
    releaser2.set(true);
  }

  public void closeReleaseValve() {
    releaser.set(false);
    releaser2.set(false);
  }

  public void sendDashboardData() {
    table.putBoolean("Intaking running", getIntakeRunning());
    table.putBoolean("Has Hatch", hasHatch());
    table.putBoolean("Has Cargo", hasCargo());
    SmartDashboard.putBoolean("Has Hatch", hasHatch());
    table.putNumber("Motor Output Power", getMotorPower());
    table.putNumber("Infrared Sensor Voltage", getSensorVoltage());
    SmartDashboard.putNumber("Infrared Sensor Voltage", getSensorVoltage());
    table.putNumber("Vacuum Current", getVacuumCurrent());
    SmartDashboard.putNumber("Vacuum Current", getVacuumCurrent());
    table.putNumber("Vacuum Current Derivative", getVacuumCurrentD());
    SmartDashboard.putNumber("Vacuum Current Derivative", getVacuumCurrentD());
    table.putNumber("Vacuum Voltage", 0.0);
    SmartDashboard.putNumber("Vacuum Pressure Value", getPressureGauge());
    table.putNumber("Vacuum Pressure Value", getPressureGauge());
    table.putNumber("Intake Motor Current", getOutputCurrent());
    table.putNumber("Vacuum Motor Current", getVacuumCurrent());

    SmartDashboard.putNumber("Intake Motor Current", getOutputCurrent());
    SmartDashboard.putNumber("Vacuum Motor Current", getVacuumCurrent());

    for (int i = 0; i <= 15; i++) {
      table.putNumber("Motor " + i + " Current", getChannelCurrent(i));
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
