
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.OI;
import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.RobotState.ArmDirection;
import org.usfirst.frc.team503.robot.RobotState.GameElement;
import org.usfirst.frc.team503.robot.RobotState.State;
import org.usfirst.frc.team503.robot.RobotState.SuperStructurePreset;
import org.usfirst.frc.team503.robot.RobotState.TargetHeight;
import org.usfirst.frc.team503.robot.subsystems.Arm;
import org.usfirst.frc.team503.robot.subsystems.Extension;
import org.usfirst.frc.team503.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj.command.Command;

public class SuperStructureCommand extends Command {

	private double aTgt, eTgt, wTgt, eLim = 0;
	private int manualIdx = 0;

	public SuperStructureCommand() {
		requires(Arm.getInstance());
		requires(Wrist.getInstance());
		requires(Extension.getInstance());
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		aTgt = Arm.getInstance().getEncoderDeg();
		eTgt = Robot.bot.gExtMinLim;
		wTgt = 90.0;
		RobotState.getInstance().setIsManual(false);
		RobotState.getInstance().setArmDirection(ArmDirection.FRONT);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if ((RobotState.getInstance().getState() == State.AUTON || !RobotState.getInstance().getIsManual())) {
			manualIdx = 0;
			if (RobotState.getInstance().getPositionChanged()) {
				ArmDirection armDirection = RobotState.getInstance().getArmDirection();
				TargetHeight tgtHeight = RobotState.getInstance().getTargetHeight();
				if (armDirection.equals(ArmDirection.FRONT)) {
					if (RobotState.getInstance().getGameElement() == GameElement.CARGO) {
						switch (tgtHeight) {
						case INTAKE:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_CARGO_INTAKE);
							break;
						case LOW:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_CARGO_LOW);
							break;
						case MIDDLE:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_CARGO_MID);
							break;
						case HIGH:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_CARGO_HIGH);
							break;
						case BUS:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_CARGO_BUS);
							break;
						case HOME:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.CARGO_HOME);
							break;
						}

					} else {
						switch (tgtHeight) {
						case INTAKE:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_HATCH_INTAKE);
							break;
						case LOW:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_HATCH_LOW);
							break;
						case MIDDLE:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_HATCH_MID);
							break;
						case HIGH:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_HATCH_HIGH);
							break;
						case BUS:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_CARGO_BUS);
							break;
						case HOME:
							if (RobotState.getInstance().getState() == RobotState.State.AUTON){
								RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.AUTO_HOME);
							}
							else{
								RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.HATCH_HOME);
							}
						
							break;
						}
					}
				} else if (armDirection.equals(ArmDirection.BACK)) {
					if (RobotState.getInstance().getGameElement() == GameElement.CARGO) {
						switch (tgtHeight) {
						case INTAKE:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_CARGO_INTAKE);
							break;
						case LOW:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_CARGO_LOW);
							break;
						case MIDDLE:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_CARGO_MID);
							break;
						case HIGH:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_CARGO_HIGH); 
							break;
						case BUS:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_CARGO_BUS);
							break;
						case HOME:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.HATCH_HOME);
							break;
						}
					} else {
						switch (tgtHeight) {
						case INTAKE:
							if (RobotState.getInstance().getState() == State.AUTON) {
								RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.VIEW_AUTO_REAR);
								break;
							}
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_HATCH_INTAKE);
							break;
						case LOW:
							if (RobotState.getInstance().getState() == State.AUTON) {
								RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_HATCH_LOW);
								break;
							}
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_HATCH_LOW);
							break;
						case MIDDLE:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_HATCH_MID);
							break;
						case HIGH:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_HATCH_HIGH);
							break;
						case BUS:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_CARGO_BUS);
							break;
						case HOME:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.HATCH_HOME);
							break;
						}

					}
				}
				RobotState.getInstance().setPositionChanged(false);

				SuperStructurePreset preset = RobotState.getInstance().getSuperStructurePreset();
				wTgt = preset.getWristPosition();
				aTgt = preset.getArmPosition();
				eTgt = preset.getExtPosition();
			}
			if (Robot.bot.hasWrist()) {
				double wristPower = Wrist.getInstance().getTalon().getOutputCurrent()
						* Wrist.getInstance().getTalon().getMotorOutputVoltage();
				if (wristPower > Robot.bot.MAX_WRIST_POWER) {
					Wrist.getInstance().setMotorOutput(0.0);
					System.out.println("WRIST POWER TOO HIGH BURN OUT WARNING");
				} else {
					if (!(RobotState.getInstance().getSuperStructurePreset() == SuperStructurePreset.FRONT_CARGO_BUS
							&& Arm.getInstance().getEncoderDeg() < -30)) {
						Wrist.getInstance().setTargetPosition(wTgt);
					} else if (RobotState.getInstance()
							.getSuperStructurePreset() == SuperStructurePreset.FRONT_CARGO_INTAKE
							&& Wrist.getInstance().getMagicStall() && Arm.getInstance().getMagicStall()
							&& Extension.getInstance().getMagicStall()) {
								if (Wrist.getInstance().getWristLimit()) {
									Wrist.getInstance().setMotorOutput(0.0);
								} else {
									Wrist.getInstance().setMotorOutput(-0.2);
								}
					}

				}
			}
			if (Robot.bot.hasArm()) {
				Arm.getInstance().setTargetPosition(aTgt);
				if (Arm.getInstance().getEncoderDeg() > -40) {
					Extension.getInstance().setTargetPosition(eTgt);
				} else {
					Extension.getInstance().setTargetPosition(0.0);
				}
			}
		} else {
			if (manualIdx == 0) {
				(new RumbleOperatorJoystick(1.0)).start();
				manualIdx++;
			}
			if (Robot.bot.hasArm()) {

				if (Arm.getInstance().getEncoderDeg() > 0.0 && Arm.getInstance().getEncoderDeg() < 180.0) {
					eLim = Robot.bot.gArmExtLength
							* (1 / Math.cos(Math.toRadians(Arm.getInstance().getEncoderDeg())) - 1);
					eLim = Math.abs(eLim);
				} else {
					eLim = 0.0;
				}

				aTgt = Arm.getInstance().getEncoderDeg();
				wTgt = Wrist.getInstance().getHRelEncoderDeg();
				eTgt = Extension.getInstance().getExtPosition();
				Arm.getInstance().setMotorOutput(-OI.getOperatorLeftYValue());
				Wrist.getInstance().setMotorOutput(-OI.getOperatorRightYValue());

				if (OI.operatorDPadUp.get()) {
					Extension.getInstance().setMotorOutput(0.50);
				} else if (OI.operatorDPadDown.get()) {
					Extension.getInstance().setMotorOutput(-0.50);
				} else {
					Extension.getInstance().setMotorOutput(0.00);
				}
			}
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return RobotState.getInstance().getState() == RobotState.State.DISABLED;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Arm.getInstance().setMotorOutput(0.0);
		Wrist.getInstance().setMotorOutput(0.0);
		Extension.getInstance().setMotorPower(0.0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
