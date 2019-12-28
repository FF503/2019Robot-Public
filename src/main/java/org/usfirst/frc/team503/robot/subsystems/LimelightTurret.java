/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.subsystems;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.RobotState.CameraDirection;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class LimelightTurret extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private Servo camServo;

	final double backPos = Robot.bot.backwardLimelightPosition, frontPos = Robot.bot.forwardLimelightPosition;

	private static LimelightTurret instance = new LimelightTurret();

	public static LimelightTurret getInstance() {
		return instance;
	}

	public LimelightTurret() {
		if(Robot.bot.hasLimelightTurret()) {
			camServo = new Servo(Robot.bot.limelightTurretServoID);
		} else {
			camServo = null;
		}
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public double getPosition() {
		if(Robot.bot.hasLimelightTurret()) {
			return camServo.get();
		} else {
			return 0.0;
		}	
	}

	public void setPosition(double angle) {
		if(Robot.bot.hasLimelightTurret()) {
			camServo.set(angle);
		}
	}

	public void turnToFront() {
		if(Robot.bot.hasLimelightTurret()) {
			setPosition(frontPos);
			
			// System.out.println("servo:" + camServo.get());
			// System.out.println("servo id: " + camServo.getChannel());
		}
		RobotState.getInstance().setCameraDirection(CameraDirection.FRONT);
		
	}

	public void turnToBack() {
		if(Robot.bot.hasLimelightTurret()) {
			setPosition(backPos);
			// System.out.println("servo:" + camServo.get());
			// System.out.println("servo id: " + camServo.getChannel());
		}
		RobotState.getInstance().setCameraDirection(CameraDirection.BACK);
	}

}