/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.vision;

import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.vision.commands.FollowTargetSimple;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriveToTarget1 extends CommandGroup {

	public DriveToTarget1(TargetData targetData) {
		RobotState.getInstance().setTargetData(targetData);
		// VisionLocalizer.getInstance().setIndividual();
		addSequential(new FollowTargetSimple());
		// addSequential(new DriveStraightNew());

		// addSequential(new FinalizeAngle(true));

		// addSequential(new ScoreTarget());
		// addSequential(new DriveForwardTimeCommand(-0.3, 0.5));

		// addSequential(new PurePursuitVisionTarget());
		// addSequential(new NewFinalizeAngle(true));
	}

	public DriveToTarget1() {
		this(RobotState.getInstance().getTargetData());
	}
}
