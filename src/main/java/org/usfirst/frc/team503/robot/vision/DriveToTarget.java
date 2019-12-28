/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.vision;

import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.vision.commands.DriveStraightNew;
import org.usfirst.frc.team503.robot.vision.commands.FollowTargetNew;
import org.usfirst.frc.team503.robot.vision.commands.NewFinalizeAngle;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriveToTarget extends CommandGroup {

	public DriveToTarget(TargetData targetData) {
		RobotState.getInstance().setTargetData(targetData);
		VisionLocalizer.getInstance().setIndividual();
		addSequential(new FollowTargetNew());
		addSequential(new DriveStraightNew());

		// addSequential(new ScoreTarget());
		// addSequential(new DriveForwardTimeCommand(-0.3, 0.5));

		// addSequential(new PurePursuitVisionTarget());
		//addSequential(new VisionPurePursuit("visionForward"));
		addSequential(new NewFinalizeAngle(true));
	}
}
