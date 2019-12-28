/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified a b-----------------------------------*/

package org.usfirst.frc.team503.robot.auton;

import org.usfirst.frc.team503.robot.RobotState.ArmDirection;
import org.usfirst.frc.team503.robot.RobotState.TargetHeight;
import org.usfirst.frc.team503.robot.commands.AssistedArcadeDriveCommand;
import org.usfirst.frc.team503.robot.commands.MoveArmCommand;
import org.usfirst.frc.team503.robot.commands.PivotCommand;
import org.usfirst.frc.team503.robot.commands.ReleaseHatchCommand;
import org.usfirst.frc.team503.robot.commands.SetPositionCommand;
import org.usfirst.frc.team503.robot.vision.commands.FollowTargetAuton;

import edu.wpi.first.wpilibj.command.WaitCommand;

public class LeftSideFarRocket extends FroggyAuton {

	@Override
	protected void initAuton() {
		addParallel(new MoveArmCommand(ArmDirection.FRONT, TargetHeight.LOW, 1.5));
		froggySequentialDrive("leftToFarSideRocket", 3.0);
		addSequential(new PivotCommand(240), 0.7);
		// addSequential(new AssistedArcadeDriveCommand());
		addSequential(new FollowTargetAuton(-2.0, 0.6), 2.0);
		addParallel(new ReleaseHatchCommand(3.0));
		addSequential(new SetPositionCommand(FieldLocations.LeftBackRocket));
		// addSequential(new WaitCommand(0.2));
		froggySequentialDrive("leftFarSideRocketBackup", 1.0);
		addSequential(new PivotCommand(-60), 0.4);
		addParallel(new MoveArmCommand(ArmDirection.FRONT, TargetHeight.HOME));
		froggySequentialDrive("leftRocketToHatchReload", 1.7);
		addSequential(new FollowTargetAuton());
		froggySequentialDrive("leftReloadToNearRocket", 1.8);
		addParallel(new MoveArmCommand(ArmDirection.FRONT, TargetHeight.LOW));
		addSequential(new PivotCommand(130.0), 0.7);
		addSequential(new FollowTargetAuton(-1.5,  0.6));
		addParallel(new ReleaseHatchCommand(3.0));
		addParallel(new MoveArmCommand(ArmDirection.FRONT, TargetHeight.HOME));
		froggySequentialDrive("leftNearRocketBackup", 1.0);
		addSequential(new AssistedArcadeDriveCommand());
	}

	@Override
	protected AutonStartingLocation getStartingLocation() {
		return AutonStartingLocation.Left;
	}
}
