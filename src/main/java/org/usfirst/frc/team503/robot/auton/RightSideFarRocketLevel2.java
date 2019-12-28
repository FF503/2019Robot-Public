/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified a b-----------------------------------*/

package org.usfirst.frc.team503.robot.auton;

import org.usfirst.frc.team503.robot.RobotState.ArmDirection;
import org.usfirst.frc.team503.robot.RobotState.TargetHeight;
import org.usfirst.frc.team503.robot.commands.AssistedArcadeDriveCommand;
import org.usfirst.frc.team503.robot.commands.DepartLevel2Command;
import org.usfirst.frc.team503.robot.commands.MoveArmCommand;
import org.usfirst.frc.team503.robot.commands.PivotCommand;
import org.usfirst.frc.team503.robot.commands.ReleaseHatchCommand;
import org.usfirst.frc.team503.robot.commands.SetPositionCommand;
import org.usfirst.frc.team503.robot.vision.commands.FollowTargetAuton;

import edu.wpi.first.wpilibj.command.WaitCommand;

public class RightSideFarRocketLevel2 extends FroggyAuton {

	@Override
	protected void initAuton() {
		// addSequential(new DepartLevel2Command(getStartingLocation()));
		addParallel(new MoveArmCommand(ArmDirection.FRONT, TargetHeight.LOW, 1.5));
		froggySequentialDrive("RightToFarSideRocket", 3.0);
		addSequential(new PivotCommand(-60), 0.7);
		addSequential(new FollowTargetAuton(-2.0), 2.0);
		addParallel(new ReleaseHatchCommand(3.0));
		addSequential(new SetPositionCommand(FieldLocations.RightBackRocket));
		addSequential(new WaitCommand(0.3));
		froggySequentialDrive("rightFarSideRocketBackup", 1.0);
		addSequential(new PivotCommand(250), 0.4);
		addParallel(new MoveArmCommand(ArmDirection.FRONT, TargetHeight.HOME));
		froggySequentialDrive("rocketToHatchReload", 2.0);
		addSequential(new FollowTargetAuton());
		froggySequentialDrive("RightReloadBackUp", 1.7);
		addSequential(new PivotCommand(50.0), 0.7);
		addSequential(new MoveArmCommand(ArmDirection.FRONT, TargetHeight.LOW));
		addSequential(new FollowTargetAuton(-1.5));
		addParallel(new ReleaseHatchCommand(3.0));
		froggySequentialDrive("rightNearRocketBackup", 1.0);
		addSequential(new AssistedArcadeDriveCommand());
	}

	@Override
	protected AutonStartingLocation getStartingLocation() {
		return AutonStartingLocation.Right;
	}
}
