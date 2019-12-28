/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.auton;

import java.util.HashMap;

import org.usfirst.frc.team503.robot.utils.FFDashboard;

import edu.wpi.first.networktables.EntryListenerFlags;

public class AutonChooser {
	private HashMap<String, FroggyAuton> autonMap = new HashMap<String, FroggyAuton>();
	FroggyAuton selectedAuton = null;
	FFDashboard table;
	FroggyAuton defaultDriveAuton = new TeleopAuton();

	private final String autonBooleanKey = "autonGo";
	private final String autonNameKey = "autonName";

	public void resetClass() {
		selectedAuton = null;
		autonMap.clear();
		table = new FFDashboard("Auton");
		populate();
		addAutonListener();
		table.putBoolean(this.autonBooleanKey, false);
		table.putString(this.autonNameKey, "Teleop Auton");
	}

	private void populate() {
		autonMap.put("Left To Center", new TeleopAuton());
		autonMap.put("Left Side Cargo Fill Up", new LeftSideCargoFillUp());
		autonMap.put("Left Side Cargo Fill Up Level 2", new LeftSideCargoFillUpLevel2());
		autonMap.put("Left Side Near Rocket", new LeftSideFarRocket());
		autonMap.put("Left Side Far Rocket", new LeftSideFarRocket());
		autonMap.put("Center Cargo Left", new TeleopAuton());
		autonMap.put("Center Cargo Right", new TeleopAuton());
		autonMap.put("Right To Center", new RightSideCenterAuton());
		autonMap.put("Right Side Cargo Fill Up", new RightSideCargoFillUp());
		autonMap.put("Right Side Cargo Fill Up Level 2", new RightSideCargoFillUpLevel2());
		autonMap.put("Right Side Near Rocket", new RightSideNearRocket());
		autonMap.put("Right Side Far Rocket", new RightSideFarRocket());
		autonMap.put("Right Side Far Rocket Level 2", new RightSideFarRocketLevel2());
		autonMap.put("Teleop Auton", new TeleopAuton());
		// old stuff
		// autonMap.put("Right Rocket Lvl 1", new RightSideRocket());
		// autonMap.put("Center Cargo fill up swing right", new TeleopAuton());
		// autonMap.put("Right Rocket Lvl 3", new RightSideCargoFillUpLevel2());
		// autonMap.put("Left Rocket Lvl 3", new LeftSideCargoFillUpLevel2());
		// autonMap.put("Teleop Auton", new TeleopAuton());
		// autonMap.put("Right To Center", new RightSideCenterAuton());

	}

	private void addAutonListener() {
		table.getEntry(autonBooleanKey).addListener(event -> {
			boolean bool = event.getEntry().getBoolean(false);
			if (bool) {
				String autonName = table.getString(autonNameKey, "Teleop Auton").trim();
				// this.selectedAuton = autonMap.get(autonName);
				// preloadTrajectory();
				// initAuton();
				setAndInitAuton(autonMap.get(autonName));
				table.putString("Robot Selected Auton", autonName);
				event.getEntry().setBoolean(false);
			}
		}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
	}

	public void setAndInitAuton(FroggyAuton auton) {
		this.selectedAuton = auton;
		initAuton();
	}

	public void cancelAuton() {
		// if (getAuton() != null) {
		// if (isAutonRunning()) {
		this.selectedAuton.cancel();
		// }
		// }
	}

	public boolean isAutonRunning() {
		return this.selectedAuton.isRunning();
	}

	private void initAuton() {
		this.selectedAuton.init();
	}

	public void runSelectedAuton() {
		this.selectedAuton.startAuton();
	}

	public FroggyAuton getAuton() {
		return this.selectedAuton;
	}

	private static AutonChooser instance = new AutonChooser();

	public static AutonChooser getInstance() {
		return instance;
	}

	public void runDefaultDrive() {
		if (!defaultDriveAuton.isRunning()) {
			cancelAuton();
			defaultDriveAuton.initAndStartAuton();
		}
	}

}
