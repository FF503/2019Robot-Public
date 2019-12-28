/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.pidTuning;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.auton.FroggyAuton;
import org.usfirst.frc.team503.robot.utils.FFDashboard;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Add your docs here.
 */
public class PIDFDataHandler {
	FroggyAuton test = null;
	TestType testType = TestType.NULL;
	FFDashboard table;

	private void addTestListener() {
		table.getEntry("NewValues").addListener(event -> {
			boolean val = event.getEntry().getBoolean(false);
			if (val) {
				try {
					testType = TestType.valueOf(table.getString("testType", "NULL"));
					storePIDF();
					event.getEntry().setBoolean(false);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

		table.getEntry("RunTest").addListener(event -> {
			boolean val = event.getEntry().getBoolean(false);
			if (val) {
				runTest();
				event.getEntry().setBoolean(false);
			}
		}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

		table.getEntry("testType").addListener(event -> {
			String val = event.getEntry().getString("NULL");
			this.testType = TestType.valueOf(val);
			this.test = testType.getTest();
		}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

		table.getEntry("GetValues").addListener(event -> {
			boolean bool = event.getEntry().getBoolean(false);
			if (bool) {
				testType = TestType.valueOf(table.getString("testType", "NULL"));
				updateValues();
				event.getEntry().setBoolean(false);
			}
		}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
	}

	public void resetClass() {
		table = new FFDashboard("PidTuning");
		addTestListener();
		testType = TestType.NULL;
		test = null;
	}

	private void runTest() {
		if (this.testType == TestType.NULL) {
			this.testType = TestType.valueOf(table.getString("testType", "NULL"));

			this.test = testType.getTest();
		}
		Command testCommand = new Command() {
			@Override
			protected void initialize() {
				test.initAndStartAuton();
			}

			@Override
			protected boolean isFinished() {
				return test.isCompleted();
			}

			@Override
			protected void end() {
				test.cancel();
				table.putBoolean("runTest", false);
			}

			@Override
			protected void interrupted() {
				end();
			}
		};

		testCommand.start();
	}

	private static PIDFDataHandler instance = new PIDFDataHandler();

	public static PIDFDataHandler getInstance() {
		return instance;
	}

	public void storePIDF() {
		testType.p = table.getNumber("P", Robot.bot.kP_PurePursuit);
		testType.i = table.getNumber("I", Robot.bot.kI_PurePursuit);
		testType.d = table.getNumber("D", Robot.bot.kD_PurePursuit);
		testType.v = table.getNumber("V", Robot.bot.kV_PurePursuit);
		testType.a = table.getNumber("A", Robot.bot.kA_PurePursuit);
		testType.turnP = table.getNumber("TurnKP", Robot.bot.turnPurePursuitKp);
		testType.f = table.getNumber("F", 0.0);
	}

	public TestType getCurrentTest() {
		return testType;
	}

	public void updateValues() {
		table.putNumber("P", testType.getP());
		table.putNumber("I", testType.getI());
		table.putNumber("D", testType.getD());
		table.putNumber("V", testType.getV());
		table.putNumber("A", testType.getA());
		table.putNumber("turnP", TestType.TURN_PID.getTurnP());
		table.putNumber("F", testType.getF());
	}

	public static enum TestType {
		DRIVE_FORWARD_PIDF(new DriveForwardTest(), Robot.bot.kP_PurePursuit, Robot.bot.kI_PurePursuit,
				Robot.bot.kD_PurePursuit, Robot.bot.kV_PurePursuit, Robot.bot.kA_PurePursuit),
	
		TURN_PID(new Curve90Test(), Robot.bot.turnPurePursuitKp), MOTION_MAGIC_ARM(), MOTION_MAGIC_WRIST(),
		MOTION_MAGIC_EXTENSION(), NULL();

		FroggyAuton autonTest;
		public double p = 0, i = 0, d = 0, v = 0, a = 0, turnP = 0, f = 0;

		TestType(FroggyAuton auton, double p) {
			autonTest = auton;
			this.p = p;
		}

		TestType(FroggyAuton auton, double p, double i, double d) {
			autonTest = auton;
			this.p = p;
			this.i = i;
			this.d = d;
		}

		TestType(FroggyAuton auton, double p, double i, double d, double v, double a) {
			autonTest = auton;
			this.p = p;
			this.i = i;
			this.d = d;
			this.v = v;
			this.a = a;
		}

		TestType() {
		}

		public double getP() {
			return p;
		}

		public double getI() {
			return i;
		}

		public double getD() {
			return d;
		}

		public double getV() {
			return v;
		}

		public double getA() {
			return a;
		}

		public double getF() {
			return f;
		}

		public double getTurnP() {
			return turnP;
		}

		public FroggyAuton getTest() {
			return autonTest;
		}
	}
}
