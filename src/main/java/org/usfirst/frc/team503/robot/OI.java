package org.usfirst.frc.team503.robot;

import org.usfirst.frc.team503.robot.RobotState.ArmDirection;
import org.usfirst.frc.team503.robot.RobotState.LedColors;
import org.usfirst.frc.team503.robot.RobotState.PipelineSelector;
import org.usfirst.frc.team503.robot.RobotState.TargetHeight;
import org.usfirst.frc.team503.robot.commands.AssistedArcadeDriveCommand;
import org.usfirst.frc.team503.robot.commands.AutoClimbCommand;
import org.usfirst.frc.team503.robot.commands.AutoHabTwoCommand;
import org.usfirst.frc.team503.robot.commands.CancelAutonCommand;
import org.usfirst.frc.team503.robot.commands.ConstantIntakeCommand;
import org.usfirst.frc.team503.robot.commands.EjectCube;
import org.usfirst.frc.team503.robot.commands.GameElementSwitcher;
import org.usfirst.frc.team503.robot.commands.LedSetCommand;
import org.usfirst.frc.team503.robot.commands.MoveArmCommand;
import org.usfirst.frc.team503.robot.commands.ReleaseHatchCommand;
import org.usfirst.frc.team503.robot.commands.ResetEncoderCommand;
import org.usfirst.frc.team503.robot.commands.ReverseDriveCommand;
import org.usfirst.frc.team503.robot.commands.SetPipelineEnum;
import org.usfirst.frc.team503.robot.commands.SwitchArmDirection;
import org.usfirst.frc.team503.robot.commands.TargetHeightSwitcher;
import org.usfirst.frc.team503.robot.commands.TeleopClimbCommand;
import org.usfirst.frc.team503.robot.commands.ToggleControlModeCommand;
import org.usfirst.frc.team503.robot.commands.ToggleIntakeCommand;
import org.usfirst.frc.team503.robot.commands.WindupClimberCommand;
import org.usfirst.frc.team503.robot.vision.commands.TeleopFollowTarget;
import org.usfirst.frc.team503.robot.vision.commands.TeleopFollowTarget.FollowingType;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
	private static XboxController driverJoystick = new XboxController(0);
	private static XboxController operatorJoystick = new XboxController(1);
	private static XboxController climberJoystick = new XboxController(2);

	private static JoystickButton releaseHatch = new JoystickButton(driverJoystick, 1);
	private static JoystickButton toggleIntake = new JoystickButton(driverJoystick, 3);
	private static JoystickButton runOuttake = new JoystickButton(driverJoystick, 2);
//	private static JoystickButton autoPlace = new JoystickButton(driverJoystick, 9);
	private static JoystickButton autoClimbLvl2 = new JoystickButton(driverJoystick, 10);
	private static JoystickButton autoFollow = new JoystickButton(driverJoystick, 4);
	// private static JoystickButton finishClimb = new
	// JoystickButton(driverJoystick, 8);
	private static JoystickButton climb1 = new JoystickButton(driverJoystick, 7);
	private static JoystickButton climb2 = new JoystickButton(driverJoystick, 8);
	private static JoystickButton cancelAuton1 = new JoystickButton(driverJoystick, 5);
	private static JoystickButton cancelAuton2 = new JoystickButton(driverJoystick, 6);

	private static JoystickButton gotoHigh = new JoystickButton(operatorJoystick, 4);
	private static JoystickButton gotoMid = new JoystickButton(operatorJoystick, 2);
	private static JoystickButton gotoLow = new JoystickButton(operatorJoystick, 1);
	private static JoystickButton gotoIntake = new JoystickButton(operatorJoystick, 8);
	private static JoystickButton toggleManual = new JoystickButton(operatorJoystick, 7);
	private static JoystickButton gotoBus = new JoystickButton(operatorJoystick, 3);
	private static JoystickButton switchArmDirection = new JoystickButton(operatorJoystick, 5);
	private static JoystickButton gotoHome = new JoystickButton(operatorJoystick, 6);
	private static JoystickButton resetEnc = new JoystickButton(operatorJoystick, 10);

	private static JoystickButton retractClimb = new JoystickButton(climberJoystick, 4);
	private static JoystickButton climbRatio = new JoystickButton(climberJoystick, 1);
	private static JoystickButton exitAutoClimb = new JoystickButton(climberJoystick, 3);

	public static Button setToHPC = new Button() {
		@Override
		public boolean get() {
			return operatorJoystick.getRawAxis(2) >= 0.9;
		}
	};

	public static Button autoClimbButton = new Button() {
		@Override
		public boolean get() {
			return climb1.get() && climb2.get();
			// return climbOld.get();
		}
	};

	public static Button cancelAuton = new Button() {

		@Override
		public boolean get() {
			return cancelAuton1.get() && cancelAuton2.get();
		}
	};

	public static Button setToCargo = new Button() {
		@Override
		public boolean get() {
			return operatorJoystick.getRawAxis(3) >= 0.9;
		}
	};

	public static Button passiveIntake = new Button() {

		@Override
		public boolean get() {
			return getDriverRightTrigger();
		}
	};

	public static boolean getToggleIntake() {
		return toggleIntake.get();
	}

	public static boolean getDriverDPadLeft() {
		return driverDPadLeft.get();
	}

	public static boolean getSetRocketButton() {
		return false;
	}

	public static boolean getDriverDPadRight() {
		return driverDPadRight.get();
	}

	public static boolean getRunOuttake() {
		return runOuttake.get();
	}

	public static boolean getToggleGameElement() {
		return false;
	}

	public static boolean getClimbOldButton() {
		return climbRatio.get();
	}

	public static Button operatorDPadUp = new Button() {
		@Override
		public boolean get() {
			return operatorJoystick.getPOV(0) == 0 || operatorJoystick.getPOV(0) == 45
					|| operatorJoystick.getPOV(0) == 315;
		}
	};

	public static Button operatorDPadDown = new Button() {
		@Override
		public boolean get() {
			return ((operatorJoystick.getPOV(0) == 180) || (operatorJoystick.getPOV(0) == 225)
					|| operatorJoystick.getPOV(0) == 135);
				
		}
	};

	public static Button operatorDPadLeft = new Button() {
		@Override
		public boolean get() {
			return (operatorJoystick.getPOV(0) == 270);
		}
	};

	public static Button driverDPadUp = new Button() {
		@Override
		public boolean get() {
			return getDriverDPadUp();
		}
	};

	public static Button driverDPadDown = new Button() {
		@Override
		public boolean get() {
			return ((driverJoystick.getPOV(0) == 180) || (driverJoystick.getPOV(0) == 225)
					|| driverJoystick.getPOV(0) == 135);
		}
	};

	// public static Button driverDPadMiddle = new Button() {
	// @Override
	// public boolean get() {
	// return ((operatorJoystick.getPOV(0) == 90) || (operatorJoystick.getPOV(0) ==
	// 270));
	// }
	// };

	public static Button driverDPadLeft = new Button() {
		@Override
		public boolean get() {
			return (driverJoystick.getPOV(0) == 270);
		}
	};

	public static Button driverDPadRight = new Button() {
		@Override
		public boolean get() {
			return (driverJoystick.getPOV(0) == 90);
		}
	};
	// public static Button elevatorToScale = new Button() {
	// @Override
	// public boolean get() {
	// return operatorJoystick.getRawAxis(3) >= 0.9;
	// }
	// };

	public static Button reverseDrive = new Button() {
		@Override
		public boolean get() {
			return driverJoystick.getRawAxis(2) >= 0.9;
		}
	};

	public static void initialize() {
		// climb.whileHeld(new ClimbCommand());
		reverseDrive.whenPressed(new ReverseDriveCommand());
		// gotoBus.whenPressed(new ResetWristEncoderCommand());
		// followTarget.whenPressed(new ToggleFollowTarget());
		// followTarget.whenPressed(new TeleopFollowTarget());
		// followBall.whileHeld(new FollowBall());
		toggleIntake.toggleWhenPressed(new ToggleIntakeCommand());
		toggleManual.whenPressed(new ToggleControlModeCommand());
		switchArmDirection.whenPressed(new SwitchArmDirection());
		releaseHatch.whenPressed(new ReleaseHatchCommand());
		runOuttake.whileHeld(new EjectCube());
		passiveIntake.whileHeld(new ConstantIntakeCommand());
		// SwitchExtPosition(RobotState.ExtensionPosition.MIDDLE));
		gotoIntake.whenPressed(new TargetHeightSwitcher(RobotState.TargetHeight.INTAKE));
		gotoLow.whenPressed(new TargetHeightSwitcher(RobotState.TargetHeight.LOW));
		gotoMid.whenPressed(new TargetHeightSwitcher(RobotState.TargetHeight.MIDDLE));
		gotoHigh.whenPressed(new TargetHeightSwitcher(RobotState.TargetHeight.HIGH));
		gotoBus.whenPressed(new TargetHeightSwitcher(RobotState.TargetHeight.BUS));

		setToCargo.whenPressed(new GameElementSwitcher(RobotState.GameElement.CARGO));
		driverDPadLeft.whenPressed(new SetPipelineEnum(PipelineSelector.LEFT));
		driverDPadRight.whenPressed(new SetPipelineEnum(PipelineSelector.RIGHT));
		retractClimb.whenPressed(new WindupClimberCommand());
		gotoHome.whenPressed(new MoveArmCommand(ArmDirection.FRONT, TargetHeight.HOME));
		setToHPC.whenPressed(new GameElementSwitcher(RobotState.GameElement.HATCH));
		
		// autoClimbButton.whenPressed(new LedSetCommand(LedColors.GOLD,
		// LedColors.BLACK, 0.75, 0));
		autoClimbButton.toggleWhenPressed(new AutoClimbCommand());
		autoClimbLvl2.toggleWhenPressed(new AutoHabTwoCommand());
		cancelAuton.whenPressed(new CancelAutonCommand());

		// driverDPadLeft.whenPressed(new TeleopFollowTarget(PipelineSelector.LEFT,
		// -2.0));
		// driverDPadRight.whenPressed(new TeleopFollowTarget(PipelineSelector.RIGHT,
		// -2.0));
		exitAutoClimb.whenPressed(new AssistedArcadeDriveCommand());
		exitAutoClimb.whenPressed(new TeleopClimbCommand());
		autoFollow.whenPressed(new TeleopFollowTarget());
		// armHigh.whenPressed(new SwitchArmPosition(RobotState.ArmPosition.HIGH));
		// armLow.whenPressed(new SwitchArmPosition(RobotState.ArmPosition.LOW));
		// armMiddle.whenPressed(new SwitchArmPosition(RobotState.ArmPosition.MIDDLE));
		// switchFieldElement.whenPressed(new GameElementSwitcher());
		// switchArmDirection.whenPressed(new SwitchArmDirection());
		// toggleHatch.whenPressed(new ToggleGrabberCommand());

		resetEnc.whenPressed(new ResetEncoderCommand());
	}

	public static boolean getDriverDPadUp() {
		return driverJoystick.getPOV(0) == 0 || driverJoystick.getPOV(0) == 45 || driverJoystick.getPOV(0) == 315;
	}

	// public static boolean getTargetButton() {
	// return followTarget.get();
	// }

	// public static boolean getSetRocketButton() {
	// return setRocket.get();
	// }



	public static boolean getFollowButton() {
		return autoFollow.get();
	}

	// public static boolean getFollowBallBttn() {
	// return followBall.get();
	// }

	public static double getDriverLeftYValue() {
		return driverJoystick.getRawAxis(1);
	}

	public static double getClimberLeftYValue() {
		return climberJoystick.getRawAxis(1);
	}

	public static double getClimberRightYValue() {
		return climberJoystick.getRawAxis(5);
	}

	public static double getDriverLeftXValue() {
		return driverJoystick.getRawAxis(0);
	}

	public static double getDriverRightYValue() {
		return driverJoystick.getRawAxis(5);
	}

	public static double getDriverRightXValue() {
		return driverJoystick.getRawAxis(4);
	}

	public static boolean getDriverLeftTrigger() {
		return driverJoystick.getRawAxis(2) >= 0.9;
	}

	public static boolean getDriverRightTrigger() {
		return driverJoystick.getRawAxis(3) >= 0.9;
	}

	public static double getOperatorLeftYValue() {
		return operatorJoystick.getRawAxis(1);
	}

	public static double getOperatorRightYValue() {
		return operatorJoystick.getRawAxis(5);
	}

	public static boolean getOperatorLeftTrigger() {
		return operatorJoystick.getRawAxis(2) >= 0.9;
	}

	public static boolean getOperatorRightTrigger() {
		return operatorJoystick.getRawAxis(3) >= 0.9;
	}

	public static void setDriveRumble(double rumble) {
		driverJoystick.setRumble(GenericHID.RumbleType.kLeftRumble, rumble);
		driverJoystick.setRumble(GenericHID.RumbleType.kRightRumble, rumble);
	}

	public static void setOperatorRumble(double rumble) {
		// operatorJoystick.setRumble(GenericHID.RumbleType.kLeftRumble, rumble);
		// operatorJoystick.setRumble(GenericHID.RumbleType.kRightRumble, rumble);
	}

	// public static boolean getFinishClimb() {
	// return finishClimb.get();
	// }

	private enum Buttons {
		A(1), B(2), X(3), Y(4), LEFT_BUMPER(5), RIGHT_BUMPER(6), GUIDE(7), BACK(8), LEFT_JOYSTICK(9),
		RIGHT_JOYSTICK(10);

		private int id;

		Buttons(int id) {
			this.id = id;
		}

		int getID() {
			return id;
		}
	}

}
