package org.usfirst.frc.team503.robot.commands;

// import org.junit.runner.FilterFactory.FilterNotCreatedException;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.RobotState.ArmDirection;
import org.usfirst.frc.team503.robot.RobotState.GameElement;
import org.usfirst.frc.team503.robot.RobotState.TargetHeight;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.Intake;
import org.usfirst.frc.team503.robot.utils.FFDashboard;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CheckSystem {
    // Constants
    private final double driveMotorPower = 1.0; // Less than or equal to 1 and greater than 0
    private final double driveRunTime = 2.0; // Time to run each step (forward, left/right turns);
    private final double driveWaitTime = 1.0; // Wait Time in between forward/turns etc.

    private final double hatchConeWaitTime = 2.0; // Wait Time in between release and grab

    private final double cargoIntakeInTime = 5.0;
    private final double cargoIntakeWaitTime = 1.0;
    private final double cargoIntakeOutTime = 5.0;

    private FFDashboard table;

    private static CheckSystem instance = new CheckSystem();

    public CheckSystem() {
        table = new FFDashboard("SystemCheck");
    }

    public static CheckSystem getInstance() {
        return instance;
    }

    public void init() {
        resetSubsystems();
        initValues();
        initNetworkTableListener();
    }

    private void resetSubsystems() {
        Drive.getInstance().resetEncoders();
        Intake.getInstance().stopIntake();
    }

    private void initValues() {
        table.putBoolean("Hatch Intake", false);
        table.putBoolean("Drive", false);
        table.putBoolean("Cargo Intake", false);
        table.putBoolean("Arm", false);
    }

    private void initNetworkTableListener() {
        // Hatch Intake
        table.getEntry("Hatch Intake").addListener(event -> {
            if (event.value.getBoolean()) {
                testHatchIntake();
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        // Drive
        table.getEntry("Drive").addListener(event -> {
            if (event.value.getBoolean()) {
                testDrive();
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        // Cargo Intake
        table.getEntry("Cargo Intake").addListener(event -> {
            if (event.value.getBoolean()) {
                testCargoIntake();
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        // Arm
        table.getEntry("Arm").addListener(event -> {
            if (event.value.getBoolean()) {
                testArm();
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    }

    private Command updateTestName(String test) {
        return new Command() {
            @Override
            protected void initialize() {
                table.getEntry("TestStatus").setString(test);
            }

            @Override
            protected boolean isFinished() {
                return true;
            }
        };
    }

    private Command setSystemCheckRunning(boolean bool) {
        return new Command() {
            @Override
            protected void initialize() {
                RobotState.getInstance().setSystemCheckRunning(bool);
            }

            @Override
            protected boolean isFinished() {
                return true;
            }

        };
    }

    private Command setEntry(NetworkTableEntry networkTableEntry, boolean bool) {
        return new Command() {
            @Override
            protected void initialize() {
                networkTableEntry.setBoolean(bool);
            }

            @Override
            protected boolean isFinished() {
                return true;
            }
        };

    }

    private Command tankDrive(double left, double right) {
        return new Command() {
            @Override
            protected void initialize() {
                Drive.getInstance().tankDrive(left, right);
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void end() {
                Drive.getInstance().tankDrive(0, 0);
            }

            @Override
            protected void interrupted() {
                end();
            }
        };
    }

    private void testDrive() {
        CommandGroup drive = new CommandGroup();
        drive.addSequential(setSystemCheckRunning(true));

        // Drive Forward
        drive.addSequential(updateTestName("DriveForward"));
        drive.addSequential(tankDrive(driveMotorPower, driveMotorPower), driveRunTime);
        // drive.addSequential(new WaitCommand(driveRunTime));

        // Stop
        drive.addSequential(updateTestName("StopDriveMotors"));
        drive.addSequential(new WaitCommand(driveWaitTime));

        // Turn Left
        drive.addSequential(updateTestName("TurnLeft"));
        drive.addSequential(tankDrive(-driveMotorPower, driveMotorPower), driveRunTime);

        // drive.addSequential(new WaitCommand(driveRunTime));

        // Stop
        drive.addSequential(updateTestName("StopDriveMotors"));

        drive.addSequential(new WaitCommand(driveWaitTime));

        // Turn Right
        drive.addSequential(updateTestName("TurnRight"));
        drive.addSequential(tankDrive(driveMotorPower, -driveMotorPower), driveRunTime);
        // drive.addSequential(new WaitCommand(driveRunTime));

        // Stop
        drive.addSequential(updateTestName("StopDriveMotors"));
        // drive.addSequential(tankDrive(0, 0));

        drive.addSequential(setEntry(table.getEntry("Drive"), false));

        drive.addSequential(setSystemCheckRunning(false));
        // drive.addSequential(resetTestName());
        drive.start();
    }

    private void testHatchIntake() {
        CommandGroup hatchPanelSucker = new CommandGroup();

        hatchPanelSucker.addSequential(setSystemCheckRunning(true));
        hatchPanelSucker.addSequential(updateTestName("Releasing Hatch"));

        hatchPanelSucker.addSequential(new ReleaseGrabberCommand());

        hatchPanelSucker.addSequential(new WaitCommand(hatchConeWaitTime));
        hatchPanelSucker.addSequential(updateTestName("Grabbing Hatch"));
        hatchPanelSucker.addSequential(new GrabGrabberCommand());

        hatchPanelSucker.addSequential(updateTestName(""));
        hatchPanelSucker.addSequential(setEntry(table.getEntry("Hatch Intake"), false));

        hatchPanelSucker.addSequential(setSystemCheckRunning(false));
        hatchPanelSucker.start();
    }

    private void testCargoIntake() {
        CommandGroup cargoIntake = new CommandGroup();

        cargoIntake.addSequential(setSystemCheckRunning(true));
        cargoIntake.addSequential(updateTestName("Intaking Cargo"));

        cargoIntake.addSequential(new Command() {
            @Override
            protected void initialize() {
                Intake.getInstance().intakeCargo();
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void interrupted() {
                Intake.getInstance().stopIntake();
            }
        }, cargoIntakeInTime);

        cargoIntake.addSequential(updateTestName("Stop Cargo Intake"));
        cargoIntake.addSequential(new WaitCommand(cargoIntakeWaitTime));

        cargoIntake.addSequential(updateTestName("Outtaking Cargo"));
        cargoIntake.addSequential(new Command() {
            @Override
            protected void initialize() {
                Intake.getInstance().intakeHatch();
            }

            @Override
            protected boolean isFinished() {
                return false;
            }

            @Override
            protected void interrupted() {
                Intake.getInstance().stopIntake();
            }
        }, cargoIntakeOutTime);

        cargoIntake.addSequential(updateTestName(""));

        cargoIntake.addSequential(setEntry(table.getEntry("Cargo Intake"), false));
        cargoIntake.addSequential(setSystemCheckRunning(true));
        cargoIntake.start();

    }

    private void testArm() {
        CommandGroup arm = new CommandGroup();
        arm.addSequential(setSystemCheckRunning(true));

        arm.addSequential(new SetArmDirection(ArmDirection.FRONT));
        arm.addSequential(new GameElementSwitcher(GameElement.HATCH));
        // arm.addSequential(new TargetHeightSwitcher(TargetHeight.LOW));
        // arm.addSequential(new WaitCommand(2));
        // arm.addSequential(new TargetHeightSwitcher(TargetHeight.MIDDLE));
        // arm.addSequential(new WaitCommand(2));
        // arm.addSequential(new SwitchArmDirection());
        arm.addSequential(new TargetHeightSwitcher(TargetHeight.HIGH));
        arm.addSequential(new WaitCommand(2));
        arm.addSequential(new TargetHeightSwitcher(TargetHeight.LOW));
        arm.addSequential(new WaitCommand(2));
        arm.addSequential(new TargetHeightSwitcher(TargetHeight.MIDDLE));
        arm.addSequential(new WaitCommand(2));
        arm.addSequential(new TargetHeightSwitcher(TargetHeight.LOW));
        arm.addSequential(new WaitCommand(2));

        arm.addSequential(setEntry(table.getEntry("Arm"), false));
        arm.addSequential(setSystemCheckRunning(false));

        arm.start();
    }

}