package org.usfirst.frc.team503.robot;

import org.usfirst.frc.team503.robot.auton.AutonChooser;
import org.usfirst.frc.team503.robot.auton.TeleopAuton;
import org.usfirst.frc.team503.robot.commands.AssistedArcadeDriveCommand;
import org.usfirst.frc.team503.robot.commands.SuperStructureCommand;
import org.usfirst.frc.team503.robot.commands.TeleopClimbCommand;
import org.usfirst.frc.team503.robot.commands.TeleopLimelightCommand;
import org.usfirst.frc.team503.robot.commands.VacuumPowerCommand;
import org.usfirst.frc.team503.robot.lidar.LidarServer;
import org.usfirst.frc.team503.robot.pidTuning.PIDFDataHandler;
import org.usfirst.frc.team503.robot.subsystems.Arm;
import org.usfirst.frc.team503.robot.subsystems.Climber;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.Extension;
import org.usfirst.frc.team503.robot.subsystems.Gyro;
import org.usfirst.frc.team503.robot.subsystems.Intake;
import org.usfirst.frc.team503.robot.subsystems.Wrist;
import org.usfirst.frc.team503.robot.utils.GPS;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

	public static RobotHardwareCompBot bot = null;
	public static NetworkTable universalGrapherTable;
	private static double memory = 0;
	private static AssistedArcadeDriveCommand aDrive;
	private static SuperStructureCommand superStructureCommand;
	private static double lastLoopStamp = 0;

	/*
	 * CvSink cvSink; CvSource outputStream;
	 * 
	 * Mat source = new Mat(); Mat output = new Mat();
	 */

	private Compressor compressor;
	// Command autonomousCommand;
	// SendableChooser<Command> chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		bot = new RobotHardwareCompBot();
		RobotState.getInstance().setState(RobotState.State.DISABLED);
		memory = Runtime.getRuntime().freeMemory();

		NetworkTableInstance.create();
		NetworkTableInstance.getDefault().deleteAllEntries();

		VisionLocalizer.getInstance().getTable().getEntry("pipeline").setDouble(9.0);
		universalGrapherTable = NetworkTableInstance.getDefault().getTable("UniversalGrapher");

		// RobotState.getInstance().setCameraDirection(CameraDirection.BACK);
		if (Robot.bot.hasCompressor()) {
			compressor = new Compressor(0);
		}

		/* Initialize a new bot object */
		// aDrive = new AssistedArcadeDriveCommand();

		// bot.initialize();
		// bot.logSmartDashboard();
		OI.initialize();

		GPS.getInstance().stop();
		// AutonChooser.getInstance().resetClass();
		Drive.getInstance().resetEncoders();
		Drive.getInstance().setBrakeMode(false);

		// RobotState.getInstance().setTargetData(TargetData.CENTERCARGOLEFT);

		// chooser.addDefault("Default Auto", new TankDriveCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		// SmartDashboard.putData("Auto mode", chooser);
		// s = new SerialPort(115200, SerialPort.Port.kOnboard, 8,
		// SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
		// s.setReadBufferSize(8);
		// s.setFlowControl(FlowControl.kXonXoff);
		try {
			SmartDashboard.putString("LIDAR status", "starting");
			boolean started = LidarServer.getInstance().start();
			SmartDashboard.putString("LIDAR status", started ? "started" : "failed to start");
		} catch (Throwable t) {
			SmartDashboard.putString("LIDAR status", "crashed: " + t);
			t.printStackTrace();
			throw t;
		}
		PIDFDataHandler.getInstance().resetClass();

		Arm.getInstance().resetEncoder();
		Wrist.getInstance().resetEncoder();
		Extension.getInstance().resetEncoder();

		// VisionLocalizer.getInstance().startTracking();

		Gyro.getInstance().resetGyro();
		VisionLocalizer.getInstance().setDrive();
		aDrive = new AssistedArcadeDriveCommand();
		superStructureCommand = new SuperStructureCommand();

		AutonChooser.getInstance().resetClass();

	}

	/**
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */

	@Override
	public void disabledInit() {
		VisionLocalizer.getInstance().getTable().getEntry("stream").setNumber(0);
		memory = Runtime.getRuntime().freeMemory();
		RobotState.getInstance().setState(RobotState.State.DISABLED);
		Drive.getInstance().resetTimer();
		// System.out.println("working");
		SmartDashboard.putNumber("memory", memory);
		Drive.getInstance().setBrakeMode(RobotState.getInstance().getBrakeInDisable());
		if (Robot.bot.hasClimber()) {
			Climber.getInstance().setCoast();
		}
		if (aDrive.isRunning()) {
			aDrive.cancel();
		}
		if (superStructureCommand.isRunning()) {
			superStructureCommand.cancel();
		}
		// AutonChooser.getInstance().cancelAuton();
		// VisionLocalizer.getInstance().setDrive();

		// RobotState.getInstance().setDashboardEnabledStatus(false);
		// LockTarget.getInstance().startTracking();
		// VisionLocalizer.getInstance().startTracking();

	}

	@Override
	public void disabledPeriodic() {
		VisionLocalizer.getInstance().getTable().getEntry("stream").setNumber(0);
		memory = Runtime.getRuntime().freeMemory();
		SmartDashboard.putNumber("memory", memory);

		// Lidar.getInstance().sendDashboardData();

		// VisionLocalizer.getInstance().sendDashboardData();
		// VisionLocalizer.getInstance().setDrive();
		Scheduler.getInstance().run();

		// VisionLocalizer.getInstance().setIndividual();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString code to get the
	 * auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons to
	 * the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		VisionLocalizer.getInstance().getTable().getEntry("stream").setNumber(0);
		// Intake.getInstance().startVacuum();
		RobotState.getInstance().setState(RobotState.State.AUTON);
		// LimelightTurret.getInstance().turnToFront();
		memory = Runtime.getRuntime().freeMemory();
		if (RobotState.getInstance().getDriveReversed()) {
			RobotState.getInstance().toggleDriveReversed();
		}
		Drive.getInstance().setBrakeMode(true);
		Drive.getInstance().zeroPowerMotors();

		// RobotState.getInstance().setDashboardEnabledStatus(true);
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		// if (autonomousCommand != null)
		// autonomousCommand.cancel();
		Drive.getInstance().resetEncoders();
		// Gyro.getInstance().resetGyro();
		GPS.getInstance().startTracking();
		if (Robot.bot.hasClimber()) {
			Climber.getInstance().setBrake();
		}

		/*
		 * cvSink = CameraServer.getInstance().getVideo(); outputStream =
		 * CameraServer.getInstance().putVideo("Blur", 640, 480);
		 */
		// new ToggleIntakeCommand().start();
		// VisionLocalizer.getInstance().setPipeline(RobotState.PipelineSelector.LEFT.getPipeline());
		VisionLocalizer.getInstance().setDrive();
		if (AutonChooser.getInstance().getAuton() == null) {
			AutonChooser.getInstance().setAndInitAuton(/* Set Default Auton Here --> */ new TeleopAuton());
			AutonChooser.getInstance().runSelectedAuton();
		} else {
			AutonChooser.getInstance().runSelectedAuton();
		}

		superStructureCommand.start();
		// new TeleopClimbCommand().start();
		// new TestArmCommand().start();
		// // new TestWristCommand().start();
		// // (new TestExtensionCommand()).start();
		// // new TestClimberCommand().start();
		// aDrive.start();
		// c.start();
		// new LEDPatternCommand().start();
		// (new PivotCommand(180)).start();;
		new TeleopLimelightCommand().start();
		new VacuumPowerCommand().start();
		SmartDashboard.putNumber("memory", memory);

		// (new IntakeStallWatcher()).start();
	}

	/**
	 * ` This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		// memory = Runtime.getRuntime().freeMemory();
		// Drive.getInstance().sendDashboardData();
		// Drive.getInstance().updateDt();
		// // System.out.println("working");
		// SmartDashboard.putNumber("memory", memory);
		// Gyro.getInstance().sendDashboardData();
		// RobotState.getInstance().sendPoseDashboardData();

		// // Arm.getInstance().sendDashboardData();
		// // Wrist.getInstance().sendDashboardData();
		// // Extension.getInstance().sendDashboardData();
		// Scheduler.getInstance().run();

		SmartDashboard.putNumber("memory", memory);
		memory = Runtime.getRuntime().freeMemory();
		// Drive.getInstance().calculateKinematicData();
		Drive.getInstance().updateDt();

		// RobotState.getInstance().sendPoseDashboardData();
		// System.out.println(Intake.getInstance().getHasBall());
		// VisionLocalizer.getInstance().sendDashboardData();
		RobotState.getInstance().sendPoseDashboardData();
		RobotState.getInstance().sendLedStatusData();

		Scheduler.getInstance().run();
		// Arm.getInstance().sendDashboardData();
		// Wrist.getInstance().sendDashboardData();
		// Extension.getInstance().sendDashboardData();
		// Intake.getInstance().sendDashboardData();
		RobotState.getInstance().sendSuperStructureDashboardData();

		// if (Drive.getInstance().checkLeftEncoderFault() ||
		// Drive.getInstance().checkRightEncoderFault()) {
		// AutonChooser.getInstance().runDefaultDrive();
		// }

	}

	@Override
	public void teleopInit() {
		VisionLocalizer.getInstance().getTable().getEntry("stream").setNumber(0);
		memory = Runtime.getRuntime().freeMemory();
		RobotState.getInstance().setState(RobotState.State.TELEOP);
		// LimelightTurret.getInstance().turnToFront();
		// Intake.getInstance().startVacuum();

		if (RobotState.getInstance().getDriveReversed()) {
			RobotState.getInstance().toggleDriveReversed();
		}
		Drive.getInstance().setBrakeMode(true);
		Drive.getInstance().zeroPowerMotors();
		// RobotState.getInstance().setDashboardEnabledStatus(true);
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		// if (autonomousCommand != null)
		// autonomousCommand.cancel();
		Drive.getInstance().resetEncoders();
		// Gyro.getInstance().resetGyro();
		GPS.getInstance().startTracking();
		Climber.getInstance().setBrake();
		VisionLocalizer.getInstance().setDrive();
		/*
		 * cvSink = CameraServer.getInstance().getVideo(); outputStream =
		 * CameraServer.getInstance().putVideo("Blur", 640, 480);
		 */
		// new ToggleIntakeCommand().start();

		superStructureCommand.start();
		new TeleopClimbCommand().start();
		new TeleopLimelightCommand().start();
		new VacuumPowerCommand().start();
		// new TestArmCommand().start();
		// new TestWristCommand().start();
		// (new TestExtensionCommand()).start();
		// new TestClimberCommand().start();
		// VisionLocalizer.getInstance().setPipeline(RobotState.PipelineSelector.LEFT.getPipeline());
		aDrive.start();
		// (new ArcadeDriveCommand()).start();
		// new TeleopLimelightCommand().start();
		if (Robot.bot.hasCompressor()) {
			compressor.start();
		}
		// if (Robot.bot.hasLEDs()) {
		// new LEDPatternCommand().start();
		// }

		SmartDashboard.putNumber("memory", memory);
		// (new IntakeStallWatcher()).start();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		SmartDashboard.putNumber("memory", memory);
		memory = Runtime.getRuntime().freeMemory();
		Drive.getInstance().updateDt();

		// System.out.println("gyro pitch = " + Gyro.getInstance().getPitch());
		// System.out.println("elev enc pos = " + Climber.getInstance().getElevPos());

		// Climber.getInstance().sendDashboardData();
		// RobotState.getInstance().sendPoseDashboardData();
		// System.out.println(Intake.getInstance().getHasBall());
		// VisionLocalizer.getInstance().sendDashboardData();
		// RobotState.getInstance().sendPoseDashboardData();

		// Arm.getInstance().sendDashboardData();
		// Wrist.getInstance().sendDashboardData();
		// Extension.getInstance().sendDashboardData();
		// Intake.getInstance().sendDashboardData();
		RobotState.getInstance().sendSuperStructureDashboardData();
		RobotState.getInstance().sendLedStatusData();
		RobotState.getInstance().sendPoseDashboardData();
		SmartDashboard.putNumber("loop time", Timer.getFPGATimestamp() - lastLoopStamp);
		SmartDashboard.putBoolean("VISION NOT FOUND", VisionLocalizer.getInstance().lostTarget());
		lastLoopStamp = Timer.getFPGATimestamp();
		Scheduler.getInstance().run();
	}

	@Override
	public void testInit() {
		memory = Runtime.getRuntime().freeMemory();
		RobotState.getInstance().setState(RobotState.State.TEST);
		// Intake.getInstance().startVacuum();
		compressor.start();
		// System.out.println("working");
		new VacuumPowerCommand().start();
		Drive.getInstance().resetTimer();
		SmartDashboard.putNumber("memory", memory);
		RobotState.getInstance().setDashboardEnabledStatus(true);
		Gyro.getInstance().resetGyro();
		Arm.getInstance().resetEncoder();
		Wrist.getInstance().resetEncoder();
		Extension.getInstance().resetEncoder();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		memory = Runtime.getRuntime().freeMemory();
		// System.out.println("working");
		SmartDashboard.putNumber("memory", memory);
		System.out.println("Gyro reset");
		Drive.getInstance().displayTemperatures();
		Gyro.getInstance().resetGyro();
		LiveWindow.updateValues();
		Scheduler.getInstance().run();
	}

	@Override
	public void robotPeriodic() {
		Arm.getInstance().sendDashboardData();
		Extension.getInstance().sendDashboardData();
		Wrist.getInstance().sendDashboardData();
		Intake.getInstance().sendDashboardData();
		Drive.getInstance().calculateKinematicData();
		Drive.getInstance().sendDashboardData();
		Gyro.getInstance().sendDashboardData();
		Climber.getInstance().sendDashboardData();
		// Drive.getInstance().updateDt();
		// RobotState.getInstance().sendVisionDashboardData();

		// double[] position = VisionLocalizer.getInstance()
		// .translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles());
		// double[] rawPosition = VisionLocalizer.getInstance()
		// .positionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles());
		// SmartDashboard.putNumber("Raw X", rawPosition[0]);
		// SmartDashboard.putNumber("Raw Y", rawPosition[1]);

		// SmartDashboard.putNumber("Translated X", position[0]);
		// SmartDashboard.putNumber("Translated Y", position[1]);

		// VisionLocalizer.getInstance().sendDashboardData();

		// SmartDashboard.putNumber("LiDAR @ 90",
		// RobotState.getInstance().getLidarDistance(90));
		// SmartDashboard.putNumber("LiDAR @ 270",
		// RobotState.getInstance().getLidarDistance(270));

		// SmartDashboard.putNumber("Adjusted angle",
		// RobotState.getInstance().getTargetData().getAdjustedAngle());
		// Gyro.getInstance().sendDashboardData();
		double tx = VisionLocalizer.getInstance().getTX();
		if (tx != 0.0) {
			RobotState.getInstance().setLostTarget(false);
		}
		double nx = VisionLocalizer.getInstance().deg2Normalized(tx);

		double pixelsX = 160 * nx + 159.5;
		double length = VisionLocalizer.getInstance().getTHor() / 2;
		double leftMost = (pixelsX - length);
		double rightMost = pixelsX + length;
		SmartDashboard.putNumber("leftMost", leftMost);
		SmartDashboard.putNumber("rightMost", rightMost);

		if (rightMost > 300 || leftMost < 20) {
			SmartDashboard.putBoolean("has target", false);
		} else {
			SmartDashboard.putBoolean("has target", true);
		}

		SmartDashboard.putNumber("Pixelsx", pixelsX);
	}
}
