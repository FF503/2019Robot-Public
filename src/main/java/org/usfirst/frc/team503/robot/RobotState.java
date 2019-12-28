package org.usfirst.frc.team503.robot;

import org.usfirst.frc.team503.robot.lidar.LidarServer;
import org.usfirst.frc.team503.robot.subsystems.Intake;
import org.usfirst.frc.team503.robot.subsystems.LED;
import org.usfirst.frc.team503.robot.utils.FFDashboard;
import org.usfirst.frc.team503.robot.utils.GPS;
import org.usfirst.frc.team503.robot.utils.Pose;
import org.usfirst.frc.team503.robot.utils.Vector2D;
import org.usfirst.frc.team503.robot.vision.TargetData;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState extends Subsystem {

	// Put methods for controlling this subsystem
	private boolean driveReversed;
	private State robotState;
	private double gyroAngle;
	private RobotDriveMode driveMode;
	private boolean driveProfileDone;
	private boolean isSystemCheckRunning;
	private boolean intakeRunning = false;
	private boolean climbDone;
	private Pose robotPose;
	private boolean releasing = false;

	// Vision things
	private boolean isDriveToTargetRunning;
	private TargetData targetInfo;
	private double currentPipeline;
	private boolean visionDebug = true;
	private double cameraMode = 0;
	private double[] visionStartPosition;
	private LimeLightType limeLightType = LimeLightType.ARM;

	private GameElement gameElement;
	private ArmDirection armDirection;
	private TargetHeight targetHeight;
	private boolean hasElement;
	private boolean hatchDependence;
	private LedColors ledColor;
	// private ExtensionPosition extPosition;
	private SuperStructurePreset sPreset;
	private CameraDirection camDirection;
	private boolean armPositionChanged;
	private boolean isArmFlip;
	private boolean grabberDeployed;
	private boolean isManualControl = true;
	private boolean toggleVision = false;
	private boolean lostTarget = false;
	private double[] targetAngles;
	private boolean isVisionFollowerRunning = false;
	private PipelineSelector pipelineSelector;
	private volatile StartingDirection startingDirection;
	private boolean hasCargo;
	private double autonToleranceAdjustment = 0;
	private boolean brakeInDisable = false;

	public RobotState() {
		driveReversed = false;
		robotState = State.DISABLED;
		gyroAngle = 0.0;
		driveMode = RobotDriveMode.LOW;
		driveProfileDone = true;
		isSystemCheckRunning = false;
		setSuperStructurePreset(SuperStructurePreset.FRONT_HATCH_INTAKE);
		setGameElement(GameElement.HATCH);
		// setArmPosition(ArmPosition.LOW);
		// setExtPosition(ExtensionPosition.LOW);
		setArmDirection(ArmDirection.FRONT);
		setTargetHeight(TargetHeight.INTAKE);
		isDriveToTargetRunning = false;
		targetInfo = TargetData.SIMPLETARGET; // Default value
		camDirection = CameraDirection.FRONT;
		setLedColor(LedColors.BLACK);
		setIsManual(false);
		setHatchDependence(true);
		setPipelineSelector(PipelineSelector.LEFT);
		setStartingDirection(StartingDirection.FORWARD);
		setBrakeInDisable(false);
	}

	public synchronized void setStartingDirection(StartingDirection startingDirection) {
		this.startingDirection = startingDirection;
	}

	public double getGyroOffset() {
		return this.startingDirection.getGyroOffset();
	}

	/**
	 * @return the cameraSelector
	 */
	public PipelineSelector getPipelineSelector() {
		return pipelineSelector;
	}

	/**
	 * @param cameraSelector the cameraSelector to set
	 */
	public void setPipelineSelector(PipelineSelector cameraSelector) {
		this.pipelineSelector = cameraSelector;
	}

	/**
	 * @return the isVisionFollowerRunning
	 */
	public boolean isVisionFollowerRunning() {
		return isVisionFollowerRunning;
	}

	/**
	 * @param isVisionFollowerRunning the isVisionFollowerRunning to set
	 */
	public void setVisionFollowerRunning(boolean isVisionFollowerRunning) {
		this.isVisionFollowerRunning = isVisionFollowerRunning;
	}

	public void setLimelightType(LimeLightType type) {
		this.limeLightType = type;
	}

	public LimeLightType getLimelightType() {
		return limeLightType;
	}

	public boolean getToggleVision() {
		return toggleVision;
	}

	public void setToggleVision(boolean toggleVision) {
		this.toggleVision = toggleVision;
	}

	public synchronized double getLidarDistance(int angle) {
		return LidarServer.getInstance().getDistance(angle);
	}

	private static RobotState instance = new RobotState();

	public static RobotState getInstance() {
		return instance;
	}

	public boolean getClimbState() {
		return climbDone;
	}

	public void setClimbState(boolean state) {
		this.climbDone = state;
	}

	public boolean getBrakeInDisable() {
		return brakeInDisable;
	}

	public void setBrakeInDisable(boolean state) {
		this.brakeInDisable = state;
	}

	public boolean getHasElement() {
		return hasElement;
	}

	public void setHasElement(boolean state) {
		this.hasElement = state;
	}

	public boolean getHatchDependence() {
		return hasElement;
	}

	public void setHatchDependence(boolean state) {
		this.hasElement = state;
	}

	public boolean getIsArmFlip() {
		return isArmFlip;
	}

	public void setIsArmFlip(boolean state) {
		this.isArmFlip = state;
	}

	public boolean getIsManual() {
		return isManualControl;
	}

	public void setIsManual(boolean state) {
		this.isManualControl = state;
	}

	public boolean getGrabberDeployed() {
		return grabberDeployed;
	}

	public void setGrabberDeployed(boolean state) {
		this.grabberDeployed = state;
	}

	public enum State {
		DISABLED, AUTON, TELEOP, TEST, ENABLED;
	}

	public enum AllianceColor {
		RED, BLUE;
	}

	public static enum LedColors {
		BLUEGREEN(0.79), GREEN(0.77), DARKGREEN(0.75), RED(0.61), BLUE(0.87), WHITE(0.93), BLACK(0.99), FOREST(-.91),
		BLUEBREATH(-0.15), BLUEHEARTBEAT(-0.23), GREENBREATH(0.11), GREENHEARTBEAT(0.07), GOLD(0.67), YELLOW(0.69),
		ORANGE(0.65);
		double power;

		LedColors(double power) {
			this.power = power;
		}

		public double getPower() {
			return power;
		}

	}

	public LedColors getLedColor() {
		return ledColor;
	}

	public void setLedColor(LedColors color) {
		ledColor = color;
		LED.getInstance().set(ledColor);
	}

	public static enum StartingDirection {
		FORWARD(0), BACKWARD(180), LEFT(-90), RIGHT(90);

		private double gyroOffset;

		StartingDirection(double gyroOffset) {
			this.gyroOffset = gyroOffset;
		}

		protected synchronized double getGyroOffset() {
			return gyroOffset;
		}
	}

	public static enum CameraDirection {
		FRONT(1), BACK(-1);
		double multiplier;

		CameraDirection(int multiplier) {
			this.multiplier = multiplier;
		}

		public double getMultiplier() {
			return multiplier;
		}

	}

	public static enum GameElement {
		CARGO, HATCH;
	}

	public static enum ArmDirection {
		FRONT, BACK;
	}

	public static enum TargetHeight {
		HOME, BUS, INTAKE, LOW, MIDDLE, HIGH;
	}

	public static enum LimeLightType {
		ARM, TURRET;
	}

	// // PRACTICE BOT
	// public static enum SuperStructurePreset {// FRONT_CARGO_INTAKE(-32, -58, 0.)
	// 	HATCH_HOME(-47., 90, 0.), AUTO_HOME(-45., 90, 0.), CARGO_HOME(-45, 45, 0), VIEW_AUTO_REAR(187., 90., 0.),
	// 	FRONT_CARGO_BUS(48.2, -21.8, 0.), BACK_CARGO_BUS(111, 53. + 180., 0.),
	// 	FRONT_CARGO_INTAKE(-38, -17, 0./*-30, -17, 0.0*/), FRONT_CARGO_LOW(0.0, 5.0/*4.*/, 0.),
	// 	FRONT_CARGO_MID(69., -5., 0.), // 0., -3., 0.
	// 	FRONT_CARGO_HIGH(84.0, 48.0, 12.7), BACK_CARGO_INTAKE(222, 183, 0.), BACK_CARGO_LOW(170., 180., 0.),
	// 	BACK_CARGO_MID(83., 179. - 12., 0.), BACK_CARGO_HIGH(89.0,129. - 7. /*138.*/, 13.3), FRONT_HATCH_INTAKE(-32, 85.0, 6.),
	// 	FRONT_HATCH_LOW(-32, 78, 6.), FRONT_HATCH_MID(17.0 /* 13 */, 87. - 5., /* 90 */ 0.),
	// 	FRONT_HATCH_HIGH(59., 87.0 - 5., 12.3), BACK_HATCH_INTAKE(187., 90. + 180., 0.), BACK_HATCH_LOW(187., 255., 0.),
	// 	BACK_HATCH_MID(119.5, 90. + 180., 4.), // 180., 87., 0.
	// 	BACK_HATCH_HIGH(95, 90. + 180., 0.);// 175., 77., 0

	// 	double aPos, wPos, ePos;

	// 	private SuperStructurePreset(double arm, double wrist, double extension) {
	// 		this.aPos = arm;
	// 		this.wPos = wrist;
	// 		this.ePos = extension; 
	// 	}

	// 	public double getArmPosition() {
	// 		return aPos;
	// 	}

	// 	public double getWristPosition() {
	// 		return wPos;
	// 	}

	// 	public double getExtPosition() {
	// 		return ePos;
	// 	}
	// }


	
	// COMP PRESTS DO NOT DELETE
	public static enum SuperStructurePreset {// FRONT_CARGO_INTAKE(-32, -58, 0.)
		HATCH_HOME(-47., 90, 0.), AUTO_HOME(-45., 90, 0.), CARGO_HOME(-45, 45, 0), VIEW_AUTO_REAR(187., 90., 0.),
		FRONT_CARGO_BUS(48.2, -21.8, 0.), BACK_CARGO_BUS(111, 53. + 180., 0.),
		FRONT_CARGO_INTAKE(-38, -14. + 3.5, 0./*-30, -17, 0.0*/), FRONT_CARGO_LOW(5.0, 5.0/*4.*/, 0.),
		FRONT_CARGO_MID(69., -5., 0.), // 0., -3., 0.
		FRONT_CARGO_HIGH(84.0, 48.0, 12.7), BACK_CARGO_INTAKE(222, 183, 0.), BACK_CARGO_LOW(170., 180., 0.),
		BACK_CARGO_MID(83., 171, 0.), BACK_CARGO_HIGH(89.0,129 /*138.*/, 13.3), FRONT_HATCH_INTAKE(-32, 85.0, 6.),
		FRONT_HATCH_LOW(-32, 78, 6.), FRONT_HATCH_MID(17.0 /* 13 */, 87., /* 90 */ 0.),
		FRONT_HATCH_HIGH(59., 87.0, 12.3), BACK_HATCH_INTAKE(187., 90. + 180., 0.), BACK_HATCH_LOW(187., 255., 0.),
		BACK_HATCH_MID(119.5, 90. + 180., 4.), // 180., 87., 0.
		BACK_HATCH_HIGH(95, 90. + 180., 0.);// 175., 77., 0

		double aPos, wPos, ePos;

		private SuperStructurePreset(double arm, double wrist, double extension) {
			this.aPos = arm;
			this.wPos = wrist;
			this.ePos = extension;
		}

		public double getArmPosition() {
			return aPos;
		}

		public double getWristPosition() {
			return wPos;
		}

		public double getExtPosition() {
			return ePos;
		}
	}

	public GameElement getGameElement() {
		return gameElement;
	}

	public void setGameElement(GameElement element) {
		this.gameElement = element;
		if (this.gameElement != GameElement.CARGO) {
			FFDashboard.getInstance().putString("Game Element", "HATCH");
		} else {
			FFDashboard.getInstance().putString("Game Element", gameElement.toString());
		}
	}

	public TargetHeight getTargetHeight() {
		return targetHeight;
	}

	public void setTargetHeight(TargetHeight height) {
		targetHeight = height;
		FFDashboard.getInstance().putString("Arm Level", height.toString());
	}

	public void setPositionChanged(boolean changed) {
		armPositionChanged = changed;
	}

	public boolean getPositionChanged() {
		return armPositionChanged;
	}

	public SuperStructurePreset getSuperStructurePreset() {
		return sPreset;
	}

	public void setSuperStructurePreset(SuperStructurePreset sPos) {
		this.sPreset = sPos;
		FFDashboard.getInstance().putString("Selected Preset", sPreset.toString());
	}

	public ArmDirection getArmDirection() {
		return armDirection;
	}

	public void setArmDirection(ArmDirection armDirection) {
		this.armDirection = armDirection;
		FFDashboard.getInstance().putString("Direction", armDirection.toString());
	}

	// public ExtensionPosition getExtPosition() {
	// return extPosition;
	// }

	// public void setExtPosition(ExtensionPosition ePos) {
	// extPosition = ePos;
	// }

	public void setCameraDirection(CameraDirection cameraDirection) {
		camDirection = cameraDirection;
	}

	public CameraDirection getCameraDirection() {
		return camDirection;
	}

	public synchronized void setPose(Pose pose) {
		robotPose = pose.copy();
		// System.out.println("setting low pose");
		GPS.getInstance().UpdatePositionData(robotPose);
	}

	public synchronized void setPose(Pose pose, Pose.priority priority) {
		robotPose = pose.copy();
		GPS.getInstance().UpdatePositionData(robotPose);
		if (priority == Pose.priority.HIGH) {
			System.out.println("setting pose high priority");
			GPS.getInstance().UpdatePositionData(robotPose);
			GPS.getInstance().UpdatePositionData(robotPose);
		}
		// System.out.println("setting low pose");
	}

	public synchronized Pose getRobotStateEstimate(Pose odometry, Pose camera, int cameraConfidence) {
		if (cameraConfidence == 1) {
			return Pose.weightAvg(odometry, camera, new Vector2D(0.5, 0.5));
		} else {
			return odometry;
		}

	}

	public synchronized Pose getPoseOdometry() {
		robotPose.translatePose(Robot.bot.POSE_TRANSLATION);
		return robotPose;
	}

	public void setSystemCheckRunning(boolean running) {
		isSystemCheckRunning = running;
	}

	public boolean isSystemCheckRunning() {
		return isSystemCheckRunning;
	}

	public void setGyroAngle(double angle) {
		gyroAngle = angle;
	}

	public double getGyroAngle() {
		return gyroAngle;
	}

	public State getState() {
		return robotState;
	}

	public void setState(State state) {
		robotState = state;
	}

	public enum RobotDriveMode {
		LOW, HIGH;
	}

	public RobotDriveMode getDriveMode() {
		return driveMode;
	}

	public void setDriveMode(RobotDriveMode mode) {
		driveMode = mode;
	}

	public boolean getDriveReversed() {
		return driveReversed;
	}

	private FFDashboard graphTable = new FFDashboard("Graph");

	public void sendPoseDashboardData() {
		Pose p = getPoseOdometry();
		double x = p.getX();
		double y = p.getY();
		double a = p.getTheta();
		SmartDashboard.putNumber("x", x);
		SmartDashboard.putNumber("y", y);
		SmartDashboard.putNumber("theta", a);
		graphTable.putNumber("X", x);
		graphTable.putNumber("Y", y);
		graphTable.putNumber("theta", a);
	}

	private FFDashboard ssTable = new FFDashboard("Super Structure");

	public void sendSuperStructureDashboardData() {
		ssTable.putString("Game Element", gameElement.toString());
		ssTable.putString("Target Goal", targetHeight.toString());
		ssTable.putString("Selected Preset", sPreset.toString());
		ssTable.putBoolean("IS RUNNING", intakeRunning);
	}

	private FFDashboard ledTable = new FFDashboard("LED");

	public void sendLedStatusData() {
		ledTable.putString("Current Color", getLedColor().name());
		ledTable.putNumber("Current PWM Value", getLedColor().getPower());
	}

	public void sendVisionDashboardData() {
		SmartDashboard.putString("Camera Direction", camDirection.toString());
		SmartDashboard.putString("Currently set target", getTargetData().toString());

	}

	public void toggleDriveReversed() {
		driveReversed = !driveReversed;
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public boolean getDriveProfileDone() {
		return driveProfileDone;
	}

	public void setDriveProfileDone(boolean done) {
		driveProfileDone = done;
	}

	/************************************ */
	// Vision stuff

	public static enum PipelineSelector {
		LEFT(5), RIGHT(4);

		double pipeline;

		PipelineSelector(double pipeline) {
			this.pipeline = pipeline;
		}

		public double getPipeline() {
			return this.pipeline;
		}
	}

	public void setTargetData(TargetData data) {
		targetInfo = data;
	}

	public TargetData getTargetData() {
		return targetInfo;
	}

	public double getCurrentPipeline() {
		return currentPipeline;
	}

	public void setCurrentPipeline(double pipeline) {
		currentPipeline = pipeline;
	}

	public void setIntakeRunning(boolean running) {
		intakeRunning = running;
	}

	public boolean getIntakeRunning() {
		return intakeRunning;
	}

	public void setVisionStartPosition(double[] position) {
		visionStartPosition = position;
	}

	public double[] getVisionStartPosition() {
		return visionStartPosition;
	}

	public void setDashboardEnabledStatus(boolean enabled) {
		FFDashboard.getInstance().putBoolean("isEnabled", enabled);
	}

	public void setVisionDebug(boolean debug) {
		visionDebug = debug;
	}

	public boolean isVisionDebug() {
		return visionDebug;
	}

	public void setCameraMode(double cameraMode) {
		this.cameraMode = cameraMode;
	}

	public double getCameraMode() {
		return cameraMode;
	}

	public boolean lostTarget() {
		return lostTarget;
	}

	public void setLostTarget(boolean lostTarget) {
		this.lostTarget = lostTarget;
	}

	public double[] getTargetAngles() {
		return targetAngles;
	}

	public void setTargetAngles(double[] targetAngles) {
		this.targetAngles = targetAngles;
	}

	public double getAutonToleranceAdjustment() {
		return autonToleranceAdjustment;
	}

	public void setAutonToleranceAdjustment(double autonToleranceAdjustment) {
		this.autonToleranceAdjustment = autonToleranceAdjustment;
	}

	public boolean hasCargo() {
		return Intake.getInstance().hasCargo();
	}

	public boolean hasHatch() {
		return Intake.getInstance().hasHatch();
	}

	public boolean getReleasing(){
		return releasing;
	}

	public void setReleasing(boolean releasing){
		this.releasing = releasing;
	}

}
