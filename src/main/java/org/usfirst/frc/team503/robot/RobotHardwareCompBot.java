package org.usfirst.frc.team503.robot;

import org.usfirst.frc.team503.robot.utils.Vector2D;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotHardwareCompBot extends RobotHardware {
	public static final double VELOCITY_ZERO_TOLERANCE = 3;
	public final int leftMasterID = 3; // front left == 1
	public final int leftSlaveID = 1; // back left == 2
	public final int rightMasterID = 14; // front right == 3
	public final int rightSlaveID = 15; // back right == 4

	public final int kCurrentLimit = 45;//45;
	public final double kRampRate = 0.0;
	/**
	 * public final int leftMasterID = 0; // front left == 1 public final int
	 * leftSlaveID = 1; // back left == 2 public final int rightMasterID = 14; //
	 * front right == 3 public final int rightSlaveID = 15; // back right == 4
	 */
	public final int right_enc_id_1 = 2;
	public final int right_enc_id_2 = 3;

	public final int left_enc_id_1 = 0;
	public final int left_enc_id_2 = 1;

	public final double turnSensitivity = 1.00;

	
	// LimeLight Servo
	public final int limelightTurretServoID = 2;

	public double forwardLimelightPosition = 0.1125;
	public double backwardLimelightPosition = 0.8625;

	// Arm
	public final int armMasterID = 11;
	public final int armSlaveID = 10;
	public final int kArmCruiseVel = 254;
	public final int kArmAcceleration = 503;

	public final double kArmF = 1023 / 285;
	public final double kArmP = 3.0;
	public final double kArmI = 0;
	public final double kArmD = 10;

	public final double gArmAngularOffset = 47.0;

	public final boolean armMasterInverted = false;
	public final boolean armSlaveInverted = false;

	@Deprecated
	public final double WHEEL_DIAMETER = 8.0;
	public final double WHEEL_BASE_INCHES = 25.375;
	public final double WHEEL_BASE_FEET = WHEEL_BASE_INCHES / 12;
	public final double WHEEL_BASE_METERS = .6858;

	public final SerialPort.Port laserPort = SerialPort.Port.kUSB;

	// Wrist/Intake
	public final int rollerIntakeID = 0;

	public final int intakePdpChannel = 9;
	public final int vacuumPdpChannel = 11;//

	public final double intakeBasePower = -0.8;
	public final double intakePowerScalar = 0.2;
	public final double intakeStallPower = -0.25;
	public final double intakeOutPower = 0.8;
	public final double intakeVaccPower = 0.6;
	public final double intakeVaccHighPower = 0.8;
	public final double intakeVaccLowPower = 0.0; 

	public final int hatchVacId = 1;
	public final int releaseId = 2;
	// public final int intakeInID = 2;

	public final double rollerCurrentThres = 45.0;//30.0;
	public final double  sensorVoltageThres = 1.2;
	public final double pressureThres = 0.6;
	public final double vacuumCurrentThres = 0.9;

	public final int wristID = 13;
	public final int wristLimID = 9;

	public final double kWristCruiseVel = 333.0;
	public final double kWristAcceleration = 3330.0;

	public final double kWristF = 1023 / 333.0;
	public final double kWristP = 1.0;// 1.0;
	public final double kWristI = 0;// 0.000;
	public final double kWristD = 40;// 1.0;

	public final double gWristMinLimit = -116.0;
	public final double gWristMaxLimit = 93.0;

	public final double gWristMaxLimitCargo = 93.0 - 15.;

	public final double gWristAngularOffset = 135;
	public final double gWristGroundOffset = 90.;

	public final boolean wristMasterInverted = false;
	public final boolean wristSlaveInverted = false;

	public final boolean wristSensorPhase = true;
	public final double MAX_WRIST_POWER = 149.0;

	// Extension
	public final int extensionID = 12;

	public final boolean extensionSensorPhase = true;

	public final double kExtF = 1023 / 3148;
	public final double kExtP = 0.8;//0.40;
	public final double kExtI = 0;
	public final double kExtD = 0;

	public final int kExtCruiseVel = 3148;
	public final int kExtAcceleration = 31480;

	public final double gExtGearRatio = 1.0;
	public final double gExtSpoolDiameter = 1.353;
	public final double gExtOffset = 0.; // extension starts 1.5 inches out to prevent grinding
	public final double gExtMinLim = 0.;
	public final double gExtMaxLim = 13.0;

	public final double gArmExtLength = 24.75;

	// Climby Boi
	public final int leftHandID = 2;
	public final int rightHandID = 12;
	public final int legID = 13;

	public final double climbPitchP = 0.06;//0.06
	public final double climbPitchI = 0.;//
	public final double climbPitchD = 0.01;//
	public final double climbElevP = 0.32;//
	public final double climbElevI = 0.;//
	public final double climbElevD = 0.;//

	public final double climbHandLim = -1.;//
	public final double climbLegLim  = -0.9;//

	public final double climbBasePower = -0.9;
	public final double climbPowerRatio = 0.25;

	// Auto Climb 
	public final double targetPitch = -24.0;// -21
	public final double targetPitch2 = -11.0;// -21
	public final double elevatorTarget = -1.0;//

	public final double kLidarDriveP = 0.001;
	public final double lidarTolerance = 5.0;
	// LiDAR

	// LEDs
	public final int LED_ID = 3;

	// Power Distribution Panel
	public final int PDP_ID = 0;

	// Wheels
	public final double kDriveWheelDiameterInches = 6.125;
	public final double kDriveWheelDiameterFeet = kDriveWheelDiameterInches / 12;
	public final double kDriveWheelDiameterMeters = 0.18542;
	@Deprecated
	public final int kEncoderTicksperRev = 360; // was 360 changed to 256 maybe should be 256 ??
	@Deprecated
	public final int kEncoderReadsperRev = 4;
	@Deprecated
	public final int kEncoderCountsperRev = kEncoderTicksperRev * kEncoderReadsperRev;

	public final int kEncoderUnitsPerRev = 4096;

	// Low gear values

	public final double kMaxAccelerationInchesPerSec = 40;
	public final double kMaxJerkInchesPerSec = 9000;
	public final double CENTER_OF_ROTATION_TRANSLATION = 3.0;

	public final boolean leftMasterReverseOutput = true;
	public final boolean leftSlaveReverseOutput = true;
	public final boolean rightMasterReverseOutput = true;
	public final boolean rightSlaveReverseOutput = true;
	public final boolean leftSensorPhase = false;
	public final boolean rightSensorPhase = true;

	public final double lowGearing = 90.0 / 7.0;
	public final double highGearing = 6.25;
	public final double upShiftRPM = 3811.28972;
	public final double downShiftRPM = 1852.71028;
	public final double shiftToleranceRPM = 40.0;
	public final double motorShiftRPM = 296.4336449;

	// Drive Position PID Coefficients
	public int kDrivePositionIZone = 200;
	public double kDrivePositionRampRate = 256.0;
	public int kDrivePositionAllowableError = 2;

	public final int driveSolenoidID1 = 0;
	public final int driveSolenoidID2 = 1;
	public final int ptoID1 = 2;
	public final int ptoID2 = 3;
	public final double driverJoystickTolerance = 0.1;
	public final int servoPort1 = 0;
	public final int servoPort2 = 3; 

	public final double sensorMarginThreshold = 20.0;
	public final double laserLowThreshold = 7.5;

	public final double PIVOT_P = 0.0065; //0.0065
	public final double PIVOT_I = 0.0002;
	public final double PIVOT_D = 0.001;//0.0022;
	public final double PIVOT_TOLERANCE = 3.0;

	public final Vector2D POSE_TRANSLATION = new Vector2D(0.001, 8.0);
	// pure pursuit constants
	public final double kMaxVelocityInchesPerSec = 300;//320;// 196;// 260;// 230;// 237.5;
	public final double kMaxVelocityIncehsPerSecReverse = 150;
	public final double pathFollowingDt = 0.05;

	public final double lookAheadDistance = 26.0;// .0;// 16.0; // between 12 and 25
	public final double TRACK_WIDTH = 26.0;

	public final double PurePursuit_TOLERANCE = 8.0;

	public final double kP_PurePursuit = 0.00015;// 0.0001;//0.03;//0.01;// 0.01;//0.001;//0.001;//0.001;//0.015;
	public final double KP_PurePursuitReverse = 0.00015;// 0.00015;//0.035;
	public final double kI_PurePursuit = 0.000;
	public final double kD_PurePursuit = 0.0;// 0.0001;
	public final double kV_PurePursuit = 1 / kMaxVelocityInchesPerSec;
	public final double kA_PurePursuit = 0.0015;// 0.003;//0.0035;// 0.0 03;
	public final double curvatureFudge = 1.2;// 1.15;//1.2;//2;// 0.6;//0.99;
	public final double curvatureFudgeRev = 1.0;// 1.15;
	public final double turnPurePursuitKp = 0.0;// 0.03;//-0.02;//-0.01;// -0.01;//-0.005//-0.01;//-0.02;//-0.01;
	public final double turnPurePursuitKpReverse = 0.0;// 0.03;
	public final double turnPurePursuitKi = 0.00;
	public final double turnPurePursuitKd = 0.00;
	public double kDrivePositionKp = 0.0;
	public double kDrivePositionKi = 0.0;
	public double kDrivePositionKd = 0.0;
	public double turnMPConstant = 0;
	public double kA = 0.00;
	
	public double faultThreshold = 80;
	public double movementThresh = 40;

	// motion profile constants
	public double turnMP_P_Backward = -0.015;
	public double turnMP_D_Backward = -0.001;

	public double turnMP_P_Forward = -0.01;
	// -0.003;
	public double turnMP_D_Forward = -0.000;// -0.0001;

	public String ARM_LIMELIGHT = "limelight";
	public String TURRET_LIMELIGHT = "limelight-b";

	public void initialize() {
	}

	public boolean hasDriveCamera() {
		return false;
	}

	@Override
	public boolean hasClimber() {
		return true;
	}

	@Override
	public boolean hasCompressor() {
		return true;
	}

	@Override
	public boolean hasArm() {
		return true;
	}

	@Override
	public boolean hasWrist() {
		return true;
	}

	@Override
	public boolean hasIntake() {
		return true;
	}

	@Override
	public boolean hasLimelightTurret() {
		return true;
	}

	@Override
	public boolean hasLidar() {
		return true;
	}

	@Override
	public boolean hasLEDs() {
		return true;
	}

	public void logSmartDashboard() {
		SmartDashboard.putString("Current Robot", getName());
	}

	@Override
	public boolean hasTwoCameras() {
		return true;
	}

	@Override
	public String getName() {
		return "CompBot";
	}

	/**************************************************************************************/
	// boonkgang
	/**************************************************************************************/

	public final double modBoonkgang = 6;
	/**************************************************************************************/
	// Camera information constants
	/**************************************************************************************/

	public final double viewportWidth = 1.0190508989888576210274138225013; // Camera constant calcs on website
	public final double viewportHeight = 0.74776935896960937288938006502407; // are online

	public final double targetGap = 11.5;;

	public final double VISION_DT = 0.020;

	public final double ySlope = 0.8105;
	public final double yConstant = 3.946;

	/**************************************************************************************/
	// Vision pure pursuit constants
	/**************************************************************************************/

	// Path generation
	public final double endCoordinateSubtract = 20;

	public final double magnitude = 10;

	public final int numTimesToSmooth = 3;

	// Smoothing
	public final double weightData = 0.2;
	public final double weightSmooth = 0.1;
	public final double smoothTolerance = 0.0000001;

	// Velocity generation
	public final double pathGenKVel = 20;
	public final double pathGenMaxVel = 300;
	public final double pathGenMaxAcc = 50;

	public int hatchVacId2 = 4;
	public int release2Id = 3;

	/**************************************************************************************/
	// LIBTARgetDrive Constants
	/**************************************************************************************/
	public final double libLookahead = 20;
	public int beamBreakID = 0;
	public int pressureSensorID = 1;

	/**************************************************************************************/
	// Camera Constants
	/**************************************************************************************/
	{
		// Signs are reversed

		forwardLimelightXOffset = 6;
		forwardLimelightYOffset = 5; // 1

		reverseLimelightXOffset = -6;
		reverseLimelightYOffset = -9;

		forwardLimelightAreaThreshold = 13.0;//14.0;//13.0;//20.0;//13.0;
		reverseLimelightAreaThreshold = 2.5;

		forwardAbsoluteAngularOffset = 13.6;
		reverseAbsoluteAngularOffset = -7;

	}

	/**************************************************************************************/
	// PrepareAngle Constants
	/**************************************************************************************/
	{
		// PID
		forwardPA_kP = 0.015;
		forwardPA_pivotkP = 0.0069; 
  		forwardPA_kI = 0.0;
		forwardPA_kD = 0;
		forwardPA_kF = 1;

		reversePA_kP = 0.01;
		reversePA_pivotkP = 0.0069;
		reversePA_kI = 0.0;
		reversePA_kD = 0;
		forwardPA_kF = 1;

		// Intersection
		forwardPA_leftIntersection = 45;
		forwardPA_rightIntersection = 55;

		reversePA_leftIntersection = 0;
		reversePA_rightIntersection = 0;

		// Base Power
		forwardPA_basePower = 0;
		reversePA_basePower = 0;

		// Tolerance
		forwardPA_tolerance = 6;
		reversePA_tolerance = 6;

	}

	/**************************************************************************************/
	// Horizontal Displacement Constants
	/**************************************************************************************/
	{
		// PID
		forwardHD_kP = 0.1;
		forwardHD_kI = 0.0;
		forwardHD_kD = 0.015;
		forwardHD_kV = 0.00;

		reverseHD_kP = 0.1;
		reverseHD_kI = 0.0;
		reverseHD_kD = 0.015;
		reverseHD_kV = 0.0;

		// Setpoint
		forwardHD_positiveSetpoint = -0.1;
		forwardHD_negativeSetpoint = -0.1;

		reverseHD_positiveSetpoint = -0.1;
		reverseHD_negativeSetpoint = -0.1;

		// Turn constant;
		forwardHD_leftTurnConstant = 0.07;
		forwardHD_rightTurnConstant = 0.07;

		reverseHD_leftTurnConstant = 0.07;
		reverseHD_rightTurnConstant = 0.07;

		// Output Range
		forwardHD_outputMin = -0.2;
		forwardHD_outputMax = 0.2;

		reverseHD_outputMin = -0.5;
		reverseHD_outputMax = 0.5;

		// Tolerance
		forwardHD_tolerance = 0.2;
		reverseHD_tolerance = 0.2;

		// Deadband
		forwardHD_deadband = 0.5;
		reverseHD_deadband = 0.5;

	}

	/**************************************************************************************/
	// Angular Displacement Constants
	/**************************************************************************************/
	{
		// PID
		forwardAD_kP = 0.02;
		forwardAD_kI = 0.0;
		forwardAD_kD = 0.0;
		forwardAD_kV = 0.2;

		reverseAD_kP = 0.02;
		reverseAD_kI = 0.0;
		reverseAD_kD = 0.0;
		reverseAD_kV = 0.0;

		// Output Range
		forwardAD_outputMin = -0.3;
		forwardAD_outputMax = 0.3;

		reverseAD_outputMin = -0.3;
		reverseAD_outputMax = 0.3;

	}

	/**************************************************************************************/
	// PrepareAngle Constants
	/**************************************************************************************/
	{
		// PID
		forwardFA_kP = 0.015;
		forwardFA_pivotkP = 0.005;
		forwardFA_kI = 0.0;
		forwardFA_kD = 0;// 0.00125;
		forwardFA_kF = 1;

		reverseFA_kP = 0.01;
		reverseFA_pivotkP = 0.0075;
		reverseFA_kI = 0.0;
		reverseFA_kD = 0;
		forwardFA_kF = 1;

		// Intersection
		forwardFA_leftIntersection = 45;
		forwardFA_rightIntersection = 55;

		reverseFA_leftIntersection = 0;
		reverseFA_rightIntersection = 0;

		// Base Power
		forwardFA_basePower = 0;
		reverseFA_basePower = 0;

		// Tolerance
		forwardFA_tolerance = 2;
		reverseFA_tolerance = 3;

	}

	/**************************************************************************************/
	// FollowTarget Constants
	/**************************************************************************************/
	{
		// PID
		forwardFT_kP = 0.0225;
		forwardFT_kI = 0;
		forwardFT_kD = 0.002;
		forwardFT_kV = 2.0;

		reverseFT_kP = 0.02;
		reverseFT_kI = 0;
		reverseFT_kD = 0.001;
		reverseFT_kV = 2.0;

		// BasePower
		forwardFT_basePower = 0.5;
		reverseFT_basePower = 0.5;

		// Tolerance
		forwardFT_tolerance = 0.5;
		reverseFT_tolerance = 0.5;

		// Output Range
		forwardFT_outputMin = -0.55;
		forwardFT_outputMax = 0.55;
		reverseFT_outputMin = -0.55;
		reverseFT_outputMax = 0.55;
	}

	/**************************************************************************************/
	// Drive Straight Constants
	/**************************************************************************************/
	{
		// PID
		forwardDS_kP = 0.035;
		forwardDS_kI = 0.0;
		forwardDS_kD = 0.015;

		reverseDS_kP = 0.035;
		reverseDS_kI = 0.0;
		reverseDS_kD = 0.015;

		// Output Range
		forwardDS_outputMin = -0.6;
		forwardDS_outputMax = 0.6;

		reverseDS_outputMin = -0.6;
		reverseDS_outputMax = 0.6;

		forwardDS_positiveSetpoint = new double[] { -6.5, 27 };
		forwardDS_negativeSetpoint = new double[] { 6.5, 27 };

		reverseDS_positiveSetpoint = new double[] { -6.5, 27 };
		reverseDS_negativeSetpoint = new double[] { 6.5, 27 };

		forwardDS_tolerance = 1;
		reverseDS_tolerance = 1;

	}
}
