package org.usfirst.frc.team503.robot;

import org.usfirst.frc.team503.robot.utils.Vector2D;

public class RobotHardwareProgrammingBot extends RobotHardware {
	public RobotHardwareProgrammingBot() {
		initialize();
	}

	public final double GYRO_P = 0;
	public final double GYRO_I = 0;
	public final double GYRO_D = 0;
	public final double GYRO_TOLERANCE = 0;
	public final int NUM_LIDAR_POINTS = 720;

	public final Vector2D POSE_TRANSLATION = new Vector2D(0, 0);

	public final int leftMasterID = 25; // front left == 1 //12
	public final int leftSlaveID = 26;// 2; // back left == 2 //13
	public final int rightMasterID = 30; // front right == 3 /3
	public final int rightSlaveID = 31; // back right == 4 /2

	public final int leftTalonID = 3;
	public final int rightTalonID = 1;

	public final int right_enc_id_1 = 1;
	public final int right_enc_id_2 = 0;
	public final int left_enc_id_1 = 5;
	public final int left_enc_id_2 = 4;

	@Deprecated
	public final double WHEEL_DIAMETER = 8.0;
	public final double WHEEL_BASE_INCHES = 27.0;
	public final double WHEEL_BASE_FEET = WHEEL_BASE_INCHES / 12;
	public final double WHEEL_BASE_METERS = .6858;

	public final double CYCLE_TIME = 0.05;

	// Wheels
	public final double kDriveWheelDiameterInches = 6.25;// 7.3
	public final double kDriveWheelDiameterFeet = kDriveWheelDiameterInches / 12;
	@Deprecated
	public final int kEncoderTicksperRev = 360; // was 360 changed to 256 maybe should be 256 ??
	@Deprecated
	public final int kEncoderReadsperRev = 4;
	@Deprecated
	public final int kEncoderCountsperRev = kEncoderTicksperRev * kEncoderReadsperRev;

	public final int kEncoderUnitsPerRev = 4096;

	public final double turnMPConstant = -.03;// -.027 with no elevator
	public final boolean leftMasterReverseOutput = false;
	public final boolean leftSlaveReverseOutput = false;
	public final boolean rightMasterReverseOutput = false;
	public final boolean rightSlaveReverseOutput = false;
	public final boolean leftSensorPhase = false;
	public final boolean rightSensorPhase = true;

	public final boolean leftEncoderReverse = true;
	public final boolean rightEncoderReverse = true;

	// Lidar Constants
	public final double kLidarDriveP = 0.01;
	public final double lidarTolerance = 2.0;

	// Drive Position PID Coefficients
	public double kDrivePositionKp = 0.0005; // 0.00000001
	public double kDrivePositionKi = 0.0000000;
	public double kDrivePositionKd = 0.0000; // .015
	public double kDrivePositionKv = .009; //
	public int kDrivePositionIZone = 200;
	public double kDrivePositionRampRate = 256.0;
	public int kDrivePositionAllowableError = 2;

	// Drive Velocity PID Coefficients
	public double kDriveVelocityKp = 0.0000001;
	public double kDriveVelocityKi = 0.0000002383;
	public double kDriveVelocityKd = 0.00001;
	public double kDriveVelocityKf = 1.50220264;
	public int kDriveVelocityIZone = 0;
	public double kDriveVelocityRampRate = 0.0;
	public int kDriveVelocityAllowableError = 0;

	// PID gains for constant heading velocity control
	// Units: Error is degrees. Output is inches/second difference to
	// left/right.
	public double kDriveHeadingVelocityKp = 4.0; // 6.0;
	public double kDriveHeadingVelocityKi = 0.0;
	public double kDriveHeadingVelocityKd = 50.0;

	/*
	 * // Path following constants public double kPathFollowingLookahead = 24.0;
	 * inches public double kPathFollowingMaxVel = 120.0; // inches/sec public
	 * double kPathFollowingMaxAccel = 80.0; // inches/sec^2
	 */

	public double kLooperDt = 0.01;

	public final boolean REVERSE_LEFT_SENSOR = false;
	public final boolean REVERSE_RIGHT_SENSOR = true;
	public final boolean REVERSE_LEFT_OUTPUT = false;
	public final boolean REVERSE_RIGHT_OUTPUT = true;

	// pure pursuit constants
	public final double pathFollowingDt = 0.05;

	public final double lookAheadDistance = 35.0; // between 12 and 25
	public final double TRACK_WIDTH = 27.0;

	public final double kMaxVelocityInchesPerSec = 190.0; // 110
	public final double kMaxAccelerationInchesPerSec = 80.0; // 650
	public final double kMaxJerkInchesPerSec = 8000.0; // 10000
	public final double CENTER_OF_ROTATION_TRANSLATION = 0.0;

	public final double PurePursuit_TOLERANCE = 8.0;

	public double curvatureFudge = 1.0;
	public double curvatureFudgeRev = 1.0;

	public final double kP_PurePursuit = 0.0;// 0.01;
	public final double KP_PurePursuitReverse = 0.0;
	public final double kI_PurePursuit = 0.000;
	public final double kD_PurePursuit = 0.0000;
	public final double kV_PurePursuit = 1 / kMaxVelocityInchesPerSec;
	public final double kA_PurePursuit = 0.001;// 0.0002;
	public double faultThreshold = 80;
	public double movementThresh = 40;
	public final double turnPurePursuitKp = 0.0;// -0.01;// -0.01;//-0.02;
	public final double turnPurePursuitKi = 0.00;
	public final double turnPurePursuitKd = 0.00;

	public final double MAX_WRIST_POWER = 0;

	public void initialize() {

	}

	public boolean hasDriveCamera() {
		return false;
	}

	@Override
	public boolean hasClimber() {
		return false;
	}

	@Override
	public boolean hasCompressor() {
		return false;
	}

	@Override
	public boolean hasArm() {
		return false;
	}

	@Override
	public boolean hasWrist() {
		return false;
	}

	@Override
	public boolean hasIntake() {
		return false;
	}

	@Override
	public boolean hasLimelightTurret() {
		return false;
	}

	@Override
	public boolean hasLidar() {
		return false;
	}

	@Override
	public boolean hasLEDs() {
		return false;
	}

	@Override
	public boolean hasTwoCameras() {
		return false;
	}

	@Override
	public String getName() {
		return "ProgrammingBot";
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

	public final double LOCK_DT = 0.020;

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

	/**************************************************************************************/
	// LIBTARgetDrive Constants
	/**************************************************************************************/
	public final double libLookahead = 20;
	public int rollerIntakeID;
	public int beamBreakID;
	public double turnPurePursuitKpReverse;

	/**************************************************************************************/
	// Camera Constants
	/**************************************************************************************/
	// public double forwardLimelightPosition = 0.0;
	// public double backwardLimelightPosition = 0.0;

	// {
	// forwardLimelightXOffset = -7;

	//
	// forwardLimelightYOffset = 0; // 1
	//
	// forwardLimelightAreaThreshold = 1.8;
	// reverseLimelightAreaThreshold = 1.8;

	// }

	/**************************************************************************************/
	// Camera Constants
	/**************************************************************************************/
	public double forwardLimelightPosition = 0.1125;
	public double backwardLimelightPosition = 0.8625;

	{

		// Signs are reversed

		forwardLimelightXOffset = 6;
		forwardLimelightYOffset = 5; // 1

		reverseLimelightXOffset = -6;
		reverseLimelightYOffset = 26;

		forwardLimelightAreaThreshold = 20;
		reverseLimelightAreaThreshold = 5;

		forwardAbsoluteAngularOffset = 11;
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
		forwardPA_tolerance = 1;
		reversePA_tolerance = 1;

	}

	/**************************************************************************************/
	// Horizontal Displacement Constants
	/**************************************************************************************/
	{
		// PID
		forwardHD_kP = 0.12;
		forwardHD_kI = 0.0;
		forwardHD_kD = 0.015;
		forwardHD_kV = 0.00;

		reverseHD_kP = 0.0;
		reverseHD_kI = 0.0;
		reverseHD_kD = 0.0;
		reverseHD_kV = 0.0;

		// Setpoint
		forwardHD_positiveSetpoint = -0.1;
		forwardHD_negativeSetpoint = -0.5;

		reverseHD_positiveSetpoint = -0.1;
		reverseHD_negativeSetpoint = 0.1;

		// Turn constant;
		forwardHD_leftTurnConstant = 0.15;
		forwardHD_rightTurnConstant = 0.1;

		reverseHD_leftTurnConstant = 0.15;
		reverseHD_rightTurnConstant = 0.03;

		// Output Range
		forwardHD_outputMin = -0.5;
		forwardHD_outputMax = 0.5;

		reverseHD_outputMin = -0.5;
		reverseHD_outputMax = 0.5;

		// Tolerance
		forwardHD_tolerance = 0.2;
		reverseHD_tolerance = 0.75;

		// Deadband
		forwardHD_deadband = 3;
		reverseHD_deadband = 3;

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

		reverseAD_kP = 0.0;
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
		forwardFA_kD = 0;
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
		forwardFT_kP = 0.0625; // 0.2
		forwardFT_kI = 0;
		forwardFT_kD = 0.002;
		forwardFT_kV = 2.5;

		reverseFT_kP = 0.02;
		reverseFT_kI = 0;
		reverseFT_kD = 0.001;
		reverseFT_kV = 2.5;

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

		// Base Power KP;
		forwardFT_basePower_kP = 0.05;
		reverseFT_basePower_kP = 0.05;
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

		forwardDS_positiveSetpoint = new double[] { -9, 27 };
		forwardDS_negativeSetpoint = new double[] { 9, 27 };

		reverseDS_positiveSetpoint = new double[] { -6.5, 27 };
		reverseDS_negativeSetpoint = new double[] { 6.5, 27 };

		forwardDS_tolerance = 1;
		reverseDS_tolerance = 1;

	}


}
