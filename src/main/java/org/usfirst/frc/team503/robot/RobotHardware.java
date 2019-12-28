/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot;

import org.usfirst.frc.team503.robot.RobotState.CameraDirection;
import org.usfirst.frc.team503.robot.utils.FFDashboard;
import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

/**
 * Add your docs here.
 */
public abstract class RobotHardware {

    RobotHardware() {
        FFDashboard.getInstance().putBoolean("oneCamera", !hasTwoCameras());
    }

    public final String motionProfilingRioFolder = "/home/lvuser/MotionProfiles/";

    // Pure Pursuit
    public double DT = 0.025;
    public double kV_PurePursuitReverse;

    public double turnMP_P_Forward;
    public double turnMP_D_Forward;
    public double kA;
    public double turnMP_P_Backward;
    public double turnMP_D_Backward;

    // Intake
    public int hatchVacId;
    public int releaseId;
    public int vacuumPdpChannel;
    public int intakeMasterID;
    public int intakeSlaveID;
    public int PDP_ID;
    public int intakePdpChannel;
    public double rollerCurrentThres;
    public double vacuumCurrentThres;
    public double sensorVoltageThres;

    public double intakeStallPower;
    public double intakeOutPower;
    public double intakeVaccPower;
    public double intakeBasePower;
	public double intakePowerScalar;
	public double intakeVaccHighPower;
	public double intakeVaccLowPower; 

    // Limelight Turret
    public int limelightTurretServoID;

    // Arm
    public double kArmF;
    public double kArmP;
    public double kArmI;
    public double kArmD;

    public int kArmCruiseVel;
    public int kArmAcceleration;

    public int armMasterID;
    public int armSlaveID;

    public boolean armMasterInverted;
    public boolean armSlaveInverted;
    public boolean armMasterSensorPhase = false;
    public boolean armSlaveSensorPhase = false;

    public int extensionID;
    public int extensionResetPostion;
    public boolean extensionMotorInverted;
    public boolean extensionSensorPhase;
    public double gExtSpoolDiameter;

    // public int wristID;
    public int wristResetPosition;
    public boolean wristMotorInverted;
    public boolean wristSensorPhase;
    public double wristResetDegree;

    public int wristID;
    public int wristLimID;

    public double kWristCruiseVel;
    public double kWristAcceleration;

    public double kWristF;
    public double kWristP;
    public double kWristI;
    public double kWristD;

    public int intakeOutID;
    public int intakeInID;

    public double gWristMinLimit;
    public double gWristMaxLimit;
    public double gWristMaxLimitCargo;

    public boolean wristMasterInverted;
    public boolean wristSlaveInverted;

    public double kExtF;
    public double kExtP;
    public double kExtI;
    public double kExtD;

    public int kExtCruiseVel;
    public int kExtAcceleration;

    public double gArmExtLength;

    public double gExtGearRatio;

    public double gExtMinLim;
    public double gExtMaxLim;

    public int rollerIntakeID;

    public int beamBreakID;

    public int camServoID;

    // Climby Boi
    public int leftHandID;
    public int rightHandID;
    public int legID;
    public double climbBasePower;
    public double climbPowerRatio;

    public double climbPitchP;
    public double climbPitchI;
    public double climbPitchD;
    public double climbElevP;
    public double climbElevI;
    public double climbElevD;

    public double climbHandLim;
    public double climbLegLim;

    // Auto Climb
    public double targetPitch;
    public double elevatorTarget;

    public int LED_ID;

    public final double driveSensitivity = .75;
    /*
     * These account for sperate encoders that are not present on Prog Bot
     */

    public int right_enc_id_1;
    public int right_enc_id_2;
    public int left_enc_id_1;
    public int left_enc_id_2;

    public double gArmAngularOffset;
    public double gWristAngularOffset;
    public double gWristGroundOffset;
    public double gExtOffset;

    public final int gSlotIdx = 0;
    public final int gTimeoutMs = 0;

    public abstract String getName();

    public final double hPIDTolerance = 0.75;
    /**************************************************************************************/
    // Really obvious camera constants
    /**************************************************************************************/
    public final double distanceBetweenTargets = 11.5;
    public final double VISION_DT = 0.010;

    /**************************************************************************************/
    // Camera Constants
    /**************************************************************************************/
    protected double forwardLimelightXOffset;
    protected double forwardLimelightYOffset;

    protected double forwardAbsoluteAngularOffset = 11;
    protected double reverseAbsoluteAngularOffset = -7;

    protected double reverseLimelightXOffset;
    protected double reverseLimelightYOffset;

    public double PIVOT_P;
    public double PIVOT_I;
    public double PIVOT_D;
    public double PIVOT_TOLERANCE;

    public abstract boolean hasClimber();

    public abstract boolean hasArm();

    public abstract boolean hasWrist();

    public abstract boolean hasIntake();

    public abstract boolean hasCompressor();

    public abstract boolean hasLimelightTurret();

    public abstract boolean hasLidar();

    public abstract boolean hasLEDs();

    public abstract boolean hasTwoCameras();

    public double[] getLimelightInformation() {
        double[] output = null;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            double[] temp = { Math.hypot(forwardLimelightXOffset, forwardLimelightYOffset),
                    Math.abs(Math.atan2(forwardLimelightYOffset, forwardLimelightXOffset)) };
            output = temp;
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            double[] temp = { Math.hypot(reverseLimelightXOffset, reverseLimelightYOffset),
                    Math.abs(Math.atan2(reverseLimelightYOffset, reverseLimelightXOffset)) };
            output = temp;
        }
        return output;
    }

    protected double forwardLimelightAreaThreshold;
    protected double reverseLimelightAreaThreshold;

    public double[] getLimelightOffsets() {
        double[] output = null;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            double[] temp = { forwardLimelightXOffset, forwardLimelightYOffset };
            output = temp;
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            double[] temp = { reverseLimelightXOffset, reverseLimelightYOffset };
            output = temp;
        }

        return output;
    }

    public double getLimelightAbsoluteOffset() {
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            return forwardAbsoluteAngularOffset;
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            return reverseAbsoluteAngularOffset;
        }
        System.err.println("RobotHardware.java line 227 no camera direction");
        return 0;
    }

    public double getAreaThreshold() {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT)
            output = forwardLimelightAreaThreshold;
        else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK)
            output = reverseLimelightAreaThreshold;

        if (RobotState.getInstance().getState() == RobotState.State.AUTON) {
            //output += 2;
            output += RobotState.getInstance().getAutonToleranceAdjustment();
        } else {
            output += RobotState.getInstance().getAutonToleranceAdjustment();
        }
        return output;
    }

    /**************************************************************************************/
    // PrepareAngle Constants
    /**************************************************************************************/
    protected double forwardPA_kP, forwardPA_pivotkP, forwardPA_kI, forwardPA_kD, forwardPA_kF;
    protected double reversePA_kP, reversePA_pivotkP, reversePA_kI, reversePA_kD, reversePA_kF;

    protected double forwardPA_leftIntersection, forwardPA_rightIntersection;
    protected double reversePA_leftIntersection, reversePA_rightIntersection;

    protected double forwardPA_basePower, reversePA_basePower;

    protected double forwardPA_tolerance, reversePA_tolerance;

    protected double forwardPA_leftCheese, forwardPA_rightCheese;
    protected double reversePA_leftCheese, reversePA_rightCheese;

    /**
     * Get Horizontal Displacement drive variables
     * 
     * @return P, pivotP, I, D, F in that order
     */
    public double[] getPAPID() {
        double[] output = null;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            double[] temp = { forwardPA_kP, forwardPA_pivotkP, forwardPA_kI, forwardPA_kD, forwardPA_kF };
            output = temp;
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            double[] temp = { reversePA_kP, reversePA_pivotkP, reversePA_kI, reversePA_kD, forwardPA_kF };
            output = temp;
        }
        return output;
    }

    public double getPAIntersection() {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            if (VisionLocalizer.getInstance()
                    .translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles())[0] > 0) {
                output = forwardPA_leftIntersection;
            } else {
                output = forwardPA_rightIntersection;
            }
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            if (VisionLocalizer.getInstance()
                    .translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles())[0] > 0) {
                output = reversePA_leftIntersection;
            } else {
                output = reversePA_rightIntersection;
            }
        }
        return output;

    }

    public double getPABasePower() {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT)
            output = forwardPA_basePower;
        else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK)
            output = reversePA_basePower;
        return output;
    }

    public double getPATolerance() {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT)
            output = forwardPA_tolerance;
        else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK)
            output = reversePA_tolerance;
        return output;
    }

    public double getPAleftCheese() {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT)
            output = forwardPA_leftCheese;
        else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK)
            output = reversePA_leftCheese;
        return output;
    }

    public double getPArightCHeese() {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT)
            output = forwardPA_leftCheese;
        else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK)
            output = reversePA_rightCheese;
        return output;
    }

    public double getPACheese() {
        double estimate = VisionLocalizer.getInstance()
                .positionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles())[0];
        if (estimate > 0) {
            return getPAleftCheese();
        } else {
            return getPArightCHeese();
        }

    }

    /**************************************************************************************/
    // Horizontal Displacement Constants
    /**************************************************************************************/
    protected double forwardHD_kP, forwardHD_kI, forwardHD_kD, forwardHD_kV;
    protected double reverseHD_kP, reverseHD_kI, reverseHD_kD, reverseHD_kV;

    protected double forwardHD_positiveSetpoint, forwardHD_negativeSetpoint;
    protected double reverseHD_positiveSetpoint, reverseHD_negativeSetpoint;

    protected double forwardHD_leftTurnConstant, forwardHD_rightTurnConstant;
    protected double reverseHD_leftTurnConstant, reverseHD_rightTurnConstant;

    protected double forwardHD_outputMin, forwardHD_outputMax;
    protected double reverseHD_outputMin, reverseHD_outputMax;

    protected double forwardHD_tolerance, reverseHD_tolerance;

    protected double forwardHD_deadband, reverseHD_deadband;

    // Output range

    // tolerance

    /**
     * Get Horizontal Displacement drive variables
     * 
     * @return P, I, D, V, in that order
     */
    public double[] getHDPIDV() {
        double[] output = null;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            double[] temp = { forwardHD_kP, forwardHD_kI, forwardHD_kD, forwardHD_kV };
            output = temp;
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            double[] temp = { reverseHD_kP, reverseHD_kI, reverseHD_kD, reverseHD_kV };
            output = temp;
        }
        return output;
    }

    public double getHDSetpoint(double position) {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            if (position < 0)
                output = forwardHD_positiveSetpoint;
            else if (position > 0)
                output = forwardHD_negativeSetpoint;
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            if (position < 0)
                output = reverseHD_positiveSetpoint;
            else if (position > 0)
                output = reverseHD_negativeSetpoint;
        }
        if (RobotState.getInstance().isVisionDebug())
            System.out.println("Setpoint set to: " + output);

        return output;
    }

    public double getHDTurnConstant(double position) {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            if (position > 0)
                output = forwardHD_leftTurnConstant;
            else if (position < 0)
                output = forwardHD_rightTurnConstant;
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            if (position > 0)
                output = reverseHD_leftTurnConstant;
            else if (position < 0)
                output = reverseHD_leftTurnConstant;
        }
        return output;

    }

    public double[] getHDOutputRange() {
        double[] output = null;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            double[] temp = { forwardHD_outputMin, forwardHD_outputMax };
            output = temp;
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            double[] temp = { reverseHD_outputMin, reverseHD_outputMax };
            output = temp;
        }
        return output;
    }

    public double getHDTolerance() {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT)
            output = forwardHD_tolerance;
        else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK)
            output = reverseHD_tolerance;
        return output;
    }

    public double getHDDeadband() {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT)
            output = forwardHD_deadband;
        else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK)
            output = forwardHD_deadband;
        return output;
    }

    /**************************************************************************************/
    // Angular Displacement Constants
    /**************************************************************************************/
    protected double forwardAD_kP, forwardAD_kI, forwardAD_kD, forwardAD_kV;
    protected double reverseAD_kP, reverseAD_kI, reverseAD_kD, reverseAD_kV;

    protected double forwardAD_outputMin, forwardAD_outputMax;
    protected double reverseAD_outputMin, reverseAD_outputMax;

    protected double forwardAD_basePower, reverseAD_basePower;

    /**
     * Get Angular Displacement drive variables
     * 
     * @return P, I, D, V, in that order
     */
    public double[] getADPIDV() {
        double[] output = null;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            double[] temp = { forwardAD_kP, forwardAD_kI, forwardAD_kD, forwardAD_kV };
            output = temp;
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            double[] temp = { reverseAD_kP, reverseAD_kI, reverseAD_kD, reverseAD_kV };
            output = temp;
        }
        return output;
    }

    public double[] getADOutputRange() {
        double[] output = null;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            double[] temp = { forwardAD_outputMin, forwardAD_outputMax };
            output = temp;
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            double[] temp = { reverseAD_outputMin, reverseAD_outputMax };
            output = temp;
        }
        return output;
    }

    public double getADBasePower() {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            output = forwardAD_basePower;
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            output = reverseAD_basePower;
        }
        return output;

    }

    /**************************************************************************************/
    // FinalizeAngle Constants
    /**************************************************************************************/
    protected double forwardFA_kP, forwardFA_pivotkP, forwardFA_kI, forwardFA_kD, forwardFA_kF;
    protected double reverseFA_kP, reverseFA_pivotkP, reverseFA_kI, reverseFA_kD, reverseFA_kF;

    protected double forwardFA_leftIntersection, forwardFA_rightIntersection;
    protected double reverseFA_leftIntersection, reverseFA_rightIntersection;

    protected double forwardFA_basePower, reverseFA_basePower;

    protected double forwardFA_tolerance, reverseFA_tolerance;

    protected double forwardFA_leftCheese, forwardFA_rightCheese;
    protected double reverseFA_leftCheese, reverseFA_rightCheese;

    /**
     * Get Horizontal Displacement drive variables
     * 
     * @return P, pivotP, I, D, F in that order
     */
    public double[] getFAPID() {
        double[] output = null;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            double[] temp = { forwardFA_kP, forwardFA_pivotkP, forwardFA_kI, forwardFA_kD, forwardFA_kF };
            output = temp;
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            double[] temp = { reverseFA_kP, reverseFA_pivotkP, reverseFA_kI, reverseFA_kD, forwardFA_kF };
            output = temp;
        }
        return output;
    }

    public double getFAIntersection() {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            if (VisionLocalizer.getInstance()
                    .translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles())[0] > 0) {
                output = forwardFA_leftIntersection;
            } else {
                output = forwardFA_rightIntersection;
            }
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            if (VisionLocalizer.getInstance()
                    .translatedPositionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles())[0] > 0) {
                output = reverseFA_leftIntersection;
            } else {
                output = reverseFA_rightIntersection;
            }
        }
        return output;

    }

    public double getFABasePower() {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT)
            output = forwardFA_basePower;
        else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK)
            output = reverseFA_basePower;
        return output;
    }

    public double getFATolerance() {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT)
            output = forwardFA_tolerance;
        else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK)
            output = reverseFA_tolerance;
        return output;
    }

    public double getFAleftCheese() {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT)
            output = forwardFA_leftCheese;
        else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK)
            output = reverseFA_leftCheese;
        return output;
    }

    public double getFArightCHeese() {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT)
            output = forwardFA_leftCheese;
        else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK)
            output = reverseFA_rightCheese;
        return output;
    }

    public double getFACheese() {
        double estimate = VisionLocalizer.getInstance()
                .positionEstimate(VisionLocalizer.getInstance().getIndividualTargetAngles())[0];
        if (estimate > 0) {
            return getFAleftCheese();
        } else {
            return getFArightCHeese();
        }

    }

    // FollowTarget Constants
    protected double forwardFT_kP, forwardFT_kI, forwardFT_kD, forwardFT_kV;
    protected double reverseFT_kP, reverseFT_kI, reverseFT_kD, reverseFT_kV;

    protected double forwardFT_basePower, reverseFT_basePower;

    protected double forwardFT_tolerance, reverseFT_tolerance;

    protected double forwardFT_outputMin, forwardFT_outputMax;
    protected double reverseFT_outputMin, reverseFT_outputMax;

    protected double forwardFT_basePower_kP, reverseFT_basePower_kP;

    /**
     * Get FollowTarget drive variables
     * 
     * @return P, I, D, V in that order
     */
    public double[] getFTPIDV() {
        double[] output = null;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            double[] temp = { forwardFT_kP, forwardFT_kI, forwardFT_kD, forwardFT_kV };
            output = temp;
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            double[] temp = { reverseFT_kP, reverseFT_kI, reverseFT_kD, forwardFT_kV };
            output = temp;
        }
        return output;
    }

    public double getFTBasePower() {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT)
            output = forwardFT_basePower;
        else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK)
            output = reverseFT_basePower;
        return output;
    }

    public double getFTTolerance() {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT)
            output = forwardFT_tolerance;
        else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK)
            output = reverseFT_tolerance;
        return output;
    }

    public double[] getFTOutputRange() {
        double[] output = null;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            double[] temp = { forwardFT_outputMin, forwardFT_outputMax };
            output = temp;
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            double[] temp = { reverseFT_outputMin, reverseFT_outputMax };
            output = temp;
        }
        return output;
    }

    public double getFTBasePowerP() {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT)
            output = forwardFT_basePower_kP;
        else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK)
            output = reverseFT_basePower_kP;
        return output;
    }

    /**************************************************************************************/
    // DriveStraight
    /**************************************************************************************/
    protected double forwardDS_kP, forwardDS_kI, forwardDS_kD;
    protected double reverseDS_kP, reverseDS_kI, reverseDS_kD;

    protected double forwardDS_outputMin, forwardDS_outputMax;
    protected double reverseDS_outputMin, reverseDS_outputMax;

    protected double forwardDS_basePower, reverseDS_basePower;

    protected double[] forwardDS_positiveSetpoint, forwardDS_negativeSetpoint;
    protected double[] reverseDS_positiveSetpoint, reverseDS_negativeSetpoint;

    protected double forwardDS_tolerance, reverseDS_tolerance;

    /**
     * Get DriveStraight drive variables
     * 
     * @return P, I, D, in that order
     */
    public double[] getDSPID() {
        double[] output = null;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            double[] temp = { forwardDS_kP, forwardDS_kI, forwardDS_kD };
            output = temp;
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            double[] temp = { reverseDS_kP, reverseDS_kI, reverseDS_kD };
            output = temp;
        }
        return output;
    }

    public double[] getDSOutputRange() {
        double[] output = null;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            double[] temp = { forwardDS_outputMin, forwardDS_outputMax };
            output = temp;
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            double[] temp = { reverseDS_outputMin, reverseDS_outputMax };
            output = temp;
        }
        return output;
    }

    public double getDSBasePower() {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            output = forwardDS_basePower;
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            output = reverseDS_basePower;
        }
        return output;

    }

    public double[] getDSSetpoint(double position) {
        double[] output = new double[2];
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT) {
            if (position < 0)
                output = forwardDS_positiveSetpoint;
            else if (position > 0)
                output = forwardDS_negativeSetpoint;
        } else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK) {
            if (position < 0)
                output = reverseDS_positiveSetpoint;
            else if (position > 0)
                output = reverseDS_negativeSetpoint;
        }

        return output;
    }

    public double getDSTolerance() {
        double output = 0;
        if (RobotState.getInstance().getCameraDirection() == CameraDirection.FRONT)
            output = forwardDS_tolerance;
        else if (RobotState.getInstance().getCameraDirection() == CameraDirection.BACK)
            output = reverseDS_tolerance;
        return output;
    }

}