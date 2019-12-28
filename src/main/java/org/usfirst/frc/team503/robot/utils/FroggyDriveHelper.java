/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.utils;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.subsystems.Drive;

/**
 * Teleop drive utility for enhanced drive control
 */
public class FroggyDriveHelper {

    private Drive drive;

    public FroggyDriveHelper() {
        this.drive = Drive.getInstance();
    }

    private static FroggyDriveHelper instance = new FroggyDriveHelper();

    public static FroggyDriveHelper getInstance() {
        return instance;
    }

    private static final double nMoveDeadband = 0.02;
    private static final double nTurnDeadband = 0.00;

    // These factor determine how fast the wheel traverses the "non linear" sine
    // curve
    private static final double nTurnNonLinearity = 0.65;

    private static final double nNegInertiaThreshold = 0.65;
    private static final double nNegInertiaTurnScalar = 3.5;
    private static final double nNegInertiaCloseScalar = 4.0;
    private static final double nNegInertiaFarScalar = 5.0;

    private static final double nSensitivity = Robot.bot.turnSensitivity;//Comp is 1.0

    // Constants for turn sensitivity oblique asymptote
    private static final double nMagScalar = 10.0;
    private static final double nHorizRev = 0.02;
    private static final double nHorizFwd = -0.02;
    private static final double nVertTrans = 0.2;
    private static final double nLinearSlope = 10.0;

    private static final double nQuickStopDeadband = 0.2;
    private static final double nQuickStopWeight = 0.1;
    private static final double nQuickStopScalar = 5.0;

    private double nOldTurn = 0.0;
    private double nQuickStopAccumlator = 0.0;
    private double nNegInertiaAccumlator = 0.0;
    private double nMoveCurvatureSmoothener = 0.0;

    public void froggyDrive(double move, double turn, boolean isQuickTurn) {

        // System.out.println("Sensitivity "+nSensitivity);

        turn = handleDeadband(turn, nTurnDeadband);
        move = handleDeadband(move, nMoveDeadband);

        double negInertia = turn - nOldTurn;
        nOldTurn = turn;

        final double denominator = Math.sin(Math.PI / 2.0 * nTurnNonLinearity);
        // Apply a sin function that's scaled to make it feel better.
        turn = Math.sin(Math.PI / 2.0 * nTurnNonLinearity * turn) / denominator;
        turn = Math.sin(Math.PI / 2.0 * nTurnNonLinearity * turn) / denominator;

        double leftPwm, rightPwm, overPower;

        double angularPower;
        double linearPower;

        // Accounts for negative inertia
        double negInertiaScalar;
        if (turn * negInertia > 0) {
            // If we are moving away from 0.0, aka, trying to get more wheel.
            negInertiaScalar = nNegInertiaTurnScalar;
        } else {
            // Otherwise, we are attempting to go back to 0.0.
            if (Math.abs(turn) > nNegInertiaThreshold) {
                negInertiaScalar = nNegInertiaFarScalar;
            } else {
                negInertiaScalar = nNegInertiaCloseScalar;
            }
        }

        double negInertiaPower = negInertia * negInertiaScalar;
        nNegInertiaAccumlator += negInertiaPower;

        turn = turn + nNegInertiaAccumlator;
        if (nNegInertiaAccumlator > 1) {
            nNegInertiaAccumlator -= 1;
        } else if (nNegInertiaAccumlator < -1) {
            nNegInertiaAccumlator += 1;
        } else {
            nNegInertiaAccumlator = 0;
        }
        linearPower = move;

        // Case for quick turn function with sped up steer control
        if (isQuickTurn) {
            if (Math.abs(linearPower) < nQuickStopDeadband) {
                double alpha = nQuickStopWeight;
                nQuickStopAccumlator = (1 - alpha) * nQuickStopAccumlator + alpha * limit(turn, 1.0) * nQuickStopScalar;
            }
            overPower = 1.0;
            angularPower = turn;
        } else {
            overPower = 0.0;
            if (linearPower >= 0) {
                nMoveCurvatureSmoothener = (nMagScalar * Math.pow(linearPower - nHorizFwd, 2) + nVertTrans)
                        / (nLinearSlope * (linearPower - nHorizFwd));

                if (nMoveCurvatureSmoothener > 1.0) {
                    nMoveCurvatureSmoothener = 1.0;
                }
            } else {
                nMoveCurvatureSmoothener = (nMagScalar * Math.pow(linearPower - nHorizRev, 2) + nVertTrans)
                        / (nLinearSlope * (linearPower - nHorizFwd));

                if (nMoveCurvatureSmoothener < -1.0) {
                    nMoveCurvatureSmoothener = 1.0;
                }
            }
            // System.out.println(nMoveCurvatureSmoothener);
            if (!(linearPower == 0)) {
                angularPower = Math.abs(move) * turn * nSensitivity * Math.abs(nMoveCurvatureSmoothener / linearPower)
                        - nQuickStopAccumlator;
            } else {
                angularPower = turn * 0.6 * nSensitivity - nQuickStopAccumlator;
            }
            if (nQuickStopAccumlator > 1) {
                nQuickStopAccumlator -= 1;
            } else if (nQuickStopAccumlator < -1) {
                nQuickStopAccumlator += 1;
            } else {
                nQuickStopAccumlator = 0.0;
            }
        }

        rightPwm = leftPwm = linearPower;
        leftPwm += angularPower;
        rightPwm -= angularPower;
        // System.out.println(linearPower + " " + angularPower);

        if (leftPwm > 1.0)

        {
            rightPwm -= overPower * (leftPwm - 1.0);
            leftPwm = 1.0;
        } else if (rightPwm > 1.0) {
            leftPwm -= overPower * (rightPwm - 1.0);
            rightPwm = 1.0;
        } else if (leftPwm < -1.0) {
            rightPwm += overPower * (-1.0 - leftPwm);
            leftPwm = -1.0;
        } else if (rightPwm < -1.0) {
            leftPwm += overPower * (-1.0 - rightPwm);
            rightPwm = -1.0;
        }
        if (RobotState.getInstance().getState() == RobotState.State.AUTON){
            double temp = leftPwm;
            leftPwm = rightPwm;
            rightPwm = temp;
        }
        drive.tankDrive(leftPwm, rightPwm);
    }

    public double handleDeadband(double value, double deadband) {
        // if (Math.abs(value) > deadband) {
        //     if (value > 0.0) {
        //         return (value - deadband) / (1.0 - deadband);
        //     } else {
        //         return (value + deadband) / (1.0 - deadband);
        //     }
        // } else {
        //     return 0.0;
        // }
        return (Math.abs(value) > Math.abs(deadband)) ? value : 0.0;
    }

    public double limit(double val, double lim) {
        return (Math.abs(val) < lim) ? val : lim * (val < 0 ? -1 : 1);
    }

}