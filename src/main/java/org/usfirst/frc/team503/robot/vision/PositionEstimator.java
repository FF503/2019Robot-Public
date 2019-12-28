package org.usfirst.frc.team503.robot.vision;

import org.usfirst.frc.team503.robot.OI;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.subsystems.Gyro;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PositionEstimator {
    private boolean isRocket = false; // get right driver joystick button
    private boolean selectorLeft = false;
    private boolean selectorRight = false;
    private boolean selectorCenter = false;

    public TargetData guessPosition() {
        double angle = VisionLocalizer.getInstance().cameraDirectionCorrectGyro(Gyro.getInstance().getYaw());
        double angleRange = 30;

        if (isRocket) {
            if (angle > 0) {
                if (Math.abs(angle - 151.25) < angleRange) {
                    return TargetData.RIGHTROCKETLEFT;
                } else if (Math.abs(angle - 28.75) < angleRange) {
                    return TargetData.RIGHTROCKETRIGHT;
                } else if (Math.abs(angle - 90.0) < angleRange) {
                    return TargetData.RIGHTROCKETCENTER;

                } else {
                    System.out.println("No target estimate");
                    return TargetData.DEFAULT;
                }

            } else if (angle < 0) {

                if (Math.abs(angle + 28.75) < angleRange) {
                    return TargetData.LEFTROCKETLEFT;
                } else if (Math.abs(angle + 151.25) < angleRange) {
                    return TargetData.LEFTROCKETRIGHT;
                } else if (Math.abs(angle - 90.0) < angleRange) {
                    return TargetData.LEFTROCKETCENTER;

                } else {
                    System.out.println("No target estimate");
                    return TargetData.DEFAULT;
                }
            } else {
                System.out.println("No target estimate");
                return TargetData.DEFAULT;
            }

        } else {
            if (Math.abs(angle) < angleRange) {
                if (selectorLeft) {
                    return TargetData.CENTERCARGOLEFT;
                } else if (selectorRight) {
                    return TargetData.CENTERCARGORIGHT;
                } else {
                    System.out.println("No target estimate");
                    return TargetData.CENTERCARGOCENTER;
                }
            } else if (180 - Math.abs(angle) < angleRange && selectorLeft) {
                return TargetData.HUMANPLAYERLEFT;
            } else if (180 - Math.abs(angle) < angleRange && selectorRight) {
                return TargetData.HUMANPLAYERRIGHT;
            } else if (Math.abs(Math.abs(angle) - 90) < angleRange) {
                if (angle > 0) {
                    if (selectorLeft) {
                        return TargetData.LEFTCARGOLEFT;
                    } else if (selectorCenter) {
                        return TargetData.LEFTCARGOCENTER;
                    } else if (selectorRight) {
                        return TargetData.LEFTCARGORIGHT;
                    } else {
                        System.out.println("No target estimate");
                        return TargetData.CENTERCARGOCENTER;
                    }
                } else if (angle < 0) {
                    if (selectorLeft) {
                        return TargetData.RIGHTCARGOLEFT;
                    } else if (selectorCenter) {
                        return TargetData.RIGHTCARGOCENTER;
                    } else if (selectorRight) {
                        return TargetData.RIGHTCARGORIGHT;
                    } else {
                        System.out.println("No target estimate");
                        return TargetData.RIGHTCARGOCENTER;
                    }
                } else {
                    System.out.println("No target estimate");
                    return TargetData.DEFAULT;
                }
            } else {
                return TargetData.DEFAULT;
            }

        }

    }

    private static PositionEstimator instance = new PositionEstimator();

    public static PositionEstimator getInstance() {
        return instance;
    }

    /**
     * Called continuously, only updates values when the
     */
    public void driverSetTarget() {
        boolean selectorLeft = OI.getDriverRightXValue() < -0.5;
        boolean selectorRight = OI.getDriverRightXValue() > 0.5;
        boolean selectorCenter = OI.getDriverRightYValue() < -0.5;
        if (selectorLeft || selectorRight || selectorCenter) {
            this.isRocket = OI.getSetRocketButton(); // CHANGE THIS BUTTON TO RIGHT STICK PRESSED
            this.selectorLeft = selectorLeft;
            this.selectorRight = selectorRight;
            this.selectorCenter = selectorCenter;
            SmartDashboard.putBoolean("rocket", this.isRocket);
            SmartDashboard.putBoolean("left", this.selectorLeft);
            SmartDashboard.putBoolean("center", this.selectorCenter);
            SmartDashboard.putBoolean("right", this.selectorRight);

            RobotState.getInstance().setTargetData(guessPosition());

        }

    }

}