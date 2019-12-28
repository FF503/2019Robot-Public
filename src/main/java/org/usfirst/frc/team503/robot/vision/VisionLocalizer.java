package org.usfirst.frc.team503.robot.vision;

import org.usfirst.frc.team503.robot.OI;
import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.RobotState.CameraDirection;
import org.usfirst.frc.team503.robot.utils.FFDashboard;
import org.usfirst.frc.team503.robot.utils.GPS;
import org.usfirst.frc.team503.robot.utils.Pose;
import org.usfirst.frc.team503.robot.vision.uselessstuff.LockTarget;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionLocalizer {

    double dClosestTargetSlope = 0.8848;
    double intercept = -1.546;
    double regressedDClosestTarget, boonkgangedX, OGX, OGY;
   
    public VisionLocalizer() {

    }

    public boolean lostTarget() {
        double tx = VisionLocalizer.getInstance().getTX();

        double nx = VisionLocalizer.getInstance().deg2Normalized(tx);

        double pixelsX = 160 * nx + 159.5;
        double length = VisionLocalizer.getInstance().getTHor() / 2;
        double leftMost = (pixelsX - length);
        double rightMost = pixelsX + length;
        SmartDashboard.putNumber("leftMost", leftMost);
        SmartDashboard.putNumber("rightMost", rightMost);
       // System.out.println("Leftmost: " + leftMost + ", Rightmost: " + rightMost);

        if (rightMost > 300 || leftMost < 20) {
            return true;
        } else {
            return false;
        }

    }

    /**
     * Converts the normalized screenspace coordinates from the limelight into
     * radians from the center of the camera
     * 
     * @param nx the normalized screenspace coordinate, ,-1<<1
     * @return the angle to the same point in radians
     */
    public double normalized2R(double nx) {
        double x = Robot.bot.viewportWidth / 2 * nx;
        return Math.PI / 2 - Math.atan2(1, x);
        // return Math.atan2(1, x);
    }

    /**
     * Converts degrees into the normalized screenspace coordinates the limelight
     * uses
     * 
     * @param nx theangle to be converted, in degrees
     * @return the angle to the same point in degrees
     */
    public double deg2Normalized(double deg) {
        double x = Math.tan(Math.toRadians(deg));
        return 2 * x / Robot.bot.viewportWidth;
    }

    /**
     * Returns screenspace camera coordinates to the 2 highest scoring individual
     * targets
     * 
     * @return angles in NORMALIZED COORDINATES, NEED TO CONVERT TO ANGLES
     */
    public synchronized double[] getIndividualTargetAngles() {
        setPipeline(RobotState.getInstance().getTargetData().getIndividualPipeline());
        double[] output = { getRawTX(0), getRawTX(1), getRawTX(2) };
        return output;
    }

    /**
     * Gets all visible dual-target raw contours (Doesn't work correctly)
     * 
     * @return Double array with all angles in normalized camera coordinates
     */
    public double[] getAllGroupedTargetAngles() {
        setGrouped();
        double[] output = { getRawTX(0), getRawTX(1), getRawTX(2) };
        return output;
    }

    /**
     * Provides angle to the center of the target in individual mode
     * 
     * @return Angle, in degrees, to the middle of the target
     */
    public double angleToCenter() {
        double[] data = getIndividualTargetAngles();
        double nxavg = (data[0] + data[1]) / 2;

        SmartDashboard.putNumber("Angle to center", Math.toDegrees(normalized2R(nxavg)));
        return normalized2R(nxavg);
        // return (normalized2R(data[0]) - normalized2R(1))/2;
    }

    /**
     * Provides the angle from the center of the robot to the target, relative to
     * the target
     * 
     * @param estimate Position of the robot, in a double[], use
     *                 VisionLocalizer.getInstance().getPosition()
     * @return Angle, in degrees
     */
    public double translatedAngleToCenter(double[] estimate) {
        // System.out.println("Position: " + estimate[0] + " " + estimate[1]);
        // TODO: This might not work, might want to remove gyro angle/change sign
        double angle = Math.toDegrees(Math.atan(estimate[0] / estimate[1])); // +
                                                                             // RobotState.getInstance().getTargetData().getAdjustedAngle();
        while (angle > 60) {
            angle -= 90;
        }

        while (angle < -60) {
            angle += 90;
        }

        return angle;
    }

    public double boonkgangedAngleToCenter() {
        double outwardsDistance = 27; // 24

        // double gyroAngle =
        // RobotState.getInstance().getTargetData().getAdjustedAngle();
        double[] estimate = translatedPositionEstimate(getIndividualTargetAngles());
        System.out.println("Position: " + estimate[0] + " " + estimate[1]);
        // TODO: This might not work, might want to remove gyro angle/change sign
        double angle = Math.toDegrees(Math.atan2(estimate[0], estimate[1] - outwardsDistance)); // +
                                                                                                // RobotState.getInstance().getTargetData().getAdjustedAngle();
        while (angle > 60) {
            angle -= 90;
        }

        while (angle < -60) {
            angle += 90;
        }

        return angle;
    }

    /**
     * Calculates the angle the robot has to turn so that it will stop a set
     * horizontal distance away from the target when x coordinate reaches 0
     * 
     * @return Target angle, in degrees, to the setpoint
     */
    public double translatedPrepareAngleTarget() {
        double[] data = translatedPositionEstimate(getIndividualTargetAngles());
        double x = data[0];
        if (x == 0) {
            System.out.println("It's 0, shoot");
        }

        double y = (data[1] - Robot.bot.getPAIntersection());
        System.out.println("Attempting to stop at " + Robot.bot.getPAIntersection() + " in front of the target");
        System.out.println("Attempting to point to (" + x + ", " + y + ")");
        return Math.toDegrees(Math.atan(x / y));
    }

    public boolean areaOnTarget() {
        return (getIndividualTargetArea() > Robot.bot.getAreaThreshold());
    }

    public double pixelsToNormalizedX(double pixels) {
        return (1.0 / 160.0) * (pixels);
    }

    public double pixelsToNormlizedY(double pixels) {
        return (1.0 / 120.0) * (pixels);
    }

    /**
     * Uses the width of the grouped target bounding box in pixels and the angle of
     * the angle from the camera to the center of the grouped targets to estimate
     * the locations of the individual targets
     * 
     * @return Double array containing the two angles, in normalized coordinates
     */
    public double[] individualAnglesFromGrouped() {
        setPipeline(RobotState.getInstance().getTargetData().getGroupedPipeline());
        double tx = VisionLocalizer.getInstance().deg2Normalized(getTX());
        SmartDashboard.putNumber("tx normalized", tx);
        double difference = pixelsToNormalizedX(getBoxWidthPixels()) / 2;
        System.out.println("Normalized coordinate difference: " + difference);

        double[] output = { tx - difference / 2, tx + difference / 2 };
        return output;
    }

    public double cameraDirectionCorrectGyro(double gyroAngle) {

        if (RobotState.getInstance().getCameraDirection().equals(CameraDirection.BACK)) {
            if (gyroAngle >= 0) {
                gyroAngle -= 180;
            } else {
                gyroAngle += 180;
            }
            gyroAngle = gyroAngle + 0;
            // System.out.println("Correcting Gyro Angle backwards to:" + gyroAngle);
        }

        return gyroAngle;
    }

    public double getError(double target, double current) {
        return target - current;
    }

    /************************************************************/
    // Localization Calculations
    /************************************************************/

    public double[] lockedPositionEstimate() {
        double[] lockedLockedTargetCoords = LockTarget.getInstance().getLastResult();
        if (lockedLockedTargetCoords[0] == lockedLockedTargetCoords[1]) {
            return translatedPositionEstimate(getIndividualTargetAngles());
        } else {
            return translatedPositionEstimate(LockTarget.getInstance().getLastResult());
        }
    }

    /**
     * Uses camera and gyro data to estimate the robot's position relative to the
     * target it's able to see
     * 
     * @return double array with x and y coordinates
     */
    public double[] positionEstimate(double[] targetAngles) {

        double angle1 = normalized2R(targetAngles[0]);
        double angle2 = normalized2R(targetAngles[1]);

        double a = Math.min(angle1, angle2);
        double b = Math.max(angle1, angle2);

        RobotState.getInstance().setTargetAngles(new double[] { Math.toDegrees(a), Math.toDegrees(b) });
        // System.out.println("a: " + a);
        // System.out.println("b: " + b);
        if (a < Math.toRadians(-20) || b > Math.toRadians(20)) {

         //   RobotState.getInstance().setLostTarget(true);
            System.out.println("Lost target with a of: " + a + " and b of: " + b);
        }

        if (b != 69) {

            SmartDashboard.putNumber("a (degrees)", Math.toDegrees(a));
            SmartDashboard.putNumber("b (degrees)", Math.toDegrees(b));

            double gyroAngleR = Math.toRadians(RobotState.getInstance().getTargetData().getAdjustedAngle());
            double phi = Math.abs(b - a);
            double beta = 0, dClosestTarget = 0, x = 0, y = 0;
            // phi: angle opposite of distanceBetweenTarget
            // beta: angle opposite of distanceToLeftTarget

            if (gyroAngleR == 0) {
                gyroAngleR += 1e-9;
            }

            // No regression on dClosestTarget, regression applied to y
            // x: No regression, using inaccurate angles thus inaccurate; "vision x"
            // y: Regression applied to y; "vision y"

            // regression on dClosestTarget
            // x: "boonkganged x"
            // y: should be the same os "vision y"

            // Calculating x without any regression at all

            if (gyroAngleR < 0) { // Robot angled to left
                if (a > 0 && b > 0) { // pointing to left of vision targets
                    beta = Math.PI / 2 - gyroAngleR - b;
                    dClosestTarget = Robot.bot.distanceBetweenTargets * Math.sin(beta) / Math.sin(phi);
                    y = Math.sin(beta + phi) * dClosestTarget;

                    double dClosestTargetX = Math.cos(phi + beta) * dClosestTarget;
                    x = Robot.bot.distanceBetweenTargets / 2 + dClosestTargetX;
                    SmartDashboard.putNumber("OG x", x);

                    y = Robot.bot.ySlope * y + Robot.bot.yConstant;
                    x = Math.tan(gyroAngleR + a) * y + Robot.bot.targetGap / 2;

                    regressedDClosestTarget = dClosestTargetSlope * dClosestTarget + intercept;
                    SmartDashboard.putNumber("regressed dClosestTarget", regressedDClosestTarget);
                    SmartDashboard.putNumber("boonkganged x",
                            Math.cos(beta + phi) * regressedDClosestTarget + Robot.bot.targetGap / 2);

                } else if (a < 0 && b > 0) { // pointing to middle of vision targets
                    // Works
                    beta = Math.PI / 2 - gyroAngleR - b;
                    dClosestTarget = Robot.bot.distanceBetweenTargets * Math.sin(beta) / Math.sin(phi);
                    y = Math.cos(-gyroAngleR - a) * dClosestTarget;

                    double dClosestTargetX = Math.sin(-gyroAngleR - a) * dClosestTarget;
                    x = Robot.bot.distanceBetweenTargets / 2 + dClosestTargetX;
                    SmartDashboard.putNumber("OG x", x);

                    y = Robot.bot.ySlope * y + Robot.bot.yConstant;
                    x = Math.tan(gyroAngleR + a) * y + Robot.bot.targetGap / 2;

                    regressedDClosestTarget = dClosestTargetSlope * dClosestTarget + intercept;
                    SmartDashboard.putNumber("regressed dClosestTarget", regressedDClosestTarget);
                    SmartDashboard.putNumber("boonkganged x",
                            Math.sin(-gyroAngleR - a) * regressedDClosestTarget + Robot.bot.targetGap / 2);

                } else if (a < 0 && b < 0) {
                    // Works
                    beta = Math.PI / 2 + gyroAngleR + a;
                    dClosestTarget = Robot.bot.distanceBetweenTargets * Math.sin(beta) / Math.sin(phi);
                    y = Math.cos(gyroAngleR - a) * dClosestTarget;

                    double dClosestTargetX = Math.cos(-gyroAngleR - a) * dClosestTarget;
                    x = Robot.bot.distanceBetweenTargets / 2 + dClosestTargetX;
                    SmartDashboard.putNumber("OG x", x);

                    y = Robot.bot.ySlope * y + Robot.bot.yConstant;
                    x = Math.tan(gyroAngleR + a) * y + Robot.bot.targetGap / 2;

                    regressedDClosestTarget = dClosestTargetSlope * dClosestTarget + intercept;
                    SmartDashboard.putNumber("regressed dClosestTarget", regressedDClosestTarget);
                    SmartDashboard.putNumber("boonkganged x",
                            Math.sin(gyroAngleR - a) * dClosestTarget + Robot.bot.targetGap / 2);
                } else {
                    // System.out.println("Camera is blocked");
                }

            } else if (gyroAngleR > 0) { // angled right
                if (a > 0 && b > 0) { // pointing to left of vision targets
                    // Works
                    beta = Math.PI / 2 - gyroAngleR - b;
                    dClosestTarget = Robot.bot.distanceBetweenTargets * Math.sin(beta) / Math.sin(phi);
                    y = Math.sin(beta + phi) * dClosestTarget;

                    double dClosestTargetX = Math.cos(phi + beta) * dClosestTarget;
                    x = Robot.bot.distanceBetweenTargets / 2 + dClosestTargetX;
                    SmartDashboard.putNumber("OG x", x);

                    y = Robot.bot.ySlope * y + Robot.bot.yConstant;
                    x = Math.tan(gyroAngleR + a) * y + Robot.bot.targetGap / 2;

                    regressedDClosestTarget = dClosestTargetSlope * dClosestTarget + intercept;
                    SmartDashboard.putNumber("regressed dClosestTarget", regressedDClosestTarget);
                    SmartDashboard.putNumber("boonkganged x",
                            Math.cos(beta + phi) * regressedDClosestTarget + Robot.bot.targetGap / 2);

                } else if (a < 0 && b > 0) { // pointing to middle of vision targets
                    beta = Math.PI / 2 - gyroAngleR - b;
                    dClosestTarget = Robot.bot.distanceBetweenTargets * Math.sin(beta) / Math.sin(phi);
                    y = Math.cos(-a - gyroAngleR) * dClosestTarget;

                    double dClosestTargetX = Math.sin(-a - gyroAngleR) * dClosestTarget;
                    x = Robot.bot.distanceBetweenTargets / 2 + dClosestTargetX;
                    SmartDashboard.putNumber("OG x", x);

                    y = Robot.bot.ySlope * y + Robot.bot.yConstant;
                    x = Math.tan(gyroAngleR + a) * y + Robot.bot.targetGap / 2;

                    regressedDClosestTarget = dClosestTargetSlope * dClosestTarget + intercept;
                    SmartDashboard.putNumber("regressed dClosestTarget", regressedDClosestTarget);
                    SmartDashboard.putNumber("boonkganged x",
                            Math.sin(-a - gyroAngleR) * regressedDClosestTarget + Robot.bot.targetGap / 2);

                } else if (a < 0 && b < 0) {
                    // beta = Math.PI / 2 + gyroAngleR + b;
                    // dClosestTarget = Robot.bot.distanceBetweenTargets * Math.sin(beta) /
                    // Math.sin(phi);
                    // y = Math.cos(gyroAngleR + a) * dClosestTarget;
                    // y = Robot.bot.ySlope * y + Robot.bot.yConstant;
                    // x = Math.tan(gyroAngleR + a) * y + Robot.bot.targetGap / 2;
                    beta = Math.PI / 2 + gyroAngleR + a;
                    dClosestTarget = Robot.bot.distanceBetweenTargets * Math.sin(beta) / Math.sin(phi);
                    y = Math.cos(gyroAngleR - a) * dClosestTarget;

                    double dClosestTargetX = Math.cos(-gyroAngleR - a) * dClosestTarget;
                    x = Robot.bot.distanceBetweenTargets / 2 + dClosestTargetX;
                    SmartDashboard.putNumber("OG x", x);

                    y = Robot.bot.ySlope * y + Robot.bot.yConstant;
                    x = Math.tan(gyroAngleR + a) * y + Robot.bot.targetGap / 2;

                    x = Math.tan(angleToCenter() + gyroAngleR) * y;

                    regressedDClosestTarget = dClosestTargetSlope * dClosestTarget + intercept;
                    SmartDashboard.putNumber("regressed dClosestTarget", regressedDClosestTarget);
                    SmartDashboard.putNumber("boonkganged x",
                            Math.sin(gyroAngleR - a) * dClosestTarget + Robot.bot.targetGap / 2);

                } else {
                    // System.out.println("Camera is blocked");
                }
            } else {
                System.out.println("Exception to gyro angle");
            }

            if (RobotState.getInstance().isVisionDebug()) {
                SmartDashboard.putNumber("individual angle 1", targetAngles[0]);
                SmartDashboard.putNumber("individual angle 2", targetAngles[1]);
                SmartDashboard.putNumber("dCLosestTarget", dClosestTarget);
                SmartDashboard.putNumber("beta (degrees)", Math.toDegrees(beta));
                SmartDashboard.putNumber("phi (degrees)", Math.toDegrees(phi));
            }

            // y = (x) / Math.tan(Math.abs(angleToCenter() + gyroAngleR));
            // y = Robot.bot.ySlope * y + Robot.bot.yConstant;
            double[] result = { x, y };
            return result;
        } else {
            double[] result = { 0, 0 };
            return result;
        }
    }

    /************************************************************/
    // Translations
    /************************************************************/
    /**
     * Use this to get the robot's position in general cases
     * 
     * @return double[] of position in inches, (x, y)
     */
    public double[] getPosition() {
        return translatedPositionEstimate(getIndividualTargetAngles());
    }

    public boolean isTargetSeen() {
        return getTV() == 1.0;
    }

    public double[] translatedPositionEstimate(double[] targetAngles) {

        double[] estimate = VisionLocalizer.getInstance().positionEstimate(targetAngles);

        double transX = VisionLocalizer.getInstance().translateCameraX(estimate[0]);
        double transY = VisionLocalizer.getInstance().translateCameraY(estimate[1]);

        double[] array = { transX, transY };

        return array;
    }

    /**
     * Calculates the target-relative x coordinate of the center of the robot
     * 
     * @param unfiltered Raw x coordinate to be translated
     * @return Translated x coordinate
     */
    public double translateCameraX(double unfiltered) {
        double angle = -Math.toRadians(RobotState.getInstance().getTargetData().getAdjustedAngle());
        double[] offsets = Robot.bot.getLimelightOffsets();
        double xOffset = Math.cos(angle) * offsets[0] - Math.sin(angle) * offsets[1];

        SmartDashboard.putNumber("xOffset", xOffset);
        return unfiltered - xOffset;
    }

    /**
     * Calculates the target-relative y coordinate of the center of the robot
     * 
     * @param unfiltered Raw y coordinate to be translated
     * @return Translated y coordinate
     */
    public double translateCameraY(double unfiltered) {
        double angle = -Math.toRadians(RobotState.getInstance().getTargetData().getAdjustedAngle());
        double[] offsets = Robot.bot.getLimelightOffsets();
        double yOffset = Math.sin(angle) * offsets[0] + Math.cos(angle) * offsets[1];
        SmartDashboard.putNumber("yOffset", yOffset);
        return Math.abs(unfiltered - yOffset);
    }

    public Pose getFieldCoordinates(double[] positionEstimate) {
        double[] targetData = positionEstimate;
        double[] result = new double[2];
        double angleToRotateCCW = -RobotState.getInstance().getTargetData().getTargetFieldAngle();
        SmartDashboard.putNumber("Angle to rotate CCW", angleToRotateCCW);
        result[0] = targetData[0] * Math.cos(Math.toRadians(angleToRotateCCW))
                + targetData[1] * Math.sin(Math.toRadians(angleToRotateCCW));
        result[1] = -targetData[0] * Math.sin(Math.toRadians(angleToRotateCCW))
                + targetData[1] * Math.cos(Math.toRadians(angleToRotateCCW));

        SmartDashboard.putNumber("x (field coords) from target", result[0]);
        SmartDashboard.putNumber("y (field coords) from target", result[1]);
        result[0] += RobotState.getInstance().getTargetData().getTargetFieldX();
        result[1] += RobotState.getInstance().getTargetData().getTargetFieldY();

        return new Pose(result[0], result[1], 0);
    }

    /**
     * @param original
     * @param angle    Angle to rotate counterclockwise
     * @return Same vector, with rotated axis
     */
    private double[] rotatateAxis(double[] original, double angle) {
        double[] result = new double[2];
        result[0] = original[0] * Math.cos(Math.toRadians(angle)) + original[1] * Math.sin(Math.toRadians(angle));
        result[1] = -original[0] * Math.sin(Math.toRadians(angle)) + original[1] * Math.cos(Math.toRadians(angle));
        return result;
    }

 
    // main arm limelight
    public NetworkTable getTable(){
        if (RobotState.getInstance().getLimelightType() == RobotState.LimeLightType.ARM){
            return NetworkTableInstance.getDefault().getTable(Robot.bot.ARM_LIMELIGHT);
        }
        else if (RobotState.getInstance().getLimelightType()  == RobotState.LimeLightType.TURRET){
            return NetworkTableInstance.getDefault().getTable(Robot.bot.TURRET_LIMELIGHT);
        }
        System.err.println("ERROR LIMELIGHT TYPE MESSED UP");
        return null;
        
    }

    /************************************************************/
    // Limelight NetworkTables interactions
    /************************************************************/

    /**
     * Sets the index of the limelight pipeline w/ networktables, use this to (not
     * robotstate) to actually change values
     * 
     * @param pipeline index of the desired pipeline
     */
    public void setPipeline(double pipeline) {
        setProcessing();
        getTable().getEntry("pipeline").setNumber(pipeline);
        if (pipeline != RobotState.getInstance().getCurrentPipeline()) {
            System.out.println("Pipeline set to: " + pipeline);
        }
        RobotState.getInstance().setCurrentPipeline(pipeline);

    }

    private double getTV() {
        return getTable().getEntry("pipeline").getNumber(0.0)
                .doubleValue();
    }

    private void setCamMode(double camMode) {
        getTable().getEntry("camMode").setNumber(camMode);
        if (camMode != RobotState.getInstance().getCameraMode()) {
            System.out.println("Camera Mode set to: " + camMode);
        }
    }

    public double getTX() {
        return getTable().getEntry("tx").getDouble(0.0);
    }

    public double getTA() {
        return getTable().getEntry("ta").getDouble(0.0);
    }

    public double getRawTX(int index) {
        return getTable().getEntry("tx" + index).getDouble(0.0);
    }

    public double getTHor() {
        return getTable().getEntry("thor").getDouble(0.0);
    }

    public double getBoxWidthPixels() {
        setPipeline(RobotState.getInstance().getTargetData().getGroupedPipeline());
        return getTHor();
    }

    public double getGroupedTargetAngle() {
        setPipeline(RobotState.getInstance().getTargetData().getGroupedPipeline());
        return getTX();
    }

    public double getGroupedTargetArea() {
        setPipeline(RobotState.getInstance().getTargetData().getGroupedPipeline());
        return getTA();
    }

    public double getIndividualTargetArea() {
        // setPipeline(7);
        // setPipeline(4);

        // setPipeline(RobotState.getInstance().getTargetData().getIndividualPipeline());
        return getTA();
    }

    public double getIndividualTargetAreaStraight() {
        setPipeline(7);
        // setPipeline(4);
        // setPipeline(RobotState.getInstance().getTargetData().getIndividualPipeline());
        return getTA();
    }

    public void setGrouped() {
        setPipeline(RobotState.getInstance().getTargetData().getGroupedPipeline());
    }

    public void setIndividual() {
        // setPipeline(5);
        // setPipeline(RobotState.getInstance().getTargetData().getIndividualPipeline());
    }

    public void setLeftPipeline() {
        setPipeline(5);
        // setPipeline(RobotState.getInstance().getTargetData().getIndividualPipeline());
    }

    public void setRightPipeline() {
        setPipeline(4);
    }

    public void setDrive() {
        setPipeline(9.0);
    }

    public void setProcessing() {
        setCamMode(0);
    }

    public double getBallAngle() {
        setPipeline(6);
        return getTX();
    }

    public double getBallArea() {
        setPipeline(6);
        return getTA();
    }

    /************************************************************/
    // Utilities
    /************************************************************/

    private static VisionLocalizer instance = new VisionLocalizer();

    public static VisionLocalizer getInstance() {
        return instance;
    }

    private Notifier runner = new Notifier(new VisionRunnable());

    private boolean runAlg = false;

    private double[] gpsDiff = { 0, 0 }, visionDiff = { 0, 0 };

    private double[] lastAccurateVisionValue = { 0, 0 }, lastAccurateGPSValue = { 0, 0 };

    private double initX, initY;

    private boolean gpsInitialized = false;
    private boolean coordsInitialized = false;

    private class VisionRunnable implements Runnable {
        @Override
        public void run() {

            FFDashboard.getInstance().putString("selectedTarget", RobotState.getInstance().getTargetData().toString());
            if (OI.driverDPadUp.get()) {
                runAlg = true;
                FFDashboard.getInstance().putBoolean("isAligning", true);
                FFDashboard.getInstance().putBoolean("cantSee", false);
                System.out.println("dpad pressed up");
            } else if (OI.driverDPadDown.get()) {
                runAlg = false;
                System.out.println("dpad pressed down");
            }

            if (runAlg) {
                if (!gpsInitialized) {

                    // Determine initial
                    initX = GPS.getInstance().getX();
                    initY = GPS.getInstance().getY();

                    // First thing to test
                    System.out.println("Pose reset to x of: " + initX);
                    System.out.println("Pose reset to y of: " + initY);

                    gpsInitialized = true;
                }

                double[] estimate = positionEstimate(getIndividualTargetAngles());
                double[] translatedEstimate = { translateCameraX(estimate[0]), translateCameraY(estimate[1]) };
                // System.out.println(
                // "Robot's vision position: (" + translatedEstimate[0] + ", " +
                // translatedEstimate[1] + ")");
                double[] rawGPSValues = {
                        (GPS.getInstance().getX() - initX)
                                * RobotState.getInstance().getCameraDirection().getMultiplier(),
                        ((GPS.getInstance()).getY() - initY)
                                * RobotState.getInstance().getCameraDirection().getMultiplier() };
                // System.out.println("Robot's raw GPS position: (" + rawGPSValues[0] + ", " +
                // rawGPSValues[1] + ")");
                SmartDashboard.putNumber("Raw GPS x", rawGPSValues[0]);
                SmartDashboard.putNumber("Raw GPS y", rawGPSValues[1]);
                double magnitude = Math.hypot(rawGPSValues[0], rawGPSValues[1]);
                double angle = RobotState.getInstance().getTargetData().getTargetFieldAngle();

                double[] gpsValues = rotatateAxis(rawGPSValues, -angle);
                // { magnitude * Math.cos(Math.toRadians(angle)),
                // magnitude * Math.sin(Math.toRadians(angle)) };

                // System.out.println("Robot's translated GPS position: (" + gpsValues[0] + ", "
                // + gpsValues[1] + ")");

                SmartDashboard.putNumber("Translated GPS X", gpsValues[0]);
                SmartDashboard.putNumber("Translated GPS Y", gpsValues[1]);
                // FFDashboard.getInstance().getSubTable("Vision").getEntry("Corrected GPS
                // Values")
                // .setDoubleArray(gpsValues);
                // FFDashboard.getInstance().getSubTable("Vision").getEntry("Raw GPS
                // Values").setDoubleArray(rawGPSValues);

                // initialize recent accurate values
                if (!coordsInitialized) {
                    for (int i = 0; i < 2; i++) {
                        lastAccurateVisionValue[i] = translatedEstimate[i];
                        lastAccurateGPSValue[i] = gpsValues[i];
                    }
                    coordsInitialized = true;
                    SmartDashboard.putNumber("Last accurate vision value x", lastAccurateVisionValue[0]);
                    SmartDashboard.putNumber("Last accurate vision value y", lastAccurateVisionValue[1]);

                    SmartDashboard.putNumber("Last accurate gps value x", lastAccurateGPSValue[0]);
                    SmartDashboard.putNumber("Last accurate gps value y", lastAccurateGPSValue[1]);

                    System.out.println("Coords initialized");
                }

                // calculate differences between current and last accurate values
                for (int i = 0; i < 2; i++) {
                    visionDiff[i] = translatedEstimate[i] - lastAccurateVisionValue[i];
                    gpsDiff[i] = gpsValues[i] - lastAccurateGPSValue[i];

                }

                double correctedX, correctedY;

                // If the difference between how much GPS and vision have changed since they
                // were both accurate
                SmartDashboard.putNumber("x difference", visionDiff[0] - gpsDiff[0]);
                SmartDashboard.putNumber("y difference", visionDiff[1] - gpsDiff[1]);

                if ((Math.abs(visionDiff[0] - gpsDiff[0]) < 5) && (Math.abs(visionDiff[1] - gpsDiff[1]) < 5)) {

                    // Output vision coordinates
                    System.out.println("Vision data is good");
                    correctedX = translatedEstimate[0];
                    correctedY = translatedEstimate[1];
                    double[] goodGPSCoords = {
                            (GPS.getInstance().getX() - initX)
                                    * RobotState.getInstance().getCameraDirection().getMultiplier(),
                            (GPS.getInstance().getY() - initY)
                                    * RobotState.getInstance().getCameraDirection().getMultiplier() };
                    for (int i = 0; i < 2; i++) {
                        lastAccurateGPSValue[i] = goodGPSCoords[i];
                        lastAccurateVisionValue[i] = translatedEstimate[i];
                    }
                    SmartDashboard.putNumber("Last accurate vision value x", lastAccurateVisionValue[0]);
                    SmartDashboard.putNumber("Last accurate vision value y", lastAccurateVisionValue[1]);

                    SmartDashboard.putNumber("Last accurate gps value x", lastAccurateGPSValue[0]);
                    SmartDashboard.putNumber("Last accurate gps value y", lastAccurateGPSValue[1]);

                } else {
                    SmartDashboard.putNumber("Vision x difference", visionDiff[0]);
                    SmartDashboard.putNumber("Vision y difference", visionDiff[1]);
                    SmartDashboard.putNumber("GPS x difference", gpsDiff[0]);
                    SmartDashboard.putNumber("GPS y difference", gpsDiff[1]);

                    // System.out.println("Using odometry redundancy");

                    correctedX = lastAccurateVisionValue[0] - gpsDiff[0];
                    correctedY = lastAccurateVisionValue[1] - gpsDiff[1];

                }

                double robotAngle = RobotState.getInstance().getTargetData().getAdjustedAngle();

                SmartDashboard.putNumber("Vision x", estimate[0]);
                SmartDashboard.putNumber("Vision y", estimate[1]);

                // TODO: Might have to fix this later
                SmartDashboard.putNumber("Translated Vision x", translatedEstimate[0]);
                SmartDashboard.putNumber("Translated Vision y", translatedEstimate[1]);
                SmartDashboard.putNumber("Gyro angle relative to target (degrees)", angle);

                SmartDashboard.putNumber("Corrected Vision x", correctedX);
                SmartDashboard.putNumber("Corrected Vision y", correctedY);

                SmartDashboard.putNumber("Gyro angle relative to target (degrees)", robotAngle);

                FFDashboard.getInstance().putNumber("Translated x", correctedX);
                FFDashboard.getInstance().putNumber("Translated y", correctedY);

                // FFDashboard.getInstance().putNumber("Translated x", translatedEstimate[0]);
                // FFDashboard.getInstance().putNumber("Translated y", translatedEstimate[1]);
                FFDashboard.getInstance().putNumber("Adjusted Angle", robotAngle);

            } else {
                FFDashboard.getInstance().putBoolean("isAligning", false);
                setDrive();
            }

        }
    }

    public void sendDashboardData() {

        double[] estimate = positionEstimate(getIndividualTargetAngles());
        double[] translatedEstimate = { translateCameraX(estimate[0]), translateCameraX(estimate[1]) };
        SmartDashboard.putNumber("Angle to target center (degrees)", translatedAngleToCenter(translatedEstimate));

        SmartDashboard.putNumber("Vision x", estimate[0]);
        SmartDashboard.putNumber("Vision y", estimate[1]);
        SmartDashboard.putNumber("Translated Vision x", translatedEstimate[0]);
        SmartDashboard.putNumber("Translated Vision y", translatedEstimate[1]);
        SmartDashboard.putNumber("Gyro angle relative to target (degrees)",
                RobotState.getInstance().getTargetData().getAdjustedAngle());
        SmartDashboard.putString("Currently set target", RobotState.getInstance().getTargetData().toString());
        SmartDashboard.putNumber("x Relative to origin", getFieldCoordinates(translatedEstimate).getX());
        SmartDashboard.putNumber("y Relative to origin", getFieldCoordinates(translatedEstimate).getY());
    }

    public void startTracking() {
        GPS.getInstance().startTracking();
        runner.startPeriodic(Robot.bot.VISION_DT);

        System.out.println("Vision tracking started");
    }

    public void stopTracking() {

        runner.close();
        VisionLocalizer.getInstance().setDrive();
    }

}
