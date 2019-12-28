package org.usfirst.frc.team503.robot.vision.uselessstuff;

import java.util.ArrayList;

import org.usfirst.frc.team503.robot.vision.VisionLocalizer;

import edu.wpi.first.wpilibj.Notifier;

public class LockTarget {
    double lockMiddle;
    double middleDistance;

    double pointAValue;

    double[] lastResult = new double[2];

    public LockTarget() {

    }

    public void initialize() {

        System.out.println("Target Lock Initialized");
        lastResult = VisionLocalizer.getInstance().individualAnglesFromGrouped();
        System.out.println("Initial lastResult0: " + lastResult[0]);
        System.out.println("Initial lastResult1: " + lastResult[1]);

        // use the width of the box and the center coordinate to initialize point a and
        // b values
    }

    public void update() {

        double[] rawAngles = VisionLocalizer.getInstance().getIndividualTargetAngles();

        ArrayList<Double> rawAngleList = new ArrayList<Double>();
        for (int i = 0; i < rawAngles.length; i++) {
            if (rawAngles[i] != 0.0) {
                rawAngleList.add(rawAngles[i]);
            }
        }

        int closestIndex0 = 0, closestIndex1 = 0;
        double minDifference0 = 2, minDifference1 = 2;

        for (int i = 0; i < rawAngleList.size(); i++) {
            double difference0 = Math.abs(rawAngleList.get(i) - lastResult[0]);
            double difference1 = Math.abs(rawAngleList.get(i) - lastResult[1]);

            if (difference0 < minDifference0) {
                minDifference0 = difference0;
                closestIndex0 = i;

            }

            if (difference1 < minDifference1) {
                minDifference1 = difference1;
                closestIndex1 = i;
                // System.out.println("minimum updated to" + rawAngleList.get(i));
            }

        }

        if (minDifference0 > 0.05) {
            System.out.println("Something's screwed up, turned too far");
        }

        // System.out.println("last 1: " + lastResult[1]);

        lastResult[0] = rawAngleList.get(closestIndex0);

        // System.out.println("new 1: " + lastResult[1]);
        lastResult[1] = rawAngleList.get(closestIndex1);

        // System.out.println(differences1.get(0));
    }

    private class LockRunnable implements Runnable {
        @Override
        public void run() {
            update();
        }
    }

    private Notifier runner = new Notifier(new LockRunnable());

    // public void startTracking() {
    // initialize();
    // runner.startPeriodic(Robot.bot.LOCK_DT);
    // }

    public synchronized double[] getLastResult() {
        return lastResult;
    }

    public void stopTracking() {
        runner.stop();
        runner.close();
    }

    private static LockTarget instance = new LockTarget();

    public static LockTarget getInstance() {
        return instance;
    }
}