package org.usfirst.frc.team503.robot.vision.uselessstuff;

import java.util.ArrayList;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.utils.DataEntry;
import org.usfirst.frc.team503.robot.utils.UniversalGrapher;

import motionProfiling.Config;
import motionProfiling.PathGenerator;
import motionProfiling.Points;
import motionProfiling.Trajectory;
import motionProfiling.Trajectory.Segment;
import motionProfiling.Waypoint;

public class VisionPath {
	/**
	 * 
	 * @param initPts
	 * @return
	 */
	public double[][] returnPointsPath(double[][] initPts) {
		double[][] newPositions = { { initPts[0][0], initPts[0][1] } };
		double spacing = 2;
		ArrayList<double[]> newPositionsList = new ArrayList<double[]>();
		newPositionsList.add(newPositions[0]);
		for (int i = 0; i < initPts.length - 1; i++) {
			double vectX = initPts[i + 1][0] - initPts[i][0];
			double vectY = initPts[i + 1][1] - initPts[i][1];
			int numPoints = (int) (Math.ceil(Math.sqrt((vectX * vectX) + (vectY * vectY))) / spacing);
			double newVectX = (vectX / Math.sqrt((vectX * vectX) + (vectY * vectY))) * spacing;
			double newVectY = (vectY / Math.sqrt((vectX * vectX) + (vectY * vectY))) * spacing;
			for (int j = 0; j < numPoints - 1; j++) {
				newPositionsList
						.add(new double[] { initPts[i][0] + newVectX * (j + 1), initPts[i][1] + newVectY * (j + 1) });
			}
			newPositionsList.add(new double[] { initPts[i + 1][0], initPts[i + 1][1] });
		}
		newPositions = listToDoubleArray(newPositionsList);
		return newPositions;
	}

	public double[][] smoother(double[][] path) {
		VisionPath vp = new VisionPath();
		double[][] newPath = vp.doubleArrayCopy(path);
		double change = Robot.bot.smoothTolerance;
		while (change >= Robot.bot.smoothTolerance) {
			change = 0;
			for (int i = 1; i < path.length - 1; i++) {
				for (int j = 0; j < path[i].length; j++) {
					double aux = newPath[i][j];
					newPath[i][j] += Robot.bot.weightData * (path[i][j] - newPath[i][j])
							+ Robot.bot.weightSmooth * (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 * newPath[i][j]));
					change += Math.abs(aux - newPath[i][j]);
				}
			}
		}
		return newPath;
	}

	public double[][] smoothVisionTrajectory(double[][] origPts, int numOfTimesToSmooth) {
		double[][] addedPtsPoses = returnPointsPath(origPts);
		double[][] smoothPath = smoother(addedPtsPoses);
		for (int i = 0; i < numOfTimesToSmooth - 1; i++) {
			smoothPath = smoother(smoothPath);
		}
		return smoothPath;
	}

	public double findCurvature(int point, double[][] smoothPath) {
		double b;
		if ((point == 0) || (point == smoothPath.length - 1)) {
			return 0;
		}
		double x1 = smoothPath[point][0];
		double y1 = smoothPath[point][1];
		double x2 = smoothPath[point - 1][0];
		double y2 = smoothPath[point - 1][1];
		double x3 = smoothPath[point + 1][0];
		double y3 = smoothPath[point + 1][1];
		if (x1 == x2) {
			x1 += 1e-9;
		}
		double k1 = .5 * ((Math.pow(x1, 2)) + (Math.pow(y1, 2)) - (Math.pow(x2, 2)) - (Math.pow(y2, 2))) / (x1 - x2);
		double k2 = (y1 - y2) / (x1 - x2);
		if ((x3 * k2) - y3 + y2 - (x2 * k2) == 0) {
			b = .5 * ((Math.pow(x2, 2)) - (2 * x2 * k1) + (Math.pow(y2, 2)) - (Math.pow(x3, 2)) + (2 * x3 * k1)
					- (Math.pow(y3, 2))) / ((x3 * k2) - y3 + y2 - (x2 * k2) + 1e-9);
		} else {
			b = .5 * ((Math.pow(x2, 2)) - (2 * x2 * k1) + (Math.pow(y2, 2)) - (Math.pow(x3, 2)) + (2 * x3 * k1)
					- (Math.pow(y3, 2))) / ((x3 * k2) - y3 + y2 - (x2 * k2));
		}
		double a = k1 - (k2 * b);
		double r = Math.sqrt((Math.pow((x1 - a), 2)) + ((Math.pow((y1 - b), 2))));
		double curvature = 1 / r;
		if (Double.isNaN(curvature)) {
			return 0;
		}
		return curvature;
	}

	public double[] maxVelocities(double[][] smoothPath) {
		double[] maxVelocities = new double[smoothPath.length];
		for (int i = 0; i < smoothPath.length; i++) {
			maxVelocities[i] = Math.min((Robot.bot.pathGenKVel / (findCurvature(i, smoothPath) + .000001)) / 20,
					Robot.bot.pathGenMaxVel);
		}
		return maxVelocities;
	}

	public double[] targetVelocities(double[][] path) {
		double distanceBackwars = 0;
		double newVel[] = new double[path.length];
		newVel[0] = 0;
		double oldVel = 0;
		for (int i = path.length - 2; i > -1; i--) {
			distanceBackwars = Math.hypot(path[i][0] - path[i + 1][0], path[i][1] - path[i + 1][1]);
			newVel[i] = Math.min(Math.sqrt((Math.pow(oldVel, 2)) + (2 * Robot.bot.pathGenMaxVel * distanceBackwars)),
					maxVelocities(path)[i]);
			oldVel = Math.sqrt((Math.pow(oldVel, 2)) + (2 * Robot.bot.pathGenMaxAcc * distanceBackwars));
		}
		return newVel;
	}

	public double[] distancesAlongPath(double[][] smoothPath) {
		double[] distancesAlongPath = new double[smoothPath.length];
		for (int i = 0; i < smoothPath.length; i++) {
			distancesAlongPath[i] = getDistanceAtPt(i, smoothPath);
		}
		return distancesAlongPath;
	}

	public double getDistanceAtPt(int point, double[][] smoothPath) {
		double dist = 0;
		if (point == 0) {
			return 0;
		}
		for (int i = 1; i < point + 1; i++) {
			dist += Math.sqrt((Math.pow((smoothPath[i][0] - smoothPath[i - 1][0]), 2))
					+ (Math.pow((smoothPath[i][1] - smoothPath[i - 1][1]), 2)));
		}
		return dist;
	}

	public double[][] listToDoubleArray(ArrayList<double[]> l1) {
		double doubleArray[][] = new double[l1.size()][2];
		for (int i = 0; i < l1.size(); i++) {
			for (int j = 0; j < 2; j++) {
				doubleArray[i][j] = l1.get(i)[j];
			}
		}
		return doubleArray;
	}

	public double[][] doubleArrayCopy(double[][] orig) {
		double[][] copy = new double[orig.length][2];
		for (int i = 0; i < orig.length; i++) {
			for (int j = 0; j < 2; j++) {
				copy[i][j] = orig[i][j];
			}
		}
		return copy;
	}

	public void printDoubleArray(double[][] da) {
		for (int i = 0; i < da.length; i++) {
			for (int j = 0; j < da[i].length; j++) {
				System.out.print(da[i][j] + " ");
			}
			System.out.println();
		}
	}

	public void printSingleArray(double[] a) {
		for (int i = 0; i < a.length; i++) {
			System.out.print(a[i] + " ");
		}
		System.out.println();
	}

	public static void main(String[] args) {
		double kVel = 3;
		double maxAcc = 13;
		double maxVelocity = 30;
		double visionX = 7; // x-coord to target that vision gives
		double visionY = 60; // y-coord to target that vision gives
		double distanceBelowTarget = 20; // to steer in you would want a point below the setpoint so the curve is smooth
		double gyroAngle = -10; // 0 is straight and positive in clockwise direction
		double magnitude = 4; // Magnitude of the vector facing the direction of the gyro heading to decide
								// the 2nd point which makes our turns smoother
		double secondX, secondY; // Coordinates for the second point
		if (gyroAngle <= 0) {
			secondX = -Math.cos(Math.toRadians(90 + gyroAngle)) * magnitude;
			secondY = Math.sin(Math.toRadians(90 + gyroAngle)) * magnitude;
		} else {
			secondX = Math.cos(Math.toRadians(90 - gyroAngle)) * magnitude;
			secondY = Math.sin(Math.toRadians(90 - gyroAngle)) * magnitude;
		}
		double[][] positions = { { 0, 0 }, { secondX, secondY }, { visionX, visionY - distanceBelowTarget },
				{ visionX, visionY } };
		int numOfTimesToSmooth = 2;
		VisionPath vp = new VisionPath();
		double initTime = System.currentTimeMillis();
		double[][] smoothPath = vp.smoothVisionTrajectory(positions, numOfTimesToSmooth);
		double finalTime = System.currentTimeMillis();
		vp.printDoubleArray(smoothPath);

		double[] targetVelocities = vp.targetVelocities(smoothPath);
	}

	/**
	 * Parses and returns calculated
	 * 
	 * @param path
	 * @param velocities
	 * @return
	 */
	public Trajectory toTrajectory(double[][] path, double[] velocities) {
		Segment[] segments = new Segment[path.length];
		double angle;

		for (int i = 0; i < path.length; i++) {
			if (i == path.length - 1) {
				angle = Math.atan2((path[i][1] - path[i - 1][1]), (path[i][0] - path[i - 1][0]));
			} else if (i == 0) {
				angle = Math.atan2((path[i + 1][1] - path[i][1]), (path[i + 1][0] - path[i][0]));
			} else {
				angle = Math.atan2((path[i + 1][1] - path[i - 1][1]), (path[i + 1][0] - path[i - 1][0]));
			}

			segments[i] = new Segment(0, velocities[i], 0, 0, angle, 0.05, path[i][0], path[i][1]);
		}

		return (new Trajectory(segments));

	}

	public Trajectory toLibTrajectory(double x, double y, Config config) {
		Points points = new Points();
		points.addWaypoint(new Waypoint(0, 0, 90, true));
		points.addWaypoint(new Waypoint(0, 60, 90, true));
		return PathGenerator.generateCentralTrajectory(points, (new Config(0.05, 130, 100, 8000)));
	}

	public void sendNetworkTablesData(double[][] path, double[] velocities) {
		double[] x = new double[path.length], y = new double[path.length];
		for (int i = 0; i < path.length; i++) {
			x[i] = path[i][0];
			y[i] = path[i][1];
		}

		UniversalGrapher.addEntries((new DataEntry("x", x)));
		UniversalGrapher.addEntries((new DataEntry("y", y)));
		UniversalGrapher.addEntries((new DataEntry("v", velocities)));
		UniversalGrapher.sendValues();

	}
}