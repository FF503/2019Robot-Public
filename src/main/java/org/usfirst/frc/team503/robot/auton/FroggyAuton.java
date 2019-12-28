/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.auton;

import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.RobotState.StartingDirection;
import org.usfirst.frc.team503.robot.commands.PurePursuitDrive;
import org.usfirst.frc.team503.robot.utils.FFDashboard;
import org.usfirst.frc.team503.robot.utils.Pose;
import org.usfirst.frc.team503.robot.utils.ProfileLoader;

import edu.wpi.first.wpilibj.command.CommandGroup;

public abstract class FroggyAuton extends CommandGroup {
    protected enum AutonStartingLocation {

        Right(212, 63, 90), RightLevel2(212, 32, 90), Origin(0, 0, 90), tenFeetForward(0, 120, 90),
        FirstCargoBay(208, 255, 180), Left(112, 63, 90), LeftLevel2(112, 32, 90), TEST(301, 15, -90),
        FirstHatch(223.0, 263.0, 0.0), FirstHatchTurned(234.0, 266.0, 90.0);

        private Pose pose;

        AutonStartingLocation(Pose pose) {
            this.pose = pose;
        }

        AutonStartingLocation(double x, double y, double theta) {
            this.pose = new Pose(x, y, theta);
        }

        public Pose getStartingPose() {
            return pose;
        }

    }

    public static enum FieldLocations {
        LeftFirstBay(116, 263, 0), RightFirstBay(208, 263, 180), RightBackRocket(298, 265, -60), LeftBackRocket(26, 265, 240),
        RightThirdBay(207.0, 307.0, 0.0), LeftHatchReload(23, 15, 90), RightHatchReload(301, 15, 90),
        RightNearRocket(298, 198, 60);

        private Pose pose;

        FieldLocations(Pose pose) {
            this.pose = pose;
        }

        FieldLocations(double x, double y, double theta) {
            this.pose = new Pose(x, y, theta);
        }

        public Pose getLocation() {
            return pose;
        }
    }

    protected abstract void initAuton();

    protected abstract AutonStartingLocation getStartingLocation();

    protected StartingDirection getStartingDirection() {
        return StartingDirection.FORWARD;
    }

    protected void init() {
        initAuton();
        initStartingDirection();
    }

    public void startAuton() {
        start();
    }

    @Override
    public synchronized void start() {
        initStartingDirection();
        initStartingLocation(getStartingLocation());
        new FFDashboard("Graph").putBoolean("start", true);
        super.start();
    }

    public void initAndStartAuton() {
        init();
        startAuton();
    }

    private void initStartingLocation(AutonStartingLocation startingLocation) {
        RobotState.getInstance().setPose(startingLocation.getStartingPose(), Pose.priority.HIGH);
    }

    private void initStartingDirection() {
        RobotState.getInstance().setStartingDirection(getStartingDirection());
    }

    private ProfileLoader getProfileInfo(String file) {
        ProfileLoader loader = new ProfileLoader();
        loader.storeTrajectory(file);
        return loader;
    }

    public void froggySequentialDrive(String file) {
        ProfileLoader loader = getProfileInfo(file);
        addSequential(
                new PurePursuitDrive(loader.getTrajectory(), loader.getReversed(), loader.getLookaheadDistance()));
    }

    public void froggySequentialDrive(String file, double timeout) {
        ProfileLoader loader = getProfileInfo(file);
        addSequential(new PurePursuitDrive(loader.getTrajectory(), loader.getReversed(), loader.getLookaheadDistance()),
                timeout);
    }

    public void froggySequentialDrive(String file, boolean dontStop) {
        ProfileLoader loader = getProfileInfo(file);
        addSequential(
                new PurePursuitDrive(loader.getTrajectory(), loader.getReversed(), loader.getLookaheadDistance()));
    }

    public void froggySequentialDrive(String file, double timeout, boolean dontStop) {
        ProfileLoader loader = getProfileInfo(file);
        addSequential(new PurePursuitDrive(loader.getTrajectory(), loader.getReversed(), loader.getLookaheadDistance()),
                timeout);
    }

    public void froggyParallelDrive(String file) {
        ProfileLoader loader = getProfileInfo(file);
        addParallel(new PurePursuitDrive(loader.getTrajectory(), loader.getReversed(), loader.getLookaheadDistance()));
    }

    protected void normalSequentialDrive(String file) {
        addSequential(new PurePursuitDrive(file));
    }

    protected void normalParallelDrive(String file) {
        addParallel(new PurePursuitDrive(file));
    }

    @Override
    public synchronized void cancel() {
        super.cancel();
        new FFDashboard("Graph").putBoolean("start", false);
    }
}
