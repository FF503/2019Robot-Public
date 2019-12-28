/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team503.robot.utils;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.HashMap;

import org.usfirst.frc.team503.robot.Robot;

import motionProfiling.Trajectory;
@Deprecated
public class ProfileLoaderOld {
    private HashMap<String, Trajectory> trajMap;
    private HashMap<String, Boolean> isReversedMap;
    private HashMap<String, Double> lookaheadMap;

    private String[] profiles;

    public ProfileLoaderOld(String[] profiles) {
        this();
        this.profiles = profiles;
    }

    public ProfileLoaderOld() {
        trajMap = new HashMap<String, Trajectory>();
        isReversedMap = new HashMap<String, Boolean>();
        lookaheadMap = new HashMap<String, Double>();
    }

    public String[] getProfiles() {
        return profiles;
    }

    public void setProfiles(String[] profiles) {
        this.profiles = profiles;
    }

    private void storeTrajectory(String file) {
        String fileName = file;

        if (!file.endsWith(".txt")) {
            file = file + ".txt";
        }
        if (!file.startsWith(Robot.bot.motionProfilingRioFolder)) {
            file = Robot.bot.motionProfilingRioFolder + file;
        }

        Trajectory trajectory = null;
        try (BufferedReader in = new BufferedReader(new FileReader(file))) {
            int length = Integer.parseInt(in.readLine());
            trajectory = new Trajectory(length);
            String[] data;
            for (int i = 0; i < length; i++) {
                String line = in.readLine();
                data = line.split(",");
                Trajectory.Segment seg = new Trajectory.Segment();
                seg.dt = Double.parseDouble(data[0]);
                seg.x = Double.parseDouble(data[1]);
                seg.y = Double.parseDouble(data[2]);
                seg.pos = Double.parseDouble(data[3]);
                seg.vel = Double.parseDouble(data[4]);
                seg.acc = Double.parseDouble(data[5]);
                seg.jerk = Double.parseDouble(data[6]);
                seg.heading = Double.parseDouble(data[7]);
                seg.curvature = Double.parseDouble(data[8]);
                trajectory.setSegment(i, seg);
            }
            isReversedMap.put(fileName, Boolean.parseBoolean(in.readLine()));
            while (!in.readLine().equals("end"))
                ;
            double lookahead = Robot.bot.lookAheadDistance;
            try {
                lookahead = Double.parseDouble(in.readLine());
            } catch (Exception e) {
            }

            trajMap.put(fileName, trajectory);
            lookaheadMap.put(fileName, lookahead);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public boolean doesKeyExist(String key) {
        return trajMap.containsKey(key);
    }

    public void loadAllProfiles() {
        for (int i = 0; i < profiles.length; i++) {
            storeTrajectory(profiles[i]);
        }
    }

    public Trajectory getTrajectory(String file) {
        return trajMap.get(file);
    }

    public boolean getReversed(String file) {
        return isReversedMap.get(file);
    }

    public double getLookahead(String file) {
        return lookaheadMap.get(file);
    }

    public Trajectory getTrajectory(int index) {
        return getTrajectory(profiles[index]);
    }

    public boolean getReversed(int index) {
        return getReversed(profiles[index]);
    }

}
