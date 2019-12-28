package org.usfirst.frc.team503.robot.utils;


import motionProfiling.Trajectory.Segment;

public class Vector2D {
    private double i, j;

    public Vector2D(double i, double j) {
        this.i = i;
        this.j = j;
    }
    public Vector2D(double[] coord){
        this.i = coord[0];
        this.j = coord[1];
    }  
    public Vector2D(double magnitude, double ang_rad, String referenceChanger){
        this.i = Math.cos(ang_rad)*magnitude;
        this.j = Math.sin(ang_rad)*magnitude;
    }

    public double i() {
        return i;
    }

    public double j() {
        return j;
    }

    public double x() {
        return i;
    }

    public double y() {
        return j;
    }

    public double dot(Vector2D b) {
        return this.i * b.i + this.j * b.j;
    }

    public double cross(Vector2D b) {
        return this.i * b.j - this.j * b.i;
    }

    public void translateUp(Vector2D other) {
        this.i += other.i;
        this.j += other.j;
    }

    public void translateDown(Vector2D other) {
        this.i -= other.i;
        this.j -= other.j;
    }

    public double distanceTo(Segment seg) {
        double dx = x() - seg.x;
        double dy = y() - seg.y;
        double hyp = Math.sqrt((dx * dx) + (dy * dy));
        return hyp;
    }

    public double distanceTo(Vector2D v){
        double dx = x() - v.x();
        double dy = y() - v.y();
        double hyp = Math.sqrt((dx * dx) + (dy * dy));
        return hyp;
    }

    public Vector2D add(Vector2D v){
        return (new Vector2D(v.i+i, j+v.j));
    }

    public void scale(double scale) {
        this.i *= scale;
        this.j *= scale;
    }

    public Vector2D copy() {
        return new Vector2D(i, j);
    }

    public Vector2D translateCoordinateFrames(Vector2D frame, double angle) {
        this.translateDown(frame);
        angle = Math.toRadians(angle);
        double x = (this.i * Math.sin(angle)) - (this.j * Math.cos(angle));
        double y = (this.j * Math.sin(angle)) + (this.i * Math.cos(angle));
        Vector2D temp = new Vector2D(x, y);
        return temp;
    }

    public Vector2D translateCoordinateFrames(Pose pose) {
        double angle = pose.getTheta();
        Vector2D frame = new Vector2D(pose.getX(), pose.getY());
        return translateCoordinateFrames(frame, angle);
    }
}