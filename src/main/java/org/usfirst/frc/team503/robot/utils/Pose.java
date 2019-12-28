package org.usfirst.frc.team503.robot.utils;

public class Pose {
	private double x = 503.503503503, y = 503.503503503, theta = 503.503503503, lastX = 503.503503503, lastY = 503.503503503, lastTheta = 503.503503503;
	public enum priority{
		LOW, HIGH;
	}
	public Pose(double x, double y, double theta) {
		this.x = x;
		this.y = y;
		this.theta = theta;
	}
	public Pose(double[] coord){
		this.x = coord[0];
		this.y = coord[1];
		this.theta = 0;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public double getTheta() {
		return theta;
	}

	public void update(double x, double y, double theta) {
		this.x = x;
		this.y = y;
		this.theta = theta;
	}

	public void increment(double x, double y, double theta) {
		this.x += x;
		this.y += y;
		this.theta += theta;
	}

	public double[] get() {
		return new double[] { x, y, theta };
	}

	public void updateX(double x) {
		lastX = this.x;
		this.x = x;
	} 

	public void updateY(double y) {
		lastY = this.y; 
		this.y = y;
	}

	public void translatePose(Vector2D translation){
		// this.x += Math.cos(Math.toRadians(this.theta))*centerOfRotDist;
		// this.y += Math.sin(Math.toRadians(this.theta))*centerOfRotDist;
		double deltaX = translation.x();
		double deltaY = translation.y();
		double x = this.getX();	
		double y = this.getY();
		double alpha = this.getTheta();
		double cos = (Math.cos(Math.toRadians(alpha)));
		double sin = (Math.sin(Math.toRadians(alpha)));
		double x1 = x + (cos*deltaX)- (sin*deltaX);
		//Math.sin(Math.toRadians(alpha))*deltaX + Math.cos(Math.toRadians(alpha))*deltaY;
		double sign = Math.signum(deltaX*deltaX + deltaY*deltaY - (x1*x1));
		double y1 = y + (sin*deltaX) + (cos*deltaX);//sign*Math.sqrt(Math.abs(deltaX*deltaX + deltaY*deltaY - (x1*x1))) + y;// x + Math.sin(Math.toRadians(alpha))*deltaY + Math.cos(Math.toRadians(alpha))*deltaX;//Math.sqrt(deltaX*deltaX + deltaY*deltaY - (x1*x1)) + y;
		this.x = x1;
		this.y = y1;
		this.theta = alpha;
	}

	public void updateTheta(double theta) {
		lastTheta = this.theta;
		this.theta = theta;
	}

	public double getLastX() {
		return lastX;
	}

	public double getLastY() {
		return lastY;
	}

	public double getLastTheta() {
		return lastTheta;
	}
	public Vector2D toVector(){
		return (new Vector2D(this.x, this.y));
	}
	public Pose copy(){
		if (x == 503.503503503){
			x = 0.0; y=0.0; theta=0.0;
			System.out.println("incident");
		}
		return new Pose(x,y,theta);
	}
	public String toString(){
		return "x:" + x + " y: " + y + " theta: " + theta;
	}

	public static Pose weightAvg(Pose odometry, Pose camera, Vector2D weight) {
		return new Pose((odometry.x*weight.i()+camera.x*weight.j())/(weight.i()+weight.j()),(odometry.y*weight.i()+camera.y*weight.j())/(weight.i()+weight.j()), odometry.theta);
	}
}
