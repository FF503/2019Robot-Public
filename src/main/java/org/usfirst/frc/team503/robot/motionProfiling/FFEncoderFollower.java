package org.usfirst.frc.team503.robot.motionProfiling;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Trajectory;
/**
 *
 */
public class FFEncoderFollower {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	int encoder_offset, encoder_tick_count;
    double wheel_circumference;
    boolean isReversed;
    int resetPoint;
    double resetError;
    
    double kp, ki, kd, kv, ka;

    double last_error, heading;

    int segment;
    Trajectory trajectory;
    

    public FFEncoderFollower(Trajectory trajectory2, boolean isReversed) {
        this.trajectory = trajectory2;
        this.isReversed = isReversed;
    }

    public FFEncoderFollower() { 
    	
    }

    /**
     * Set a new trajectory to follow, and reset the cumulative errors and segment counts
     */
    public void setTrajectory(Trajectory traj) {
        this.trajectory = traj;
        reset();
    }

    /**
     * Configure the PID/VA Variables for the Follower
     * @param kp The proportional term. This is usually quite high (0.8 - 1.0 are common values)
     * @param ki The integral term. Currently unused.
     * @param kd The derivative term. Adjust this if you are unhappy with the tracking of the follower. 0.0 is the default
     * @param kv The velocity ratio. This should be 1 over your maximum velocity @ 100% throttle.
     *           This converts m/s given by the algorithm to a scale of -1..1 to be used by your
     *           motor controllers
     * @param kA2 The acceleration term. Adjust this if you want to reach higher or lower speeds faster. 0.0 is the default
     */
    public void configurePIDVA(double kp, double ki, double kd, double kv, double kA) {
        this.kp = kp; this.ki = ki; this.kd = kd;
        this.kv = kv; this.ka = kA;
    }

    /**
     * Configure the Encoders being used in the follower.
     * @param initial_position      The initial 'offset' of your encoder. This should be set to the encoder value just
     *                              before you start to track
     * @param ticks_per_revolution  How many ticks per revolution the encoder has
     * @param wheel_diameter        The diameter of your wheels (or pulleys for track systems) in meters
     */
    public void configureEncoder(int initial_position, int ticks_per_revolution, double wheel_diameter) {
        encoder_offset = initial_position;
        encoder_tick_count = ticks_per_revolution;
        wheel_circumference = Math.PI * wheel_diameter;
    }

    /**
     * Reset the follower to start again. Encoders must be reconfigured.
     */
    public void reset() {
        last_error = 0; segment = 0;
    }

    /**
     * Calculate the desired output for the motors, based on the amount of ticks the encoder has gone through.
     * This does not account for heading of the robot. To account for heading, add some extra terms in your control
     * loop for realignment based on gyroscope input and the desired heading given by this object.
     * @param encoder_tick The amount of ticks the encoder has currently measured.
     * @return             The desired output for your motor controller
     */
    public double calculate(int encoder_tick) {
        // Number of Revolutions * Wheel Circumference
        double distance_covered = Math.abs(((double)(encoder_tick - encoder_offset) / encoder_tick_count)
                * wheel_circumference);
        if (segment < trajectory.length()) {
            Trajectory.Segment seg = trajectory.get(segment);
            double error;
          
           error = seg.position - resetError - distance_covered;
            double calculated_value =
                    kp * error +                                    // Proportional
                    kd * ((error - last_error) / seg.dt) +          // Derivative
                    (kv * seg.velocity + ka * seg.acceleration);    // V and A Terms
            last_error = error;
            heading = seg.heading;
            segment++;
            System.out.println("Segment: " + segment);
            System.out.println("Position: " + distance_covered);
            System.out.println("Motor output: " + calculated_value);
            SmartDashboard.putNumber("Motion profile error", error);
            
         
            
            if (isReversed) {
            	return -calculated_value;
            }
            else {
            	return calculated_value;
            }
            
        } else return 0;
    }

    /**
     * @return the desired heading of the current point in the trajectory
     */
    public double getHeading() {
        return heading;
    }

    /**
     * @return the current segment being operated on
     */
    public Trajectory.Segment getSegment() {
        return trajectory.get(segment);
    }

    /**
     * @return whether we have finished tracking this trajectory or not.
     */
    public boolean isFinished() {
        return segment >= trajectory.length();
    }

    public int getSegNum(){
        return segment;
    }
    public int getNumSegments(){
        return trajectory.length();
    }

    public void setResetPoint(int reset) {
    	resetPoint = reset;
    }
}
