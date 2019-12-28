package org.usfirst.frc.team503.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.utils.FFDashboard;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/****************************************************************
 * Class Drive.java Description Provides all services for the robot drive train.
 * 
 * @author Frog Force 503 Programming team
 * @version 0.1 Change History 0.0 Initial Load
 * 
 ************************x****************************************/

public class Drive extends Subsystem {

    private boolean encoderFault;
    private static final double kDeadband = 0.02;

    private FFDashboard table = new FFDashboard("Drive");
    private FFDashboard kinematicTable = new FFDashboard("Graph");

    private CANSparkMax leftMaster, leftSlave, rightMaster, rightSlave;
    private Encoder leftEncoder, rightEncoder;
    private double lastUpdateTime = Timer.getFPGATimestamp();

    private static double[] motorCurrents = new double[2];
    private static double lastTime, curTime, lastVelR, curVelR, curAccelR, differenceCurrent, lastAccelR, curJerkR,
            lastVelL, curVelL, curAccelL, lastAccelL, curJerkL, mStartTime, curPosR, lastPosR, lastPosL, curPosL;

    /***************************************************************************************
     * Drive - Constructor
     ***************************************
     ************************************************/
    public Drive() {
        leftEncoder = new Encoder(Robot.bot.left_enc_id_1, Robot.bot.left_enc_id_2);
        rightEncoder = new Encoder(Robot.bot.right_enc_id_1, Robot.bot.right_enc_id_2);
        leftEncoder.reset();
        rightEncoder.reset();

        curTime = System.currentTimeMillis() / 1000;
        lastTime = curTime;
        mStartTime = 0;
        curAccelR = 0;
        lastAccelR = 0;
        lastVelR = 0;
        curVelR = 0;
        curAccelL = 0;
        lastAccelL = 0;
        lastVelL = 0;
        curVelL = 0;
        motorCurrents[0] = 0;
        motorCurrents[1] = 0;

        System.out.println("constructing drive train");
        // System.out.println(Robot.bot.driveSolenoidID1 + ", " +
        // Robot.bot.driveSolenoidID2);

        leftMaster = new CANSparkMax(Robot.bot.leftMasterID, MotorType.kBrushless);
        leftSlave = new CANSparkMax(Robot.bot.leftSlaveID, MotorType.kBrushless);
        rightMaster = new CANSparkMax(Robot.bot.rightMasterID, MotorType.kBrushless);
        rightSlave = new CANSparkMax(Robot.bot.rightSlaveID, MotorType.kBrushless);

        leftMaster.restoreFactoryDefaults();
        leftSlave.restoreFactoryDefaults();
        rightMaster.restoreFactoryDefaults();
        rightSlave.restoreFactoryDefaults();

        leftSlave.setSmartCurrentLimit(Robot.bot.kCurrentLimit);
        leftMaster.setSmartCurrentLimit(Robot.bot.kCurrentLimit);
        rightMaster.setSmartCurrentLimit(Robot.bot.kCurrentLimit);
        rightSlave.setSmartCurrentLimit(Robot.bot.kCurrentLimit);

        rightSlave.setOpenLoopRampRate(0.0);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        leftMaster.setInverted(Robot.bot.leftMasterReverseOutput);
        leftSlave.setInverted(Robot.bot.leftSlaveReverseOutput);
      

       
        rightMaster.setInverted(Robot.bot.rightMasterReverseOutput);
        rightSlave.setInverted(Robot.bot.rightSlaveReverseOutput);

        // rightMaster.reverseSensor(true);
        // rightMaster.reverseOutput(true);

        // leftMaster.setStatusFrameRateMs(TalonSRX.StatusFrameRate.QuadEncoder, 20);
        // rightMaster.setStatusFrameRateMs(TalonSRX.StatusFrameRate.QuadEncoder, 20);

        System.out.println("constructing drive train completed...");
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
    public void displayTemperatures(){
        System.out.println("right Temp: " + rightMaster  .getMotorTemperature());
        System.out.println("left Temp: " + leftMaster.getMotorTemperature());
        System.out.println("right slave Temp: " + rightSlave.getMotorTemperature());
        System.out.println("left slave temp: " + leftSlave.getMotorTemperature());
    }

    public boolean checkLeftEncoderFault(){
        double speed = Math.abs(getLeftVelocityInches());
        double expectedSpeed = getLeftExpectedVelocity();
        boolean difference = expectedSpeed - speed > Robot.bot.faultThreshold;
        boolean noMovement = speed < 1;
        boolean expectedVelMoving = expectedSpeed > Robot.bot.movementThresh;
        return (noMovement) && expectedVelMoving;
    }
    public boolean checkRightEncoderFault(){
        double speed = Math.abs(getRightVelocityInches());
        double expectedSpeed = getRightExpectedVelocity();
        boolean difference = expectedSpeed - speed > Robot.bot.faultThreshold;
        boolean noMovement = speed < 1;
        boolean expectedVelMoving = expectedSpeed > Robot.bot.movementThresh;
        return (noMovement) && expectedVelMoving;
    }
    public double getLeftExpectedVelocity(){
        return Math.abs(leftMaster.get()/Robot.bot.kV_PurePursuit);
    }

    public double getRightExpectedVelocity(){
        return Math.abs(rightMaster.get()/Robot.bot.kV_PurePursuit);
    }

    private CANSparkMax getLeftMotor() {
        return leftMaster;
    }

    private CANSparkMax getRightMotor() {
        return rightMaster;
    }

    /********************************************************************************
     * Section - Routines for Driving Robot
     ******************************************************************************/

    /*
     * Arcade Drive Routine
     */

    public void arcadeDrive(double moveValue, double rotateValue) {
        double leftMotorSpeed;
        double rightMotorSpeed;

        moveValue = limit(moveValue);
        rotateValue = limit(rotateValue);

        moveValue = applyDeadband(moveValue, kDeadband);
        rotateValue = applyDeadband(rotateValue, kDeadband);

        if (moveValue > 0.0) {
            if (rotateValue > 0.0) {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = Math.max(moveValue, rotateValue);
            } else {
                leftMotorSpeed = Math.max(moveValue, -rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            }
        } else {
            if (rotateValue > 0.0) {
                leftMotorSpeed = -Math.max(-moveValue, rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            } else {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
            }
        }
        setMotorOutputs(leftMotorSpeed, rightMotorSpeed);
    }

    /*
     * Tank Drive Routine
     */

    public void tankDrive(double leftValue, double rightValue) {
        leftValue = limit(leftValue);
        rightValue = limit(rightValue);

        setMotorOutputs(leftValue, rightValue);
    }

    public void tankDrive(DriveMotorOutput output) {
        double leftValue = output.getLeftPercent();
        double rightValue = output.getRightPercent();

        tankDrive(leftValue, rightValue);
    }

    /*
     * Limit values from -1 to +1
     */
    private static double limit(double num) {
        if (num > 1.0) {
            num = 1.0;
        } else if (num < -1.0) {
            num = -1.0;
        }
        return num;
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The
     * remaining range between the kDeadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value     value to clip
     * @param kDeadband range around zero
     */
    private static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    /*
     * Send command to drive motors
     */

    public void setMotorOutputs(double leftSpeed, double rightSpeed) {

        // if (RobotState.getInstance().getDriveMode() ==
        // RobotState.RobotDriveMode.CLIMB
        // && ((leftSpeed < 0.0) || (rightSpeed < 0.0))) {
        // leftMaster.set(ControlMode.PercentOutput, 0.0);
        // rightMaster.set(ControlMode.PercentOutput, 0.0);
        // } else {
        if ((Robot.bot.leftMasterReverseOutput || Robot.bot.getName().equals("ProgrammingBot"))
                && RobotState.getInstance().getState() != RobotState.State.AUTON) {
            double temp = leftSpeed;
            leftSpeed = rightSpeed;
            rightSpeed = temp;
        }
        //System.out.println("Drive Setting Motors, Left:" + -leftSpeed + " Right:" + rightSpeed );
        leftMaster.set(-leftSpeed);
        rightMaster.set(rightSpeed);
        //leftSlave.set(-leftSpeed);
        //rightSlave.set(rightSpeed);
        // }
    }

    public void zeroPowerMotors() {
        tankDrive(0.0, 0.0);
    }

    /********************************************************************************
     * Section - Routines to interface with Encoders
     *******************************************************************************/

    /*
     * Reset both encoder positions to 0
     */

    public void resetEncoders() {
        /*
         * leftMaster.setPosition(0); rightMaster.setPosition(0);
         * 
         * leftMaster.setEncPosition(0); rightMaster.setEncPosition(0);
         */
        leftEncoder.reset();
        rightEncoder.reset();
    }

    /*
     * Read Left Encoder in rotations and convert to inches
     */

    public double getLeftDistanceInches() {
        double encPos = getLeftPosition();
        return ticksToInches(encPos);
    }

    /**
     * Gets left motor speed
     * 
     * @return left speed in inches/sec
     */

    public double getLeftVelocityInches() {
        return curVelL;
    }

    /**
     * Gets right motor speed
     * 
     * @return right speed in inches/sec
     */

    public double getRightVelocityInches() {
        return curVelR;
    }

    /***
     * gets overall robot velocity
     * 
     * @return robot velocity in inches/sec
     */
    public double getRobotVelocity() {
        return (curVelL + curVelR) / 2;
    }

    /*
     * Get difference of drive motor currents
     */

    public double getCurrentDiff(boolean abs) {
        motorCurrents[0] = leftMaster.getOutputCurrent();
        motorCurrents[1] = rightMaster.getOutputCurrent();

        if (abs) {
            differenceCurrent = Math.abs(Math.abs(motorCurrents[0]) - Math.abs(motorCurrents[1]));
        } else
            differenceCurrent = motorCurrents[0] - motorCurrents[1];

        return differenceCurrent;
    }

    /**
     * Read Right Encoder in rotations and convert to inches
     */

    public double getRightDistanceInches() {
        double encPos = getRightPosition();
        return ticksToInches(encPos);
    }

    public double getAverageEncoderCounts() {
        return (getLeftPosition() + getRightPosition()) / 2.0;
    }

    /**
     * Return average of left and right encoder values in inches
     */

    public double getAverageDistanceInches() {
        double encPos = (getLeftDistanceInches() + getRightDistanceInches()) / 2;
        return encPos;
    }

    /**
     * Read Left Encoder and return absolute click count
     * 
     * @return Native units of encoder
     */

    public double getLeftPosition() {
        // the talon optical encoder returns position in revolutions-multiply times
        // counts/rev to get to position
        // mult by 4 because quad
        return leftEncoder.getDistance() * 4;
    }

    /**
     * Read Right Encoder and return absolute click count
     * 
     * @return Native units of encoder
     */

    public double getRightPosition() {
        // the talon optical encoder returns position in revolutions-multiply times
        // counts/rev to get to position
        // mult by 4 because quad
        return rightEncoder.getDistance() * 4; // getposition = enc rotations
    }

    /********************************************************************************
     * Section - Encoder Conversion Routines
     *******************************************************************************/

    private static double ticksToInches(double ticks) {
        return rotationsToInches(ticksToRotations(ticks));
        // return(ticks / (double) Robot.bot.kEncoderCountsperRev) * ((double)
        // Robot.bot.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Robot.bot.kDriveWheelDiameterInches * Math.PI);
    }

    private static double ticksToRotations(double ticks) {
        return ticks / Robot.bot.kEncoderUnitsPerRev;
        // return (ticks / Robot.bot.kEncoderCountsperRev);
    }

    private static double inchesToRotations(double inches) {
        return inches / (Robot.bot.kDriveWheelDiameterInches * Math.PI);
    }

    public void setBrakeMode(boolean on) {
        IdleMode mode = on ? IdleMode.kBrake : IdleMode.kCoast;
        leftMaster.setIdleMode(mode);
        leftSlave.setIdleMode(mode);
        rightMaster.setIdleMode(mode);
        rightSlave.setIdleMode(mode);
    }

    public void calculateKinematicData() {
        curPosL = getLeftDistanceInches();
        curPosR = getRightDistanceInches();
        curTime = Timer.getFPGATimestamp() - mStartTime;
        curVelR = derivative(curPosR, lastPosR, curTime, lastTime);
        curVelL = derivative(curPosL, lastPosL, curTime, lastTime);
        curAccelR = derivative(curVelR, lastVelR, curTime, lastTime);
        curAccelL = derivative(curVelL, lastVelL, curTime, lastTime);
        lastVelR = curVelR;
        lastVelL = curVelL;
        curJerkR = derivative(curAccelR, lastAccelR, curTime, lastTime);
        curJerkL = derivative(curAccelL, lastAccelL, curTime, lastTime);
        lastAccelR = curAccelR;
        lastAccelL = curAccelL;
        lastTime = curTime;
        lastPosL = curPosL;
        lastPosR = curPosR;

     //   kinematicTable.putNumber("Time", curTime);
        kinematicTable.putNumber("Velocity", (curVelR + curVelL) / 2);
        kinematicTable.putNumber("Acceleration", Math.min(curAccelL, curAccelR));
        kinematicTable.putNumber("Jerk", Math.min(curJerkR, curJerkL));
    }

    public void setTimerOffset() {
        mStartTime = Timer.getFPGATimestamp();
    }

    private static double rpmToInchesPerSecond(int rpm) {
        return rotationsToInches(rpm) / 60;
    }

    public double derivative(double vel2, double vel1, double t2, double t1) {
        return (vel2 - vel1) / (t2 - t1);
    }

    public double getAngularVelocity() {
        double w = getRightVelocityInches() - getLeftVelocityInches();
        w /= Robot.bot.WHEEL_BASE_INCHES;
        return w;
    }

    private static Drive instance = new Drive();

    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }
        return instance;
    }

    public void sendDashboardData() {
        // SmartDashboard.putNumber("Motor current difference",
        // Drive.getInstance().getCurrentDiff(true));
        // SmartDashboard.putNumber("Left master current",
        // leftMaster.getOutputCurrent());
        // SmartDashboard.putNumber("Right master current",
        // rightMaster.getOutputCurrent());
        SmartDashboard.putNumber("Left encoder counts", getLeftPosition());

        // leftMaster.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Right encoder counts", getRightPosition());
        // SmartDashboard.putNumber("Left motor output",
        // leftMaster.getMotorOutputPercent());
        // SmartDashboard.putNumber("Right motor output",
        // rightMaster.getMotorOutputPercent());
        // SmartDashboard.putNumber("Left encoder inches", getLeftDistanceInches());
        // SmartDashboard.putNumber("Right encoder inches", getRightDistanceInches());
        // SmartDashboard.putNumber("Average encoder counts",
        // getAverageEncoderCounts());
        SmartDashboard.putNumber("Right Motor Velocity", getRightVelocityInches());
        SmartDashboard.putNumber("Left Motor Velocity", getLeftVelocityInches());
        SmartDashboard.putNumber("Right Motor Accel", curAccelR);
        SmartDashboard.putNumber("Left Motor Accel", curAccelL);
        SmartDashboard.putNumber("left motor inches", curPosL);
        SmartDashboard.putNumber("right motor inches", curPosR);
        // SmartDashboard.putNumber("Min Motor Vel", Math.min(getRightVelocityInches(),
        // getLeftVelocityInches()));
        // SmartDashboard.putNumber("Min Motor Accel", Math.min(curAccelR, curAccelL));
        // SmartDashboard.putNumber("Min Motor Jerk", Math.min(curJerkL, curJerkR));
        // SmartDashboard.putNumber("Left master bus voltage",
        // leftMaster.getBusVoltage());
        // SmartDashboard.putNumber("Right master bus voltage",
        // rightMaster.getBusVoltage());
        // SmartDashboard.putNumber("Left master output voltage",
        // leftMaster.getMotorOutputVoltage());
        // SmartDashboard.putNumber("Right master output voltage",
        // rightMaster.getMotorOutputVoltage());
        // SmartDashboard.putBoolean("Drive motion profile done",
        // RobotState.getInstance().getDriveProfileDone());
        // SmartDashboard.putBoolean("Robot is reversed",
        // RobotState.getInstance().getDriveReversed());

        table.putString("Robot Drive Mode", RobotState.getInstance().getDriveMode().toString());
        table.putNumber("Left Power", leftMaster.get() * 100);
        table.putNumber("Right Power", rightMaster.get() * 100);
        table.putNumber("Robot Angular Velocity", getAngularVelocity());
        table.putBoolean("isRobotReversed", RobotState.getInstance().getDriveReversed());
        // Divide by 2 for average then by 12 to get it in feet
        table.putNumber("Average Robot Velocity", (getLeftVelocityInches() + getRightVelocityInches()) / 24);

        table.putNumber("Left Position", getLeftDistanceInches());
        table.putNumber("Right Position", getRightDistanceInches());
        table.putBoolean("left Encoder Fault:", checkLeftEncoderFault());
        table.putBoolean("right Encoder Fault:", checkRightEncoderFault());
        table.putNumber("Left Velocity", getLeftVelocityInches());
        table.putNumber("Right Velocity", getRightVelocityInches());
        table.putNumber("Left Acceleration", curAccelL);
        table.putNumber("Right Acceleration", curAccelR);
        table.putNumber("Left Jerk", curJerkL);
        table.putNumber("Right Jerk", curJerkR);
        table.putNumber("Left Master Temp", leftMaster.getMotorTemperature());
        table.putNumber("Right Master Temp", rightMaster.getMotorTemperature());
        table.putNumber("Left Slave Temp", leftSlave.getMotorTemperature());
        table.putNumber("Right Slave Temp", rightSlave.getMotorTemperature());
        // table.putNumber("Left Slave Voltage In", leftSlave.getBusVoltage());
        // table.putNumber("Right Slave Voltage In", rightSlave.getBusVoltage());
        // table.putNumber("Left Master Voltage In", leftMaster.getBusVoltage());
        // table.putNumber("Right Master Voltage In", rightMaster.getBusVoltage());
        table.putBoolean("Right Temp Fault", rightMaster.getMotorTemperature() > 60.0);
        table.putBoolean("Left Temp Fault", leftMaster.getMotorTemperature() > 60.0);
    }

    public void updateDt() {
        FFDashboard.getInstance().putNumber("Time", getTimeElapsed());
    }

    public double getTimeElapsed() {
        double currentTime = Timer.getFPGATimestamp();
        return currentTime - mStartTime;
    }

    public void resetTimer() {
        setTimerOffset();
    }

    public static class DriveMotorOutput {
        private double leftPercent = 0, rightPercent = 0, leftVelocity = 0, rightVelocity = 0, leftAccel = 0,
                rightAccel = 0;

        public DriveMotorOutput(double leftVel, double rightVel) {
            setVelocity(leftVel, rightVel);
        }

        public DriveMotorOutput() {
        }

        public void setVelocity(double left, double right) {
            setLeftVelocity(left);
            setRightVelocity(right);
        }

        public void setLeftVelocity(double left) {
            this.leftVelocity = left;
        }

        public void setRightVelocity(double right) {
            this.rightVelocity = right;
        }

        public double getRightVelocity() {
            return rightVelocity;
        }

        public double getLeftVelocity() {
            return leftVelocity;
        }

        public void setLeftPercent(double left) {
            leftPercent = left;
        }

        public void setRightPercent(double right) {
            rightPercent = right;
        }

        public double getRightPercent() {
            return rightPercent;
        }

        public double getLeftPercent() {
            return leftPercent;
        }

        private void setLeftAccel(double left) {
            leftAccel = left;
        }

        private void setRightAccel(double right) {
            rightAccel = right;
        }

        public double getLeftAccel() {
            return leftAccel;
        }

        public double getRightAccel() {
            return rightAccel;
        }

        public void setAccelerations(double left, double right) {
            setLeftAccel(left);
            setRightAccel(right);
        }

        public void setPercentPowers(double left, double right) {
            setLeftPercent(left);
            setRightPercent(right);
        }

        public void reverseDrive() {
            double tempLeft = leftVelocity;
            leftVelocity = -rightVelocity;
            rightVelocity = -tempLeft;

            double tempAccel = leftAccel;
            leftAccel = -rightAccel;
            rightAccel = -tempAccel;

        }

        public void setDriveDirection(boolean bool) {
            // double sign = bool ? -1 : 1;
            /*
             * double tempLeft = leftVelocity; leftVelocity = -rightVelocity; rightVelocity
             * = -tempLeft;
             * 
             * double tempAccel = leftAccel; leftAccel = -rightAccel; rightAccel =
             * -tempAccel;
             */
            System.out.println("REVERSE OUTPUT: " + bool);

            if (Robot.bot.leftMasterReverseOutput || Robot.bot.getName().equals("ProgrammingBot")) {
                double temp = leftPercent;
                leftPercent = rightPercent;// * mult;
                rightPercent = temp;// * mult;
            }
            if (bool) {

                leftPercent = leftPercent * -1;
                rightPercent = rightPercent * -1;
            }

        }
    }

}
