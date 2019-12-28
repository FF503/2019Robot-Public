package org.usfirst.frc.team503.robot.utils;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ErrorCollection;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPXPIDSetConfigUtil;
import com.ctre.phoenix.motorcontrol.can.VictorSPXPIDSetConfiguration;

//import edu.wpi.first.wpilibj.hal.HAL;
/**
 * VEX Victor SPX Motor Controller when used on CAN Bus.
 */
public class FroggyVictorSPX extends com.ctre.phoenix.motorcontrol.can.BaseMotorController
    implements IMotorControllerEnhanced {
		
	/**
	 * Constructor
	 * 
	 * @param deviceNumber
	 *            [0,62]
	 */
	public FroggyVictorSPX(int deviceNumber) {
		super(deviceNumber | 0x01040000);
	}


    /**
     * Gets all PID set persistant settings.
     *
	 * @param pid               Object with all of the PID set persistant settings
	 * @param pidIdx            0 for Primary closed-loop. 1 for auxiliary closed-loop.
     * @param timeoutMs
     *              Timeout value in ms. If nonzero, function will wait for
     *              config success and report an error if it times out.
     *              If zero, no blocking or checking is performed.
     */
    public void getPIDConfigs(VictorSPXPIDSetConfiguration pid, int pidIdx, int timeoutMs)
    {
        baseGetPIDConfigs(pid, pidIdx, timeoutMs);
        pid.selectedFeedbackSensor = RemoteFeedbackDevice.valueOf(configGetParameter(ParamEnum.eFeedbackSensorType, pidIdx, timeoutMs));
    
    }

    
    /**
	 * Gets the output current of the motor controller.
	 *
	 * @return The output current (in amps).
	 */
	public double getOutputCurrent() {
		return super.getOutputCurrent();
	}
    /**
     * Gets all PID set persistant settings (overloaded so timeoutMs is 50 ms
     * and pidIdx is 0).
     *
	 * @param pid               Object with all of the PID set persistant settings
     */
	public void getPIDConfigs(VictorSPXPIDSetConfiguration pid) {
        int pidIdx = 0;
        int timeoutMs = 50;
        getPIDConfigs(pid, pidIdx, timeoutMs);
    }	
    /**
     * Gets all persistant settings.
     *
	 * @param allConfigs        Object with all of the persistant settings
     * @param timeoutMs
     *              Timeout value in ms. If nonzero, function will wait for
     *              config success and report an error if it times out.
     *              If zero, no blocking or checking is performed.
     */
    public void getAllConfigs(VictorSPXConfiguration allConfigs, int timeoutMs) {
    
        baseGetAllConfigs(allConfigs, timeoutMs);
    
        getPIDConfigs(allConfigs.primaryPID, 0, timeoutMs);
        getPIDConfigs(allConfigs.auxiliaryPID, 1, timeoutMs);
        allConfigs.sum0Term =  RemoteFeedbackDevice.valueOf(configGetParameter(ParamEnum.eSensorTerm, 0, timeoutMs));
        allConfigs.sum1Term =  RemoteFeedbackDevice.valueOf(configGetParameter(ParamEnum.eSensorTerm, 1, timeoutMs));
        allConfigs.diff0Term = RemoteFeedbackDevice.valueOf(configGetParameter(ParamEnum.eSensorTerm, 2, timeoutMs));
        allConfigs.diff1Term = RemoteFeedbackDevice.valueOf(configGetParameter(ParamEnum.eSensorTerm, 3, timeoutMs));
    
        allConfigs.forwardLimitSwitchSource = RemoteLimitSwitchSource.valueOf(configGetParameter(ParamEnum.eLimitSwitchSource, 0, timeoutMs));
        allConfigs.reverseLimitSwitchSource = RemoteLimitSwitchSource.valueOf(configGetParameter(ParamEnum.eLimitSwitchSource, 1, timeoutMs));
        allConfigs.forwardLimitSwitchDeviceID = (int) configGetParameter(ParamEnum.eLimitSwitchRemoteDevID, 0, timeoutMs);
        allConfigs.reverseLimitSwitchDeviceID = (int) configGetParameter(ParamEnum.eLimitSwitchRemoteDevID, 1, timeoutMs);
        allConfigs.forwardLimitSwitchNormal = LimitSwitchNormal.valueOf(configGetParameter(ParamEnum.eLimitSwitchNormClosedAndDis, 0, timeoutMs));
        allConfigs.reverseLimitSwitchNormal = LimitSwitchNormal.valueOf(configGetParameter(ParamEnum.eLimitSwitchNormClosedAndDis, 1, timeoutMs));
    
    }
    /**
     * Gets all persistant settings (overloaded so timeoutMs is 50 ms).
     *
	 * @param allConfigs        Object with all of the persistant settings
     */
    public void getAllConfigs(VictorSPXConfiguration allConfigs) {
        int timeoutMs = 0;
        getAllConfigs(allConfigs, timeoutMs);
    }

    @Override
    public ErrorCode setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs, int timeoutMs) {
        return null;
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
            int timeoutMs) {
        return null;
    }

    @Override
    public ErrorCode configPeakCurrentLimit(int amps, int timeoutMs) {
        return null;
    }

    @Override
    public ErrorCode configPeakCurrentDuration(int milliseconds, int timeoutMs) {
        return null;
    }

    @Override
    public ErrorCode configContinuousCurrentLimit(int amps, int timeoutMs) {
        return null;
    }

    @Override
    public void enableCurrentLimit(boolean enable) {

    }

    @Override
	public SensorCollection getSensorCollection() {
		return null;
	}


}
