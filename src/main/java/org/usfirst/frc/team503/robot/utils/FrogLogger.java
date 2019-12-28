package org.usfirst.frc.team503.robot.utils;

import java.text.SimpleDateFormat;
/*
 * Writes a formated message to the system log 
 */
import java.util.Date;

/***********************************************
* Class   		FrogLogger.java
* Description	Provides logging services for the robot. 	
* @author 		Frogforce 503 Programming team 
* @version 		0.1
* Change History 
* 	0.0		Initial Load
*  
***********************************************/
public class FrogLogger {

	//String timeStamp = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss").format(new Date());

	
	public static void logMsg(String contents, String source, boolean isErr) 
	{	
		String timeStamp = new SimpleDateFormat("HH.mm.ss").format(new Date());
		if(isErr)
			System.err.println("[ERRLog] " + "(" + source + ") [" + timeStamp + "] " +contents);
		else
			System.out.println("[Log] " + "(" + source + ") [" + timeStamp + "] " +contents);
	}
	
	public static void logMsg(String contents, String source) 
	{
		logMsg(contents, source, false);
	}
	
	public static void logMsg(String contents, String source, Exception e) 
	{
		logMsg(contents, source, true);
		e.printStackTrace();
	}
}
