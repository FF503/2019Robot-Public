package org.usfirst.frc.team503.robot.utils;

import java.util.ArrayList;

import org.usfirst.frc.team503.robot.Robot;

public class UniversalGrapher{

	/**
	 * How to use the UniversalGrapher class to graph values you want 
	 * 1. Initialize the DataEntry objects with a description of the DataEntry in the constructor
	 * 2. Call (name of data entry object).addValue({value}) in a loop for as long as you want to
	 * 		send the values. What goes on the x axis on the graph should have the value added first
	 * 3. In the end of the program, call UniversalGrapher.addEntries({name of dataEntry object}) for each
	 * 		object you initialized
	 * 4. Call UniversalGrapher.sendValues(); to send the arrays to network tables
	 * 
	 * After the last method is called, arrays will be sent to networkTables with the constructor argument
	 * as the key
	 */
	
	private static ArrayList<DataEntry> allEntries = new ArrayList<DataEntry>();
	private static String[] allNTNames;
	public static int numberOfRuns = 0;

	public static boolean readyToSendValues = false;

	public static void setReadyToSend() {
		readyToSendValues = true;
	}

	public static void addEntries(DataEntry entry) {
		allEntries.add(entry);
	}

	public static void sendValues() {
		numberOfRuns++;
		allNTNames = new String[allEntries.size()];
		System.out.println(allEntries.size());
		System.out.println(allNTNames.length);
		for (int i = 0; i < allEntries.size(); i++) {
			allNTNames[i] = allEntries.get(i).networkTableName;
		}

		for (DataEntry o : allEntries) {
			Robot.universalGrapherTable.getEntry(o.networkTableName).forceSetDoubleArray(o.getArray());
		}
		
		Robot.universalGrapherTable.getEntry("Number of Runs").forceSetNumber(numberOfRuns);
		Robot.universalGrapherTable.getEntry("All Key Names").forceSetStringArray(allNTNames);

		clearAllEntries();
	}
	
	public static void clearAllEntries() {
		allNTNames = null;
		allEntries.clear();
		numberOfRuns = 0;
	}

	
}