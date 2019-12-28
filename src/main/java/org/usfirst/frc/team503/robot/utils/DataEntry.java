package org.usfirst.frc.team503.robot.utils;

import java.util.ArrayList;

public class DataEntry {
	private ArrayList<Double> valueArrayList = new ArrayList<Double>();
	public String networkTableName;
	
	
	public DataEntry(String name) {
		networkTableName = name;
	}

	public DataEntry(String name, double[] data) {
		for(double o:data) {
			valueArrayList.add(o);
		}
		networkTableName = name;
	}
	
	public void setTableKeyName(String name) {
		networkTableName = name;
	}
	
	public void setValue(ArrayList<Double> newArrayList) {
		valueArrayList = newArrayList;
	}

	public ArrayList<Double> getEntries() {
		return valueArrayList;
	}

	public void addValue(double value) {
		valueArrayList.add(value);
	}

	public double[] getArray() {
		double[] oldArray = new double[valueArrayList.size()];
		for (int i = 0; i < valueArrayList.size(); i++) {
			oldArray[i] = valueArrayList.get(i).doubleValue();
		}
		return oldArray;
	}
	
	public void clearEntries() {
		valueArrayList.clear();
	}

}
