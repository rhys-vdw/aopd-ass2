package agents;

import pplanning.simviewer.model.GridCell;

public class PerformanceMeasure 
{
	public GridCell cell;
	public int dCheapestWithError;
	public int dMax;
	public boolean initialised = false;
	
	PerformanceMeasure(GridCell cell, int dCheapestWithError, int dMax)
	{
		this.cell = cell;
		this.dCheapestWithError = dCheapestWithError;
		this.dMax = dMax;
		this.initialised = true;
	}
}
