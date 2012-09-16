package agents;

import pplanning.simviewer.model.GridCell;

class DasCellInfo implements Comparable<DasCellInfo> {
	private DasCellInfo parent = null;
	private final GridCell cell;
	private float gCost;
	private float hCost;
	private float fCost;
	private int dCheapestRaw;
	private CellSetMembership cellSetMembership;
	private int nExpansionNumber; 

	// The below are required for the d_cheapest calculation
	private int dError = 0;
	//private float dAverageError = 0.0f;
	private int nCumulativeErrorOfPartialSolution = 0;
	private int nPartialSolutionDepth = 0; // Node depth so far

	private final float EPSILON = 0.001f; // used for floating point comparisons

	public int getDepth()
	{
		return(this.nPartialSolutionDepth);
	}

	//		public float getAverageError()
	//		{
	//			return(this.dAverageError);
	//		}

	public int getCumulativeError()
	{
		return(nCumulativeErrorOfPartialSolution);
	}

	public float getAverageError()
	{
		float avgError;
		if (this.nPartialSolutionDepth != 0)
		{
			avgError = (float)this.nCumulativeErrorOfPartialSolution / (float)this.nPartialSolutionDepth;
		}
		else
		{
			avgError = 0.0f;
		}
		//System.out.println("avgError: " + avgError);
		return(avgError);
	}

	/**
	 * Set the estimated number of expansions from this node to the goal state.
	 * This is the raw estimate, not including mean step error.
	 *
	 * TODO: Should this be a float? I think int makes sense for a d estimate.
	 *
	 * @param dCheapestRaw the new d value
	 */
	public void setDCheapestRaw(int dCheapestRaw) {
		this.dCheapestRaw = dCheapestRaw;
	}

	/**
	 * Get the estimated number of expansions from this node to the goal state.
	 * This is the raw estimate, not including mean step error.
	 *
	 * @return the estimated number of expansions to the goal state
	 */
	public int getDCheapestRaw() {
		return this.dCheapestRaw;
	}

	public float getDCheapestWithError() {
		float dCheapestWithError = 0.0f;
		float avgError = this.getAverageError();

		//System.out.println("avgError: " + avgError);

		if (avgError < 1.0f-EPSILON)
		{
			dCheapestWithError = (float)this.dCheapestRaw / (1.0f-avgError);
		}
		else
		{
			dCheapestWithError = Float.POSITIVE_INFINITY;
		}

		return(dCheapestWithError);
	}

	/** Read only. */
	public GridCell getCell() {
		return this.cell;
	}

	public DasCellInfo getParent() {
		return this.parent;
	}

	// TODO: Consider the implications to partialSolutionDepth when the parent is
	// changed. Do we need to address this?
	public void setParent(DasCellInfo parent) {
		this.nPartialSolutionDepth = parent.getDepth() + 1;
		this.parent = parent;
	}

	public float getGCost() {
		return this.gCost;
	}
	public void setGCost(float gCost) {
		this.gCost = gCost;
		updateFCost();
	}

	public float getHCost() {
		return this.hCost;
	}

	public void setHCost(float hCost) {
		this.hCost = hCost;
		updateFCost();
	}

	/** Read only. */
	public float getFCost() {
		return this.fCost;
	}

	public CellSetMembership getCellMembership() {
		return this.cellSetMembership;
	}

	public void setCellMembership(CellSetMembership cellMembership) {
		this.cellSetMembership = cellMembership;
	}

	public int getExpansionNumber()
	{
		return(this.nExpansionNumber);
	}
	
	public void setExpansionNumber(int _expansionNumber)
	{
		nExpansionNumber = _expansionNumber;
	}

	/**
	 * Compare the f cost of this cell to another.
	 * @other the CellInfo to be compared
	 */
	public int compareTo(DasCellInfo other) {
		// compare f cost of either cell

		int nReturnValue;
		// Check if the values are "the same" 
		if (Math.abs(this.fCost - other.fCost) < EPSILON)
		{
			if (Math.abs(this.hCost - other.hCost) < EPSILON)
			{
				// Both f and h are the same, in which case, return 0
				//float err1 = 0;
				//float err2 = 1;
				System.out.println(this.cell.getCoord() + "with h" + this.hCost + "is the same as " + other.cell.getCoord() + "with h" + other.hCost);
				//System.out.println("tie breaker on depth " + err1 + " vs. " + err2);
				//if (err1 != err2)
				//	System.out.println("test");
				//if (err1 < err2)
				//{
				//	nReturnValue = -1;
				//}
				//else if (err1 > err2)
				//{
				//	nReturnValue = 1;
				//}
				//else
				{
					nReturnValue = 0;
				}
			}
			else if (this.hCost < other.hCost)
			{
				nReturnValue = -1;
			}
			else
			{
				nReturnValue = 1;
			}
		}
		else if (this.fCost < other.fCost)
		{
			nReturnValue = -1;
		}
		else
		{
			nReturnValue = 1;
		}
		return(nReturnValue);
	}

	/**
	 * Update the total cost estimate of this path. Should be called every time
	 * gCost of hCost is modified.
	 */
	void updateFCost() {
		this.fCost = gCost + hCost;
	}

	/**
	 * Create a new CellInfo for specified cell with specified g and h costs.
	 * @cell            the cell
	 * @gCost           the cost of the path from start to cell
	 * @hCost           the heuristic estimate of the remaining cost to the goal
	 * @cellMembership  the set that this node should start in
	 * @parent          the parent node of this cell
	 * @expansionNumber the number of expansions that have occurred before this
	 *                  node was generated
	 * @dCheapestRaw    estimate of expansions required from this node to the goal
	 */
	public DasCellInfo(GridCell cell, float gCost, float hCost,
			CellSetMembership cellMembership, DasCellInfo parent,
			int expansionNumber, int dCheapestRaw) {
		// Set cell.
		this.cell = cell;

		// Set costs.
		this.gCost = gCost;
		this.hCost = hCost;
		updateFCost();

		// Set membership
		this.cellSetMembership = cellMembership;

		// Set d cheapest and error.
		this.dCheapestRaw = dCheapestRaw;

		// Set expansion number.
		this.nExpansionNumber = expansionNumber;

		// Set parent and partial solution cost and error.
		this.parent = parent;
		if (parent == null) {
			// This is the start node; set to zero.
			this.nPartialSolutionDepth = 0;
			this.dError = 0;
			this.nCumulativeErrorOfPartialSolution = 0;
		} else {
			// This is not the first node, base data on its parent.
			this.nPartialSolutionDepth = parent.getDepth() + 1;
			this.dError = dCheapestRaw - parent.dCheapestRaw + 1;
			this.nCumulativeErrorOfPartialSolution = parent.getCumulativeError() + dError;
		}

		/*
		System.out.println("d = " + dCheapestRaw + " d_error = " + dError + " d^ = " + getDCheapestWithError());
		*/
	}

	@Override 
	public String toString() {
		return cell + " in set " + cellSetMembership.toString();
	}
}
