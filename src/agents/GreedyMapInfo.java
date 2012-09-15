package agents;

import java.util.PriorityQueue;
import au.rmit.ract.planning.pathplanning.entity.ComputedPlan;

import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridDomain;
import pplanning.simviewer.model.GridCoord;

public class GreedyMapInfo 
{

	private GreedyCellInfo[][] cells;
	private PriorityQueue<GreedyCellInfo> openQueue;
	
	// Added this data for debugging - probably not useful in our actual algorithm,
	// is just to aid analysis.
	private int width;
	private int height;
	
	private int nClosedCount = 0;

	public GreedyMapInfo(GridDomain map) {
		this.width = map.getWidth();
		this.height = map.getHeight();
		this.cells = new GreedyCellInfo[width][height];
		this.openQueue = new PriorityQueue<GreedyCellInfo>();
	}
	
	public ComputedPlan computePlan(GridCell goal) {
		ComputedPlan plan = new ComputedPlan();

		Trace.print("Greedy Generating plan...");

		for (GreedyCellInfo cell = getCellInfo(goal);
		     cell.getParent() != null;
		     cell = cell.getParent()) {
			GridCell gc = cell.getCell();
			//Trace.print("Prepending " + gc);
			plan.prependStep(gc);
		}

		Trace.print("...Done.");

		plan.setCost(getGCost(goal));
		return plan;
	}
	
	public void add(GridCell cell, float gCost, float hCost) {
		// should only be called when no cell already exists in array
		assert getCellInfo(cell) == null;

		// add new node to array and open queue
		GreedyCellInfo cellInfo = new GreedyCellInfo(cell, gCost, hCost, CellSetMembership.OPEN);
		cells[cell.getCoord().getX()][cell.getCoord().getY()] = cellInfo;
		openQueue.offer(cellInfo);
	}
	
	public void add(GridCell cell, float gCost, float hCost, GridCell parent) {
		add(cell, gCost, hCost);
		setParent(cell, parent);
	}
	
	/**
	 * Returns true if there are no cells in the open set.
	 * @return true if there are no cells in the open set
	 */
	public boolean isOpenEmpty() {
		return openQueue.isEmpty();
	}

	/**
	 * Returns the number of cells in the open set.
	 * @return the number of cells in the open set
	 */
	public int openCount() {
		return openQueue.size();
	}
	
	/*
	 * This function just points out that we need a structure for closed list.
	 * It is only used for debugging of the output of closed list.
	 */
	public int closedCount()
	{
		return(nClosedCount);
	}

	/**
	 * Move cheapest open cell to the closed set and return it.
	 * @return the cell formerly the cheapest from the open set
	 */
	public GridCell closeCheapestOpen() {
		GreedyCellInfo cellInfo = openQueue.poll();

		assert(cellInfo != null);

		cellInfo.setCellMembership(CellSetMembership.CLOSED);
		
		// We will have better closed list management, but for now, just tally the number of times we close an entry - 
		// this will help with analysis.
		++nClosedCount;

		return cellInfo.getCell();
	}

	public GridCell getParent(GridCell cell) {
		GreedyCellInfo cellInfo = getCellInfo(cell);
		assert cellInfo != null;
		return cellInfo.getParent().getCell();
	}

	public void setParent(GridCell cell, GridCell parent) {
		GreedyCellInfo cellInfo = getCellInfo(cell);
		GreedyCellInfo parentInfo = getCellInfo(parent);

		assert cellInfo != null;
		assert parentInfo != null;

		cellInfo.setParent(parentInfo);
	}

	public float getFCost(GridCell cell) {
		GreedyCellInfo cellInfo = getCellInfo(cell);
		assert cellInfo != null;
		return cellInfo.getFCost();
	}

	public float getGCost(GridCell cell) {
		GreedyCellInfo cellInfo = getCellInfo(cell);
		assert cellInfo != null;
		return cellInfo.getGCost();
	}

	public void setGCost(GridCell cell, float gCost) {
		GreedyCellInfo cellInfo = getCellInfo(cell);
		assert cellInfo != null;
		assert cellInfo.getCellMembership() != CellSetMembership.OPEN;
		cellInfo.setGCost(gCost);
	}

	public float getHCost(GridCell cell) {
		GreedyCellInfo cellInfo = getCellInfo(cell);
		assert cellInfo != null;
		return cellInfo.getHCost();
	}

	public void setHCost(GridCell cell, float hCost) {
		GreedyCellInfo cellInfo = getCellInfo(cell);

		assert cellInfo != null;
		assert cellInfo.getCellMembership() != CellSetMembership.OPEN;

		cellInfo.setHCost(hCost);
	}

	public void setCosts(GridCell cell, float gCost, float hCost) {
		GreedyCellInfo cellInfo = getCellInfo(cell);

		assert cellInfo != null;
		assert cellInfo.getCellMembership() != CellSetMembership.OPEN;

		cellInfo.setGCost(gCost);
		cellInfo.setHCost(hCost);
	}

	/**
	 * Check if cell is in open set.
	 * @param cell the cell to check
	 * @return true if cell is in open set, otherwise false
	 */
	public boolean isOpen(GridCell cell) {
		GreedyCellInfo cellInfo = getCellInfo(cell);
		return (cellInfo == null) ? false : cellInfo.getCellMembership() == CellSetMembership.OPEN;
	}

	/**
	 * Check if cell is in closed set.
	 * @param cell the cell to check
	 * @return true if cell is in closed set, otherwise false
	 */
	public boolean isClosed(GridCell cell) {
		GreedyCellInfo cellInfo = getCellInfo(cell);
		return (cellInfo == null) ? false : cellInfo.getCellMembership() == CellSetMembership.CLOSED;
	}

	/* get cell info associated with cell. */
	private GreedyCellInfo getCellInfo(GridCell cell) {
		GridCoord gc = cell.getCoord();
		if (cells == null) Trace.print("cells is null");
		return cells[cell.getCoord().getX()][cell.getCoord().getY()];
	}
	
}
