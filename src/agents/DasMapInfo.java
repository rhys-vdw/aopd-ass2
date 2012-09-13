package agents;

import java.util.PriorityQueue;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.ListIterator;
import java.lang.IllegalStateException;
import java.lang.IllegalArgumentException;

import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridDomain;
import pplanning.simviewer.model.GridCoord;

import au.rmit.ract.planning.pathplanning.entity.ComputedPlan;

/**
 * A class to store metadata about the current map.
 *
 * Direct access to CellInfo instances cannot be provided to ensure that a cell
 * is always present in the open queue when it is in the open set.
 *
 * Nodes that are in the open set cannot be altered. Later this will probably
 * need to be changed, but for standard A* this is fine.
 *
 * I have used assertions instead of exceptions for speed (since we can disable
 * them on run).
 */
public class DasMapInfo {
	private DasCellInfo[][] cells;
	private PriorityQueue<DasCellInfo> openQueue;
	private PriorityQueue<DasCellInfo> prunedQueue;

	// Added this data for debugging - probably not useful in our actual algorithm,
	// is just to aid analysis.
	private int width;
	private int height;

	private int nClosedCount = 0;

	public DasMapInfo(GridDomain map) {
		this.width = map.getWidth();
		this.height = map.getHeight();
		this.cells = new DasCellInfo[width][height];
		this.openQueue = new PriorityQueue<DasCellInfo>();
		this.prunedQueue = new PriorityQueue<DasCellInfo>();

	}

	public ComputedPlan computePlan(GridCell goal)
	{
		ComputedPlan plan = new ComputedPlan();

		Trace.print("Generating new incumbent plan...");

		for (DasCellInfo cellInfo = getCellInfo(goal);
		     cellInfo.getParent() != null;
		     cellInfo = cellInfo.getParent())
		{
			GridCell cell = cellInfo.getCell();
			//System.out.println("Prepending " + gc);
			plan.prependStep(cell);
		}

		Trace.print("...Done.");

		plan.setCost(getGCost(goal));
		return plan;
	}

	public void addStartCell(GridCell cell, float hCost, int dCheapestRaw) {
		// Start cell has zero gCost, and no parent.
		add(cell, 0f, hCost, dCheapestRaw, 0, null);
	}

	/**
	 * Add cell to open set. This will fail if cell has already been added.
	 * @param cell           the cell
	 * @param gCost          the cost to get to the cell
	 * @param hCost          the heuristic estimate to get to the goal
	 * @param dCheapestRaw   the estimated goaldepth from the cell
	 * @param expansionCount the number of expansions performed before this cell
	 *                       was generated
	 * @param parent         the previous cell in a path
	 */
	public void add(GridCell cell, float gCost, float hCost, int dCheapestRaw,
			int expansionCount, GridCell parent)
	{
		// Should only be called when no info exists for node.
		if (getCellInfo(cell) != null) {
			throw new IllegalArgumentException("Cannot add cell " + cell +
					", which has already been added as '" + getCellInfo(cell) + "'");
		}

		// Find cell info associated with parent.
		DasCellInfo parentInfo = (parent == null) ? null : getCellInfo(parent);

		// Create new cell info.
		DasCellInfo cellInfo = new DasCellInfo(cell, gCost, hCost,
				CellSetMembership.OPEN, parentInfo, expansionCount, dCheapestRaw);

		// add new node to array and open queue
		cells[cell.getCoord().getX()][cell.getCoord().getY()] = cellInfo;
		openQueue.offer(cellInfo);
	}

	/**
	 * Returns true if there are no cells in the open set.
	 * @return true if there are no cells in the open set
	 */
	public boolean isOpenEmpty() {
		return (openQueue.size() == 0);
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
	public int closedCount() {
		return nClosedCount ;
	}

	/**
	 * Move cheapest open cell to the closed set and return it.
	 * Effectively, this, combined with the "add" functionality is the verb: Expansion
	 * @return the cell formerly the cheapest from the open set
	 */
	public GridCell closeCheapestOpen()
	{
		DasCellInfo cellInfo = openQueue.poll();

		if (cellInfo == null) {
			throw new IllegalStateException(
					"Open set is empty - cannot close cheapest open cell");
		}

		cellInfo.setCellMembership(CellSetMembership.CLOSED);
		nClosedCount++;

		return cellInfo.getCell();
	}

	/**
	 * Move cell from closed list to pruned list. A cell should always be removed
	 * from the open list before being pruned.
	 * @param cell the cell to be pruned.
	 */
	public void pruneCell(GridCell cell) {
		DasCellInfo cellInfo = safeGetCellInfo(cell);

		if (cellInfo.getCellMembership() != CellSetMembership.CLOSED) {
			throw new IllegalStateException("Cannot prune cell " + cell +
					" - not in closed set.");
		}

		cellInfo.setCellMembership(CellSetMembership.PRUNED);
		prunedQueue.offer(cellInfo);
		nClosedCount--;
	}

	public boolean isPrunedEmpty()
	{
		return(prunedQueue.isEmpty());
	}

	/**
	 * Return a selection of pruned states to the open set. The number of states
	 * moved is the number that is estimated can be explored with the given
	 * number of expansions.
	 * @param expansions the number of expansions to limit the number of nodes
	 *                   recovered
	 */
	public void recoverPrunedStates(int expansionCount) {
		// the sum of d^cheapest for each cell reopened
		int dSum = 0;
		int count = 0;

		while (dSum < expansionCount && prunedQueue.size() > 0) {
			
			DasCellInfo cellInfo = prunedQueue.poll();

			dSum += cellInfo.getDCheapestWithError();

			// add to open set
			cellInfo.setCellMembership(CellSetMembership.OPEN);
			openQueue.offer(cellInfo);
			count++;
		}
		
		System.out.println("Recovering " + count + " nodes");
	}

	public GridCell getParent(GridCell cell) {
		DasCellInfo cellInfo = safeGetCellInfo(cell);
		return cellInfo.getParent().getCell();
	}

	public void setParent(GridCell cell, GridCell parent) {
		DasCellInfo cellInfo = safeGetCellInfo(cell);
		DasCellInfo parentInfo = safeGetCellInfo(parent);

		cellInfo.setParent(parentInfo);
	}

	public float getFCost(GridCell cell) {
		DasCellInfo cellInfo = safeGetCellInfo(cell);
		return cellInfo.getFCost();
	}

	public float getGCost(GridCell cell) {
		DasCellInfo cellInfo = safeGetCellInfo(cell);
		return cellInfo.getGCost();
	}

	public void setGCost(GridCell cell, float gCost) {
		DasCellInfo cellInfo = safeGetCellInfo(cell);

		// TODO: should we allow this? I think it might be necessary
		if (cellInfo.getCellMembership() == CellSetMembership.OPEN) {
			throw new IllegalArgumentException(
					"Cell's g cost cannot be altered whilst in the open set.");
		}

		cellInfo.setGCost(gCost);
	}

	public float getHCost(GridCell cell) {
		DasCellInfo cellInfo = safeGetCellInfo(cell);
		return cellInfo.getHCost();
	}

	public void setHCost(GridCell cell, float hCost) {
		DasCellInfo cellInfo = safeGetCellInfo(cell);

		// TODO: should we allow this? I think it might be necessary
		if (cellInfo.getCellMembership() == CellSetMembership.OPEN) {
			throw new IllegalArgumentException(
					"Cell's g cost cannot be altered whilst in the open set.");
		}

		cellInfo.setHCost(hCost);
	}

	public void setCosts(GridCell cell, float gCost, float hCost) {
		DasCellInfo cellInfo = safeGetCellInfo(cell);

		// TODO: should we allow this? I think it might be necessary
		if (cellInfo.getCellMembership() == CellSetMembership.OPEN) {
			throw new IllegalArgumentException(
					"Cell's g cost cannot be altered whilst in the open set.");
		}

		cellInfo.setGCost(gCost);
		cellInfo.setHCost(hCost);
	}

	public float getDCheapestWithError(GridCell cell) {
		DasCellInfo cellInfo = safeGetCellInfo(cell);

		return cellInfo.getDCheapestWithError();
	}

	public int getExpansionNumber(GridCell cell) {
		DasCellInfo cellInfo = safeGetCellInfo(cell);

		return cellInfo.getExpansionNumber();
	}

	/**
	 * Get set that this cell is currently in.
	 * @param cell the cell to check
	 * @return the set that the cell is in, or null if it has not been added.
	 */
	public CellSetMembership getSetMembership(GridCell cell) {
		DasCellInfo cellInfo = getCellInfo(cell);
		return (cellInfo == null) ? null : cellInfo.getCellMembership();
	}

	/**
	 * Check if cell is in open set.
	 * @param cell the cell to check
	 * @return true if cell is in open set, otherwise false
	 */
	public boolean isOpen(GridCell cell) {
		DasCellInfo cellInfo = getCellInfo(cell);
		return cellInfo != null && cellInfo.getCellMembership() == CellSetMembership.OPEN;
	}

	/**
	 * Check if cell is in closed set.
	 * @param cell the cell to check
	 * @return true if cell is in closed set, otherwise false
	 */
	public boolean isClosed(GridCell cell) {
		DasCellInfo cellInfo = getCellInfo(cell);
		return cellInfo != null && cellInfo.getCellMembership() == CellSetMembership.CLOSED;
	}

	/**
	 * Check if cell is in pruned set.
	 * @param cell the cell to check
	 * @return true if cell is in pruned set, otherwise false
	 */
	public boolean isPruned(GridCell cell) {
		DasCellInfo cellInfo = getCellInfo(cell);
		return cellInfo != null && cellInfo.getCellMembership() == CellSetMembership.PRUNED;
	}

	/** Return an ArrayList of all the GridCells currently in the closed set. */
	public ArrayList<GridCell> getClosedArrayList() {
		ArrayList<GridCell> closed = new ArrayList<GridCell>(nClosedCount);
		for (int x = 0; x < width; x++) {
			for (int y = 0; y < height; y++) {
				DasCellInfo cellInfo = cells[x][y];
				if (cellInfo != null && cellInfo.getCellMembership() == CellSetMembership.CLOSED) {
					closed.add(cellInfo.getCell());
				}
			}
		}
		return closed;
	}

	/** Return an ArrayList of all GridCells currently in the open set. */
	public ArrayList<GridCell> getOpenArrayList() {
		ArrayList<GridCell> open = new ArrayList<GridCell>(openQueue.size());
		for (DasCellInfo cellInfo : openQueue) {
			open.add(cellInfo.getCell());
		}
		return open;
	}
	
	/** Return an ArrayList of all GridCells currently in the pruned set. */
	public ArrayList<GridCell> getPrunedArrayList() {
		ArrayList<GridCell> pruned = new ArrayList<GridCell>(prunedQueue.size());
		for (DasCellInfo cellInfo : prunedQueue) {
			pruned.add(cellInfo.getCell());
		}
		return pruned;
	}	

	private DasCellInfo safeGetCellInfo(GridCell cell) {
		DasCellInfo cellInfo = getCellInfo(cell);
		if (cellInfo == null) {
			throw new IllegalStateException("Cell " + cell + " has no been added.");
		}
		return cells[cell.getCoord().getX()][cell.getCoord().getY()];
	}

	/* get cell info associated with cell. */
	private DasCellInfo getCellInfo(GridCell cell) {
		return cells[cell.getCoord().getX()][cell.getCoord().getY()];
	}
}
