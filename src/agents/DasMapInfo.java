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

	private ComputedPlan planIncumbent = null;

	// Track the number of expansions performed -  e_curr value
	// TODO: investigate refactoring this to long to avoid potential truncactions in operations
	private int nExpansionsCount = 0;

	// These values needs tuning!
	// This is the size of the sliding window, in entries.
	final private int EXPANSION_DELAY_WINDOW_LENGTH = 15;

	// r_default. Used before conExpansionIntervals has settled.
	// This is the number of expansions to perform before the sliding window is deemed 'settled'
	final private int SETTLING_EXPANSION_COUNT = 200;

	// Time in ns to use as the expected interval between expansions, before settling.
	// Shouldn't be used with the new refactoring.
	final private long SETTLING_EXPANSION_AVG_INTERVAL = 30;


	// Time of the most recent expansion
	private long timeMostRecentExpansion = 0;

	// Delta e value
	private SlidingWindow conExpansionDelays = new SlidingWindow(EXPANSION_DELAY_WINDOW_LENGTH,
			SETTLING_EXPANSION_COUNT);

	// r value
	private SlidingWindow conExpansionIntervals = new SlidingWindow(EXPANSION_DELAY_WINDOW_LENGTH,
			SETTLING_EXPANSION_COUNT);

	public DasMapInfo(GridDomain map) {
		this.width = map.getWidth();
		this.height = map.getHeight();
		this.cells = new DasCellInfo[width][height];
		this.openQueue = new PriorityQueue<DasCellInfo>();
		this.prunedQueue = new PriorityQueue<DasCellInfo>();

	}

	public void computePlan(GridCell goal)
	{
		ComputedPlan planNewIncumbent = new ComputedPlan();

		Trace.print("Generating new incumbent plan...");

		for (DasCellInfo cell = getCellInfo(goal);
		     cell.getParent() != null;
		     cell = cell.getParent())
		{
			GridCell gc = cell.getCell();
			//System.out.println("Prepending " + gc);
			planNewIncumbent.prependStep(gc);
		}

		Trace.print("...Done.");

		planNewIncumbent.setCost(getGCost(goal));
		planIncumbent = planNewIncumbent;
	}

	public ComputedPlan GetIncumbentPlan()
	{
		return(planIncumbent);
	}

	/**
	 * Add cell to open set. This will fail if cell has already been added.
	 * @param cell the cell
	 * @param gCost the cost to get to the cell
	 * @param hCost the heuristic estimate to get to the goal
	 * @param parent the previous cell in a path
	 */
	public void add(GridCell cell, float gCost, float hCost, int dCheapestRaw, int error)
	{
		// should only be called when no cell already exists in array
		if (getCellInfo(cell) != null) {
			throw new IllegalArgumentException("Cannot add cell " + cell +
					", which has already been added");
		}

		// create new cell info
		DasCellInfo cellInfo = new DasCellInfo(cell, gCost, hCost,
				CellSetMembership.OPEN, nExpansionsCount, dCheapestRaw);

		// add new node to array and open queue
		cells[cell.getCoord().getX()][cell.getCoord().getY()] = cellInfo;
		openQueue.offer(cellInfo);
	}

	/**
	 * Add cell to open set. This will fail if cell has already been added
	 * @param cell the cell
	 * @param gCost the cost to get to the cell
	 * @param hCost the heuristic estimate to get to the goal
	 */
	public void add(GridCell cell, float gCost, float hCost, int dCheapestRaw,
			GridCell parent)
	{
		// TODO: need to add some traces to check the intuition here.
		// i.e. show that error goes up as the direction is away from the goal.
		int dError = dCheapestRaw - getCellInfo(parent).getDCheapestRaw() + 1;

		add(cell, gCost, hCost, dCheapestRaw, dError);
		setParent(cell, parent);
	}

	public boolean getSettled()
	{
		// TODO: need to change sliding window class so that it has both windows as one.
		boolean isSettled = (conExpansionDelays.getSettled() && conExpansionIntervals.getSettled());
		return(isSettled);
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

		// We will have better closed list management, but for now, just tally the
		// number of times we close an entry - this will help with analysis.
		++nClosedCount;

		// TODO: Not sure if it should be incremented before or after!
		// Incremented e_curr value
		++nExpansionsCount;


		// Record the number of expansions performed before processing the node
		// after each expansion.
		// i.e. we are recording the current level of vacillation.
		long nCurrentExpansionDelay = this.nExpansionsCount - cellInfo.getExpansionNumber();

		long timeCurrent = System.nanoTime();

		long timeExpansionsDelta = timeCurrent - this.timeMostRecentExpansion;

		// Store this time, for the next time we perform an expansion.
		this.timeMostRecentExpansion = timeCurrent;
		// Add the expansion delay to the circular queue, so that
		// we can compute a rolling average.
		conExpansionDelays.Push(nCurrentExpansionDelay);
			// Don't perform certain functionality for the start node.
		if (cellInfo.getParent() != null)
		{

			conExpansionIntervals.Push(timeExpansionsDelta);
		}

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

		while (dSum < expansionCount && prunedQueue.size() > 0) {
			DasCellInfo cellInfo = prunedQueue.poll();

			dSum += cellInfo.getDCheapestWithError();

			// add to open set
			cellInfo.setCellMembership(CellSetMembership.OPEN);
			openQueue.offer(cellInfo);
		}

		Trace.print("Resetting windows");
		conExpansionIntervals.reset();
		conExpansionDelays.reset();
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

	public float calculateAvgExpansionInterval()
	{
		return conExpansionIntervals.getAvg();
	}

	public double calculateAvgExpansionDelay()
	{
		return conExpansionDelays.getAvg();
	}

	public float getDCheapestWithError(GridCell cell)
	{
		return getCellInfo(cell).getDCheapestWithError();
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
