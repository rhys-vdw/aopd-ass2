package agents;

import java.util.PriorityQueue;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.ListIterator;
import java.util.Random;
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
 * I have used assertions instead of exceptions for speed (since we can disable
 * them on run).
 */
public class PreferredMapInfo implements Comparator<GridCell> {

	// Used in the case of tiebreakers after f and h.
	static Random rand = new Random(System.nanoTime());
	// Added this data for debugging - probably not useful in our actual algorithm,
	// is just to aid analysis.
	GridDomain map;

	private int closedCount = 0;

	// Priority queues for open and pruned sets.
	private PriorityQueue<GridCell> openQueue;

	// Cell properties.
	private CellSetMembership[][] sets;
	private GridCell[][]          parents;
	private float[][]             gCosts;
	private float[][]             hCosts;

	private final int INITIAL_QUEUE_CAPACITY = 11;

	public PreferredMapInfo(GridDomain map) {
		this.map = map;
		int width = map.getWidth();
		int height = map.getHeight();

		this.sets                 = new CellSetMembership[width][height];
		this.gCosts               = new float[width][height];
		this.hCosts               = new float[width][height];
		this.parents              = new GridCell[width][height];

		// Initialize queues for open and pruned sets.
		this.openQueue = new PriorityQueue<GridCell>(INITIAL_QUEUE_CAPACITY, this);
	}

	public ComputedPlan computePlan(GridCell goal)
	{
		ComputedPlan plan = new ComputedPlan();

		GridCell cell = goal;
		while (cell != null) {
			plan.prependStep(cell);
			cell = getParent(cell);
		}

		plan.setCost(getGCost(goal));
		return plan;
	}

	public void addStartCell(GridCell cell, float hCost) {
		// Start cell has zero gCost, and no parent.
		add(cell, 0f, hCost, null);
	}

	/**
	 * Add cell to open set. This will fail if cell has already been added.
	 * @param cell            the cell
	 * @param gCost           the cost to get to the cell
	 * @param hCost           the heuristic estimate to get to the goal
	 * @param parent          the previous cell in a path
	 */
	public void add(GridCell cell, float gCost, float hCost, GridCell parent)
	{
		// Should only be called when no info exists for node.
		CellSetMembership prevSet = getSetMembership(cell);
		if (prevSet != CellSetMembership.NONE) {
			throw new IllegalArgumentException("Cannot add cell " + cell +
					", which has already been added as to the " + prevSet + ".");
		}

		int x = cell.getCoord().getX();
		int y = cell.getCoord().getY();

		// Set cell properties that will not change
		hCosts[x][y] = hCost;
		gCosts[x][y] = gCost;
		parents[x][y] = parent;

		// Add to open set.
		sets[x][y] = CellSetMembership.OPEN;
		openQueue.offer(cell);
	}

	/**
	 * Move cell that has already been added to open list. Never call this on an
	 * open cell.
	 */
	public void reopenCell(GridCell cell) {
		// Should only be called when no info exists for node.
		CellSetMembership prevSet = getSetMembership(cell);
		if (prevSet == CellSetMembership.OPEN) {
			throw new IllegalArgumentException("Cannot open cell " + cell +
					", already in the open set!");
		}

		int x = cell.getCoord().getX();
		int y = cell.getCoord().getY();

		// Add to open set
		sets[x][y] = CellSetMembership.OPEN;
		openQueue.offer(cell);
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
	public int closedCount() {
		return closedCount;
	}

	/**
	 * Move cheapest open cell to the closed set and return it.
	 * @return the cell formerly the cheapest from the open set
	 */
	public GridCell closeCheapestOpen()
	{
		GridCell cell = openQueue.poll();

		if (cell == null) {
			throw new IllegalStateException(
					"Open set is empty - cannot close cheapest open cell");
		}

		GridCoord gc = cell.getCoord();
		sets[gc.getX()][gc.getY()] = CellSetMembership.CLOSED;
		closedCount++;

		return cell;
	}

	public GridCell getParent(GridCell cell) {
		GridCoord gc = cell.getCoord();
		return parents[gc.getX()][gc.getY()];
	}

	public void setParent(GridCell cell, GridCell parent) {
		GridCoord gc = cell.getCoord();
		parents[gc.getX()][gc.getY()] = parent;
	}

	public float getFCost(GridCell cell) {
		return getGCost(cell) + 5.0f * getHCost(cell);
	}

	public float getGCost(GridCell cell) {
		GridCoord gc = cell.getCoord();
		return gCosts[gc.getX()][gc.getY()];
	}

	private void setQueuedGCost(GridCell cell, float gCost, PriorityQueue<GridCell> queue) {
		// Remove node from priority queue. (Changing its g cost in place would
		// cause the heaps to become unsorted.)
		boolean wasPresent = queue.remove(cell);

		// Ensure that it was indeed in the open set.
		if (wasPresent == false) {
			throw new IllegalArgumentException("Cell was not found priority queue!");
		}

		// Update g cost.
		GridCoord gc = cell.getCoord();
		gCosts[gc.getX()][gc.getY()] = gCost;

		// Reinsert node sorted.
		queue.offer(cell);
	}

	public void setGCost(GridCell cell, float gCost) {
		switch (getSetMembership(cell)) {
			case NONE:
			case CLOSED: {
				GridCoord gc = cell.getCoord();
				gCosts[gc.getX()][gc.getY()] = gCost;
				break;
			}
			case OPEN: {
				setQueuedGCost(cell, gCost, openQueue);
				break;
			}
			default: {
				assert false;
			}
		}
	}

	/**
	 * Get heuristic cost estimate from this cell to the goal.
	 * NOTE: We don't need write access to this for this assignment.
	 * @param cell the cell h is estimated from
	 */
	public float getHCost(GridCell cell) {
		GridCoord gc = cell.getCoord();
		return hCosts[gc.getX()][gc.getY()];
	}

	/**
	 * Get set that this cell is currently in.
	 * @param cell the cell to check
	 * @return the set that the cell is in, or null if it has not been added.
	 */
	public CellSetMembership getSetMembership(GridCell cell) {
		GridCoord gc = cell.getCoord();
		CellSetMembership set = sets[gc.getX()][gc.getY()];
		return set == null ? CellSetMembership.NONE : set;
	}

	/**
	 * Has this cell been added yet?
	 * @param cell the cell to check
	 * @return true if the cell belongs to any set, otherwise false
	 */
	public boolean cellExists(GridCell cell) {
		GridCoord gc = cell.getCoord();
		return (sets[gc.getX()][gc.getY()] != null);
	}

	/**
	 * Check if cell is in open set.
	 * @param cell the cell to check
	 * @return true if cell is in open set, otherwise false
	 */
	public boolean isOpen(GridCell cell) {
		return getSetMembership(cell) == CellSetMembership.OPEN;
	}

	/**
	 * Check if cell is in closed set.
	 * @param cell the cell to check
	 * @return true if cell is in closed set, otherwise false
	 */
	public boolean isClosed(GridCell cell) {
		return getSetMembership(cell) == CellSetMembership.CLOSED;
	}

	/* -- GRID CELL COMPARATOR -- */

	/**
	 * Compare two grid cells. Compares
	 * cells on their f cost, breaking ties on h.
	 * @param a grid cell 1
	 * @param b grid cell 2
	 * @return 0, -1 or 1 if a is equal to, less than or greater than b respectively
	 */
	public int compare(GridCell a, GridCell b) {
		// Compare total cost estimate.
		int fCompare = FloatUtil.compare(getFCost(a), getFCost(b));
		if (fCompare != 0) {
			return fCompare;
		}

		// Break ties on heuristic estimate.
		return FloatUtil.compare(getHCost(a), getHCost(b));
	}

	void printCell(GridCell cell)
	{
		int x = cell.getCoord().getX();
		int y = cell.getCoord().getY();
		CellSetMembership mem = sets[x][y];
		GridCell parent = parents[x][y];
		float g =  gCosts[x][y];
		float h =  hCosts[x][y];
		float f = getFCost(cell);
		System.out.println("\nPrinting data for cell " + cell);

		System.out.println("Set Membership: " + mem);
		System.out.println("Parent: " + parent);
		System.out.println("g: " + g);
		System.out.println("h: " + h);
		System.out.println("f: " + f);
		System.out.println("\n************************");
	}

	/* -- DEBUG -- */

	/** Return an ArrayList of all the GridCells currently in the closed set. */
	public ArrayList<GridCell> getClosedArrayList() {
		ArrayList<GridCell> closed = new ArrayList<GridCell>(closedCount);
		for (int x = 0; x < map.getWidth(); x++) {
			for (int y = 0; y < map.getHeight(); y++) {
				if (sets[x][y] == CellSetMembership.CLOSED) {
					closed.add(map.getCell(x, y));
				}
			}
		}
		return closed;
	}

	/** Return an ArrayList of all GridCells currently in the open set. */
	public ArrayList<GridCell> getOpenArrayList() {
		return new ArrayList<GridCell>(openQueue);
	}
}
