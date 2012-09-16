package agents;

import java.util.PriorityQueue;
import java.util.ArrayList;
import java.util.Comparator;
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
public class RwaMapInfo implements Comparator<GridCell> {

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

	// Heuristic weight.
	private float                 hWeight;

	private final float EPSILON = 0.001f; // used for floating point comparisons
	private final int INITIAL_QUEUE_CAPACITY = 11;

	/**
	 * Construct a new RwaMapInfo.
	 * @param map the map to store information about
	 * @param hWeight the initial heuristic weight
	 */
	public RwaMapInfo(GridDomain map, float hWeight) {
		this.map = map;
		int width = map.getWidth();
		int height = map.getHeight();

		// Initialize arrays.
		this.sets    = new CellSetMembership[width][height];
		this.parents = new GridCell[width][height];
		this.gCosts  = new float[width][height];
		this.hCosts  = new float[width][height];

		// Set heuristic weight.
		this.hWeight = hWeight;

		// Initialize queues for open sets.
		this.openQueue = new PriorityQueue<GridCell>(INITIAL_QUEUE_CAPACITY, this);
	}

	public ComputedPlan computePlan(GridCell goal)
	{

		Trace.print("Generating new incumbent plan...");

		ComputedPlan plan = new ComputedPlan();

		GridCell cell = goal;
		while (cell != null) {
			plan.prependStep(cell);
			cell = getParent(cell);
		}

		plan.setCost(getGCost(goal));
		return plan;
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

		// Set cell properties
		gCosts[x][y] = gCost;
		hCosts[x][y] = hCost;
		parents[x][y] = parent;

		// Add to open set.
		sets[x][y] = CellSetMembership.OPEN;
		openQueue.offer(cell);
	}

	public void reopenCell(GridCell cell) {

		if (getSetMembership(cell) == CellSetMembership.OPEN) {
			throw new IllegalArgumentException("Cell already in open set.");
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
		return (openQueue.size() == 0);
	}

	/**
	 * Returns the number of cells in the open set.
	 * @return the number of cells in the open set
	 */
	public int openCount() {
		return openQueue.size();
	}

	/**
	 * Returns the number of cells in the closed set.
	 * @return the number of cells in the closed set
	 */
	public int closedCount() {
		return closedCount;
	}

	/**
	 * Returns the current heuristic weight.
	 * @return the current heuristic weight.
	 */
	public float getHWeight() {
		return hWeight;
	}

	/**
	 * Sets the current heuristic weight. The open set should be empty before
	 * this is executed.
	 */
	public void setHWeight(float hWeight) {
		if (isOpenEmpty() == false) {
			throw new IllegalStateException("Cannot change heuristic weight while " +
					"there are cells in the open set.");
		}

		this.hWeight = hWeight;
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

	/**
	 * Move all nodes from open and closed lists to "seen".
	 */
	public void moveAllToSeen() {
		// Set all cells in a set to seen.
		for (int y = 0; y < map.getHeight(); y++) {
			for (int x = 0; x < map.getWidth(); x++) {
				if (sets[x][y] != null) {
					sets[x][y] = CellSetMembership.SEEN;
				}
			}
		}

		// Clear open queue.
		openQueue.clear();
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
		return getGCost(cell) + hWeight * getHCost(cell);
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
			case SEEN:
			case CLOSED: {
				GridCoord gc = cell.getCoord();
				gCosts[gc.getX()][gc.getY()] = gCost;
				break;
			}
			case OPEN: {
				setQueuedGCost(cell, gCost, openQueue);
				break;
			}
			case NONE: {
				throw new IllegalArgumentException("Cell is not in a set.");
			}
			default: {
				assert false;
			}
		}
	}

	/**
	 * Get heuristic cost estimate from this cell to the goal.
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
	 * Check if cell has been generated.
	 * @param cell the cell to check
	 * @return true if the cell belongs to any set, otherwise false
	 */
	public boolean cellExists(GridCell cell) {
		GridCoord gc = cell.getCoord();
		return (sets[gc.getX()][gc.getY()] != null);
	}

	/**
	 * Check if cell is in the open set.
	 * @param cell the cell to check
	 * @return true if cell is in open set, otherwise false
	 */
	public boolean isOpen(GridCell cell) {
		return getSetMembership(cell) == CellSetMembership.OPEN;
	}

	/**
	 * Check if cell is in the closed set.
	 * @param cell the cell to check
	 * @return true if cell is in closed set, otherwise false
	 */
	public boolean isClosed(GridCell cell) {
		return getSetMembership(cell) == CellSetMembership.CLOSED;
	}

	/**
	 * Check if cell is in the seen set.
	 * @param cell the cell to check
	 * @return true if cell is in seen set, otherwise false
	 */
	public boolean isSeen(GridCell cell) {
		return getSetMembership(cell) == CellSetMembership.SEEN;
	}

	/* -- GRID CELL COMPARATOR -- */

	/**
	 * Perform an approximate comparison of two floating point values.
	 * @param a value 1
	 * @param b value 2
	 * @return 0, -1 or 1 if a is equal to, less than or greater than b respectively
	 */
	private int compareFloat(float a, float b) {
		if (Math.abs(a - b) < EPSILON) {
			return 0;
		}
		return (a > b) ? 1 : -1;
	}

	/**
	 * Perform an approximate comparison of two floating point values. Compares
	 * cells on their f cost, breaking ties on h.
	 * @param a grid cell 1
	 * @param b grid cell 2
	 * @return 0, -1 or 1 if a is equal to, less than or greater than b respectively
	 */
	public int compare(GridCell a, GridCell b) {

		// Compare total cost estimate.
		int fCompare = compareFloat(getFCost(a), getFCost(b));
		if (fCompare != 0) {
			return fCompare;
		}

		// Break ties on heuristic estimate.
		return compareFloat(getHCost(a), getHCost(b));
	}

	public boolean equals() { return false; }

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
