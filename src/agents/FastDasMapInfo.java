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
 * We are using a mapsized array for each attribute, so that we can allocate 
 * all of the storage at construction time, rather than dynamically
 * It is felt that this could be a performance restriction.
 * The old implementation, storing everything in Node and pushing/popping
 * these nodes onto a priority queue, is in the DasCellInfo class, but 
 * has not been maintained for a while.
 * 
 * It is probably a more readable/maintainable implementation, but the suspicion is that
 * the dynamic allocation of nodes causes performance issues. 
 * 
 * Note that each attribute given to a node will have to have an array allocated for 
 * to cover each node.
 **/
public class FastDasMapInfo 
{
	GridDomain map;

	private int closedCount = 0;

	// Priority queues for open and pruned sets.
	private PriorityQueue<GridCell> openQueue;

	private WeightedHFComparator weightedHComp;
	private PriorityQueue<GridCell> prunedQueue;

	
	// Cell properties.
	private CellSetMembership[][] sets; 					// Set membership of each node
	private GridCell[][]          parents;					// Parents of each node
	private int[][]               gCosts;					// G Costs of each node
	private int[][]               hCosts;					// H Estimate of each node
	private int[][]               dCheapestRaws;			// Initial D Cheapest value of each node
	private float[][]             dCheapestWithErrors;		// Error corrected d^cheapest of each node
	private int[][]               dErrors;					// Error present (vs heuristic) of each node
	private int[][]               expansionNumbers;		// Expansion number stamped on each node
	private int[][]               cumulativeErrors;		// Error experienced so far, to this node, from the start point
	private int[][]               depths;					// Depth of each node, from the start point

	// Used to initialise the priority queues - arbitrary value
	// Does not seem to have an impact on performance.
	private final int INITIAL_QUEUE_CAPACITY = 1000;

	
	/**
	 * Constructor for the map info class
	 * @param map
	 */
	public FastDasMapInfo(GridDomain map) {
		this.map = map;
		int width = map.getWidth();
		int height = map.getHeight();

		this.sets                 = new CellSetMembership[width][height];
		this.gCosts               = new int[width][height];
		this.hCosts               = new int[width][height];
		this.parents              = new GridCell[width][height];
		this.dCheapestRaws        = new int[width][height];
		this.dCheapestWithErrors  = new float[width][height];
		this.dErrors              = new int[width][height];
		this.expansionNumbers     = new int[width][height];
		this.cumulativeErrors     = new int[width][height];
		this.depths               = new int[width][height];

		// Initialize queues for open and pruned sets.
		this.openQueue = new PriorityQueue<GridCell>(INITIAL_QUEUE_CAPACITY,
				new FComparator(this));
		
		weightedHComp = new WeightedHFComparator(this, 1000f); // W=1000, obliterate G initially
		this.prunedQueue = new PriorityQueue<GridCell>(INITIAL_QUEUE_CAPACITY, weightedHComp);
	}

	public ComputedPlan computePlan(GridCell goal)
	{

		//Trace.print("Generating new incumbent plan...");

		ComputedPlan plan = new ComputedPlan();

		GridCell cell = goal;
		while (cell != null) {
			plan.prependStep(cell);
			cell = getParent(cell);
		}

		plan.setCost(getGCost(goal));
		return plan;
	}

	public void addStartCell(GridCell cell, int hCost, int dCheapestRaw) {
		// Start cell has zero gCost, and no parent.
		add(cell, 0, hCost, dCheapestRaw, 0, null);
	}

	/**
	 * Add cell to open set. This will fail if cell has already been added.
	 * @param cell            the cell
	 * @param gCost           the cost to get to the cell
	 * @param hCost           the heuristic estimate to get to the goal
	 * @param dCheapestRaw    the estimated goaldepth from the cell
	 * @param expansionNumber the number of expansions performed before this cell
	 *                        was generated
	 * @param parent          the previous cell in a path
	 */
	public void add(GridCell cell, int gCost, int hCost, int dCheapestRaw,
			int expansionNumber, GridCell parent)
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
		dCheapestRaws[x][y] = dCheapestRaw;

		// Update other cell properties
		setPathToCell(cell, gCost, expansionNumber, parent);

		// Add to open set.
		sets[x][y] = CellSetMembership.OPEN;
		openQueue.offer(cell);

	}

	public void setPathToCell(GridCell cell, int gCost, int expansionNumber,
			GridCell parent) {

		int x = cell.getCoord().getX();
		int y = cell.getCoord().getY();

		// Set cell properties
		setGCost(cell, gCost);
		expansionNumbers[x][y] = expansionNumber;

		// Reuse old dCheapestRaw value
		int dCheapestRaw = getDCheapestRaw(cell);

		// Calculate depth data and error based on parent
		parents[x][y] = parent;
		if (parent != null) {
			depths[x][y] = getDepth(parent) + 1;
			int dError =  dCheapestRaw - getDCheapestRaw(parent) + 1;
			dErrors[x][y] = dError;
			cumulativeErrors[x][y] = getCumulativeError(parent) + dError;
			dCheapestWithErrors[x][y] = calculateDCheapestWithError(cell);
		} else {
			dCheapestWithErrors[x][y] = dCheapestRaw;
		}
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
		return closedCount ;
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
	 * Move cell from closed list to pruned list. A cell should always be removed
	 * from the open list before being pruned.
	 * @param cell the cell to be pruned.
	 */
	public void pruneCell(GridCell cell) {
		if (getSetMembership(cell) != CellSetMembership.CLOSED) {
			throw new IllegalStateException("Cannot prune cell " + cell +
					" - not in closed set.");
		}
		// GS: note that timePercentRemaining is no longer used!
		// Remove from open set.
		closedCount--;

		// Set set attribute to pruned.
		GridCoord gc = cell.getCoord();
		sets[gc.getX()][gc.getY()] = CellSetMembership.PRUNED;

		prunedQueue.offer(cell);

	}
	
	void NotifySolutionFound()
	{
		weightedHComp.setWeight(1.0f);
		int szPrunedQueue = prunedQueue.size();
		GridCell conCells[] = new GridCell[szPrunedQueue];
//		System.out.println("After Solution Found: Size of pruned " + szPrunedQueue);
		for (int n = 0; n < szPrunedQueue; n++)
		{
			
			conCells[n] = prunedQueue.poll();
		}
		
		for (int n = 0; n < szPrunedQueue; n++)
		{
			//System.out.println("Readding " + iterCells);
			prunedQueue.offer(conCells[n]);
		}
	}

	public boolean isPrunedEmpty()
	{
		return prunedQueue.isEmpty();
	}

	/**
	 * Return a selection of pruned states to the open set. The number of states
	 * moved is the number that is estimated can be explored with the given
	 * number of expansions.
	 * @param expansions the number of expansions to limit the number of nodes
	 *                   recovered
	 */
	public void recoverPrunedStates(int expansionCount) {
		int expansionsRemaining = expansionCount;

		int count = 0;
		while (expansionsRemaining > 0 && prunedQueue.size() > 0)
		{
			GridCell cell = prunedQueue.poll();
			expansionsRemaining -= getDCheapestWithError(cell);

			// Set set attribute to opened.
			GridCoord gc = cell.getCoord();
			sets[gc.getX()][gc.getY()] = CellSetMembership.OPEN;

			// Add to opened priority queue.
			openQueue.offer(cell);
		}
	}
	

	public GridCell getParent(GridCell cell) {
		GridCoord gc = cell.getCoord();
		return parents[gc.getX()][gc.getY()];
	}

	public void setParent(GridCell cell, GridCell parent) {
		GridCoord gc = cell.getCoord();
		parents[gc.getX()][gc.getY()] = parent;
	}

	public int getFCost(GridCell cell) {
		return getGCost(cell) + getHCost(cell);
	}

	public int getGCost(GridCell cell) {
		GridCoord gc = cell.getCoord();
		return gCosts[gc.getX()][gc.getY()];
	}

	private void setQueuedGCost(GridCell cell, int gCost, PriorityQueue<GridCell> queue) {
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

	private void setGCost(GridCell cell, int gCost) {
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
			case PRUNED: {
				setQueuedGCost(cell, gCost, prunedQueue);
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
	public int getHCost(GridCell cell) {
		GridCoord gc = cell.getCoord();
		return hCosts[gc.getX()][gc.getY()];
	}

	private int getDepth(GridCell cell) {
		GridCoord gc = cell.getCoord();
		return depths[gc.getX()][gc.getY()];
	}

	private int getDCheapestRaw(GridCell cell) {
		GridCoord gc = cell.getCoord();
		return dCheapestRaws[gc.getX()][gc.getY()];
	}

	public int getCumulativeError(GridCell cell) {
		GridCoord gc = cell.getCoord();
		return cumulativeErrors[gc.getX()][gc.getY()];
	}

	public float calculateDCheapestWithError(GridCell cell)
	{
		float avgError = getAverageError(cell);
		float result;

		if (FloatUtil.compare(avgError, 1.0f) == -1) {
			float dCheapest = (float) getDCheapestRaw(cell);
			result = dCheapest / (1.0f - avgError);
		} else {
			result = Float.POSITIVE_INFINITY;
		}
		//System.out.println("avgError: " + avgError + " result: " + result);
		//printCell(cell);
		return result;
	}

	public float getDCheapestWithError(GridCell cell) {

		GridCoord gc = cell.getCoord();
		float dCheapestWithError = dCheapestWithErrors[gc.getX()][gc.getY()];
		return(dCheapestWithError);
	}

	/**
	 * Get average single step error.
	 */
	private float getAverageError(GridCell cell) {
		if (getDepth(cell) == 0) {
			return 0;
		}
		return (float)(getCumulativeError(cell) / getDepth(cell));
	}

	public int getExpansionNumber(GridCell cell) {
		GridCoord gc = cell.getCoord();
		return expansionNumbers[gc.getX()][gc.getY()];
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

	/**
	 * Check if cell is in pruned set.
	 * @param cell the cell to check
	 * @return true if cell is in pruned set, otherwise false
	 */
	public boolean isPruned(GridCell cell) {
		return getSetMembership(cell) == CellSetMembership.PRUNED;
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
		int dRaw = dCheapestRaws[x][y];
		float dCheapestWithError = dCheapestWithErrors[x][y];
		int dError = dErrors[x][y];
		int expansionNumber = expansionNumbers[x][y];
		int cumulativeError = cumulativeErrors[x][y];
		int depth = depths[x][y];
		System.out.println("\nPrinting data for cell " + cell);

		System.out.println("Set Membership: " + mem);
		System.out.println("Parent: " + parent);
		System.out.println("g: " + g);
		System.out.println("h: " + h);
		System.out.println("f: " + f);
		System.out.println("dCheapestRaw: " + dRaw);
		System.out.println("dCheapestWithError: " + dCheapestWithError);
		System.out.println("dError: " + dError);
		System.out.println("ExpansionNumber: " + expansionNumber);
		System.out.println("CumulativeError: " + cumulativeError);
		System.out.println("Depth: " + depth );
		System.out.println("\n************************");
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

	/** Return an ArrayList of all GridCells currently in the pruned set. */
	public ArrayList<GridCell> getPrunedArrayList() {
		return new ArrayList<GridCell>(prunedQueue);
	}
}
