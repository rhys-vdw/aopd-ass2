package agents;

import java.util.PriorityQueue;
// TODO:
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.ListIterator;

import au.rmit.ract.planning.pathplanning.entity.ComputedPlan;

import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridDomain;
import pplanning.simviewer.model.GridCoord;

/**
 * A class to store metadata about the current map.
 *
 * Direct access to CellInfo instances cannot be provided to ensure that a cell
 * is always present in the open queue when it is in the open set.
 *
 * Nodes that are in the open set cannot be altered. Later this will probably
 * need to be changed, but for standard A* this is fine.
 *
 * To fix this a linked list will be used instead of a priority queue. Whenever
 * the f cost of a node in the open list is changed, a flag will change to show
 * that the list is out of order. The list will be sorted when
 * closeCheapestOpenCell is called. Insertions to the open list will be in order.
 *
 * I have used assertions instead of exceptions for speed (since we can disable
 * them on run).
 */
public class DasMapInfo {
	private DasCellInfo[][] cells;
	private PriorityQueue<DasCellInfo> openQueue;
	
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
	}

	public ComputedPlan computePlan(GridCell goal) {
		ComputedPlan plan = new ComputedPlan();

		Trace.print("Generating plan...");

		for (DasCellInfo cell = getCellInfo(goal);
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

	/**
	 * Add cell to open set. This will fail if cell has already been added
	 * @param cell the cell
	 * @param gCost the cost to get to the cell
	 * @param hCost the heuristic estimate to get to the goal
	 * @param parent the previous cell in a path
	 */
	public void add(GridCell cell, float gCost, float hCost) {
		// should only be called when no cell already exists in array
		assert getCellInfo(cell) == null;

		// add new node to array and open queue
		DasCellInfo cellInfo = new DasCellInfo(cell, gCost, hCost, CellSetMembership.OPEN);
		cells[cell.getCoord().getX()][cell.getCoord().getY()] = cellInfo;
		openQueue.offer(cellInfo);
	}

	/**
	 * Add cell to open set. This will fail if cell has already been added
	 * @param cell the cell
	 * @param gCost the cost to get to the cell
	 * @param hCost the heuristic estimate to get to the goal
	 */
	public void add(GridCell cell, float gCost, float hCost, GridCell parent) {
		add(cell, gCost, hCost);
		setParent(cell, parent);
	}

	/**
	 * Move cell to open set.
	 * @param cell the cell to move
	 */
	/*
	public void setCellOpen(GridCell cell) {
		DasCellInfo cellInfo = getCellInfo(cell);
		// TODO: add to priority queue

		// must have cell to 
		assert (cellInfo != null);


	} */

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
		DasCellInfo cellInfo = openQueue.poll();

		assert(cellInfo != null);

		cellInfo.setCellMembership(CellSetMembership.CLOSED);
		
		// We will have better closed list management, but for now, just tally the
		// number of times we close an entry - this will help with analysis.
		++nClosedCount;

		return cellInfo.getCell();
	}

	/**
	 * Move cell from closed list to pruned list. A cell should always be removed
	 * from the open list before being pruned.
	 * @param cell the cell to be pruned.
	 */
	public void pruneCell(GridCell cell) {
		DasCellInfo cellInfo = getCellInfo(cell);

		assert cellInfo != null;
		assert cellInfo.getCellMembership() == CellSetMembership.CLOSED;

		cellInfo.setCellMembership(CellSetMembership.PRUNED);
	}

	public GridCell getParent(GridCell cell) {
		DasCellInfo cellInfo = getCellInfo(cell);
		assert cellInfo != null;
		return cellInfo.getParent().getCell();
	}

	public void setParent(GridCell cell, GridCell parent) {
		DasCellInfo cellInfo = getCellInfo(cell);
		DasCellInfo parentInfo = getCellInfo(parent);

		assert cellInfo != null;
		assert parentInfo != null;

		cellInfo.setParent(parentInfo);
	}

	public float getFCost(GridCell cell) {
		DasCellInfo cellInfo = getCellInfo(cell);
		assert cellInfo != null;
		return cellInfo.getFCost();
	}

	public float getGCost(GridCell cell) {
		DasCellInfo cellInfo = getCellInfo(cell);
		assert cellInfo != null;
		return cellInfo.getGCost();
	}

	public void setGCost(GridCell cell, float gCost) {
		DasCellInfo cellInfo = getCellInfo(cell);
		assert cellInfo != null;
		assert cellInfo.getCellMembership() != CellSetMembership.OPEN;
		cellInfo.setGCost(gCost);
	}

	public float getHCost(GridCell cell) {
		DasCellInfo cellInfo = getCellInfo(cell);
		assert cellInfo != null;
		return cellInfo.getHCost();
	}

	public void setHCost(GridCell cell, float hCost) {
		DasCellInfo cellInfo = getCellInfo(cell);

		assert cellInfo != null;
		assert cellInfo.getCellMembership() != CellSetMembership.OPEN;

		cellInfo.setHCost(hCost);
	}

	public void setCosts(GridCell cell, float gCost, float hCost) {
		DasCellInfo cellInfo = getCellInfo(cell);

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
		DasCellInfo cellInfo = getCellInfo(cell);
		return (cellInfo == null) ? false : cellInfo.getCellMembership() == CellSetMembership.OPEN;
	}

	/**
	 * Check if cell is in closed set.
	 * @param cell the cell to check
	 * @return true if cell is in closed set, otherwise false
	 */
	public boolean isClosed(GridCell cell) {
		DasCellInfo cellInfo = getCellInfo(cell);
		return (cellInfo == null) ? false : cellInfo.getCellMembership() == CellSetMembership.CLOSED;
	}

	/* get cell info associated with cell. */
	private DasCellInfo getCellInfo(GridCell cell) {
		GridCoord gc = cell.getCoord();
		if (cells == null) Trace.print("cells is null");
		return cells[cell.getCoord().getX()][cell.getCoord().getY()];
	}
	
	/*
	 * Return a container for visualising the Open and Closed Sets
	 * Very expensive function!
	 * @param Open and Closed Sets
	 * @return none
	 */
	public void getSearchSetsAsArrayList(ArrayList<GridCell> _conOpen, ArrayList<GridCell> _conClosed)
	{
		//ArrayList<DasCellInfo> conOpenSet = new ArrayList<CellInfo>();
		//ArrayList<DasCellInfo> conClosedSet = new ArrayList<CellInfo>();
		
		for (int x = 0; x < width; x++)
		{
			for (int y = 0; y < height; y++)
			{
				DasCellInfo currCell = cells[x][y];
				if (currCell != null)
				{
					if (currCell.getCellMembership() == CellSetMembership.OPEN)
					{
						_conOpen.add(currCell.getCell());
					}
					else if (currCell.getCellMembership() == CellSetMembership.CLOSED)
					{
						_conClosed.add(currCell.getCell());
					}
				}
			}
		}
	}
}
