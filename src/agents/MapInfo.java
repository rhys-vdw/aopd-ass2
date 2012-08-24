package agents;

import java.util.PriorityQueue;
// TODO:
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
public class MapInfo {
	private CellInfo[][] cells;
	private PriorityQueue<CellInfo> openQueue;

	public MapInfo(GridDomain map) {
		this.cells = new CellInfo[map.getWidth()][map.getHeight()];
		this.openQueue = new PriorityQueue<CellInfo>();
	}

	public ComputedPlan computePlan(GridCell goal) {
		ComputedPlan plan = new ComputedPlan();

		System.out.println("Generating plan...");

		for (CellInfo cell = getCellInfo(goal);
		     cell.getParent() != null;
		     cell = cell.getParent()) {
			GridCell gc = cell.getCell();
			System.out.println("Prepending " + gc);
			plan.prependStep(gc);
		}

		System.out.println("...Done.");

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
		CellInfo cellInfo = new CellInfo(cell, gCost, hCost, Set.OPEN);
		cells[cell.getCoord().getX()][cell.getCoord().getY()] = cellInfo;
		openQueue.add(cellInfo);
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
		CellInfo cellInfo = getCellInfo(cell);
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

	/**
	 * Move cheapest open cell to the closed set and return it.
	 * @return the cell formerly the cheapest from the open set
	 */
	public GridCell closeCheapestOpen() {
		CellInfo cellInfo = openQueue.poll();

		assert(cellInfo != null);

		cellInfo.setSet(Set.CLOSED);

		return cellInfo.getCell();
	}

	public GridCell getParent(GridCell cell) {
		CellInfo cellInfo = getCellInfo(cell);
		assert cellInfo != null;
		return cellInfo.getParent().getCell();
	}

	public void setParent(GridCell cell, GridCell parent) {
		CellInfo cellInfo = getCellInfo(cell);
		CellInfo parentInfo = getCellInfo(parent);

		assert cellInfo != null;
		assert parentInfo != null;

		cellInfo.setParent(parentInfo);
	}

	public float getFCost(GridCell cell) {
		CellInfo cellInfo = getCellInfo(cell);
		assert cellInfo != null;
		return cellInfo.getFCost();
	}

	public float getGCost(GridCell cell) {
		CellInfo cellInfo = getCellInfo(cell);
		assert cellInfo != null;
		return cellInfo.getGCost();
	}

	public void setGCost(GridCell cell, float gCost) {
		CellInfo cellInfo = getCellInfo(cell);
		assert cellInfo != null;
		assert cellInfo.getSet() != Set.OPEN;
		cellInfo.setGCost(gCost);
	}

	public float getHCost(GridCell cell) {
		CellInfo cellInfo = getCellInfo(cell);
		assert cellInfo != null;
		return cellInfo.getHCost();
	}

	public void setHCost(GridCell cell, float hCost) {
		CellInfo cellInfo = getCellInfo(cell);

		assert cellInfo != null;
		assert cellInfo.getSet() != Set.OPEN;

		cellInfo.setHCost(hCost);
	}

	public void setCosts(GridCell cell, float gCost, float hCost) {
		CellInfo cellInfo = getCellInfo(cell);

		assert cellInfo != null;
		assert cellInfo.getSet() != Set.OPEN;

		cellInfo.setGCost(gCost);
		cellInfo.setHCost(hCost);
	}

	/**
	 * Check if cell is in open set.
	 * @param cell the cell to check
	 * @return true if cell is in open set, otherwise false
	 */
	public boolean isOpen(GridCell cell) {
		CellInfo cellInfo = getCellInfo(cell);
		return (cellInfo == null) ? false : cellInfo.getSet() == Set.OPEN;
	}

	/**
	 * Check if cell is in closed set.
	 * @param cell the cell to check
	 * @return true if cell is in closed set, otherwise false
	 */
	public boolean isClosed(GridCell cell) {
		CellInfo cellInfo = getCellInfo(cell);
		return (cellInfo == null) ? false : cellInfo.getSet() == Set.CLOSED;
	}

	/* get cell info associated with cell. */
	private CellInfo getCellInfo(GridCell cell) {
		GridCoord gc = cell.getCoord();
		if (cells == null) System.out.println("cells is null");
		return cells[cell.getCoord().getX()][cell.getCoord().getY()];
	}
}
