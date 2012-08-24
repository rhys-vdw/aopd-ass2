package agents;
import pplanning.simviewer.model.GridDomain;
import pplanning.simviewer.model.GridState;
import pplanning.simviewer.model.GridCoord;

/**
 * A class to store metadata about the current map.
 *
 * Direct access to CellInfo instances cannot be provided to ensure that a cell
 * is always present in the open queue when it is in the open set.
 */
public class MapInfo {
	private CellInfo[][] cells;
	private PriorityQueue<CellInfo> openQueue = new PriorityQueue<CellInfo>();

	public MapInfo(GridDomain map) {
		cells = new StateInfo<S>[map.getWidth()][map.getHeight()];
	}

	/**
	 * Add cell to open set. This will fail if cell has already been added
	 * @param cell the cell
	 * @param gCost the cost to get to the cell
	 * @param hCost the heuristic estimate to get to the goal
	 */
	private void addCell(GridCell cell, float gCost, float hCost) {
		// should only be called when no cell already exists in array
		assert cells[cell.getCoord.getX()][cell.getCoord.getY()] == null;

		// add new node to array and open queue
		CellInfo cellInfo = new NodeInfo(cell, gCost, hCost, Set.OPEN);
		cells[cell.getCoord.getX()][cell.getCoord.getY()] = cellInfo;
		openQueue.add(cellInfo);

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
	 * Move cheapest open cell to the closed set and return it.
	 */
	public GridCell closeCheapestOpen() {
		CellInfo cellInfo = openQueue.poll();

		assert(cellInfo != null);

		cellInfo.setSet(Set.CLOSED);
	}

	public GridCell getParent(GridCell cell) {
		CellInfo cellInfo = getCellInfo(cell);
		assert cellInfo != null;
		return cellInfo.getParent();
	}

	public GridCell setParent(GridCell cell, GridCell parent) {
		CellInfo cellInfo = getCellInfo(cell);
		CellInfo parentInfo = getCellInfo(parent);

		assert cellInfo != null;
		assert parentInfo != null;

		return cellInfo.setParent(parentInfo);
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

	public float setGCost(GridCell cell, float gCost) {
		CellInfo cellInfo = getCellInfo(cell);
		assert cellInfo != null;
		assert cellInfo.getSet() != Set.OPEN
		cellInfo.setGCost(gCost);
	}

	public float getHCost(GridCell cell) {
		CellInfo cellInfo = getCellInfo(cell);
		assert cellInfo 
			= null;
		return cellInfo.getHCost();
	}

	public float setHCost(GridCell cell, float hCost) {
		CellInfo cellInfo = getCellInfo(cell);
		assert cellInfo != null;
		assert cellInfo.getSet() != Set.OPEN
		cellInfo.setHCost(fCost);
	}

	public void setCellCosts(GridCell cell, float gCost, float hCost) {
		CellInfo cellInfo = getCellInfo(cell);

		assert cellInfo != null;
		assert cellInfo.getSet() != Set.OPEN

		cellInfo.setGCost(gCost);
		cellInfo.setHCost(hCost);

		// TODO: treat open nodes differently
	}

	/**
	 * Check if cell is in open set.
	 * @param cell the cell to check
	 * @return true if cell is in open set, otherwise false
	 */
	public bool isCellOpen(GridCell cell) {
		CellInfo cellInfo = getCellInfo(cell);
		return (cellInfo == null) ? false : cellInfo.getSet() == Set.OPEN;
	}

	/**
	 * Check if cell is in closed set.
	 * @param cell the cell to check
	 * @return true if cell is in closed set, otherwise false
	 */
	public bool isCellClosed(GridCell cell) {
		CellInfo cellInfo = getCellInfo(cell);
		return (cellInfo == null) ? false : cellInfo.getSet() == Set.CLOSED;
	}

	/* get cell info associated with cell. */
	private CellInfo getCellInfo(GridCell cell) {
		return cells[cell.getCoord.getX()][cell.getCoord.getY()];
	}
}
