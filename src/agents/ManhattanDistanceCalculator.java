package agents;

import pplanning.simviewer.model.GridCell;

/**
 * Return the Manhattan distance between two nodes
 *
 */
class ManhattanDistanceCalculator implements DistanceCalculator {
	public int dCost(GridCell from, GridCell to) {
		return Math.abs(to.getCoord().getX() - from.getCoord().getX()) +
		       Math.abs(to.getCoord().getY() - from.getCoord().getY());
	}
}
