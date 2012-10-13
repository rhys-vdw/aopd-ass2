package agents;

import pplanning.simviewer.model.GridCell;

/**
 * Return the chessboard distance between two points
 */
class ChessboardDistanceCalculator implements DistanceCalculator {
	public int dCost(GridCell from, GridCell to) {
		return Math.max(Math.abs(to.getCoord().getX() - from.getCoord().getX()),
		                Math.abs(to.getCoord().getY() - from.getCoord().getY()));
	}
}
