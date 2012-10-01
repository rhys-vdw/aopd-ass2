package agents;

import pplanning.simviewer.model.GridCell;

class ChessboardDistanceCalculator implements DistanceCalculator {
	public int dCost(GridCell from, GridCell to) {
		return Math.max(Math.abs(to.getCoord().getX() - from.getCoord().getX()),
		                Math.abs(to.getCoord().getY() - from.getCoord().getY()));
	}
}
