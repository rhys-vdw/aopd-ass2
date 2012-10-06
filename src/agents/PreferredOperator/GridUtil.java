package agents;

import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridDomain;

public class GridUtil {
	final static int[][] MANHATTAN_SUCCESSORS = {
		{ 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 } };

	final static int[][] CHESSBOARD_SUCCESSORS = {
		{ -1, 1  }, { 0, 1 },  {  1, 1 },
		{ -1, 0  },            {  1, 0 },
		{ -1, -1 }, { 0, -1 }, { -1, 0 } };

	public enum Connectivity {
		MANHATTAN  (MANHATTAN_SUCCESSORS),
		CHESSBOARD (CHESSBOARD_SUCCESSORS);

		private int[][] successors;

		public int[][] getSuccessors() {
			return successors;
		}

		Connectivity(int[][] successors) {
			this.successors = successors;
		}
	}

	/**
	 * Determine whether grid is four or eight directional. This is expensive,
	 * only ever call it once per execution.
	 * TODO: Is there a better way to do this?
	 */
	private Connectivity checkConnectivity(GridDomain map) {
		// Get center cell (so that it is not on the perimeter)
		GridCell cell = map.getCell(map.getWidth() / 2, map.getHeight() / 2);

		// Judge map type on number of successors.
		int successorCount = map.getSuccessors(cell).size();
		if (successorCount == 8) {
			return Connectivity.CHESSBOARD;
		}

		assert successorCount == 4;

		return Connectivity.MANHATTAN;
	}
}
