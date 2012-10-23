package agents;

import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridDomain;
import java.lang.IllegalArgumentException;

/**
 * A helper class to help search algorithms handle search domains with different
 * connectivity.
 */
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
	 * Get a distance calculator object for given map.
	 * @param map the map
	 */
	public static DistanceCalculator createDistanceCalculator(GridDomain map) {
		Connectivity connectivity = checkConnectivity(map);
		return getDistanceCalculator(connectivity);
	}

	/**
	 * Get a distance calculator object for a map of given connectivity.
	 * @param connectivity the connectivity of the map
	 */
	public static DistanceCalculator getDistanceCalculator(Connectivity connectivity) {
		switch (connectivity) {
			case MANHATTAN:
				return new ManhattanDistanceCalculator();
			case CHESSBOARD:
				return new ChessboardDistanceCalculator();
			default:
				throw new IllegalArgumentException(
						"No distance calculator found for '" + connectivity + "'!");
		}
	}

	/**
	 * Determine whether grid is four or eight directional. This is expensive,
	 * only ever call it once per execution.
	 * TODO: Is there a better way to do this?
	 * 
	 * Note: It seems that performance is better with the chessboard heuristic in both cases
	 * Always return chessboard calculator
	 */
	public static Connectivity checkConnectivity(GridDomain map) {
		// Get center cell (so that it is not on the perimeter)
//		GridCell cell = map.getCell(map.getWidth() / 2, map.getHeight() / 2);
//
//		// Judge map type on number of successors.
//		int successorCount = map.getSuccessors(cell).size();
//		if (successorCount == 8) {
			return Connectivity.CHESSBOARD;
//		}
//
//		assert successorCount == 4;
//
//		return Connectivity.MANHATTAN;
	}
}
