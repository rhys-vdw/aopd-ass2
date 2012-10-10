package agents;

import java.util.Comparator;
import pplanning.simviewer.model.GridCell;

public class WeightedGFComparator implements Comparator<GridCell> {

	private FastDasMapInfo mapInfo;
	private float weight;

	WeightedGFComparator(FastDasMapInfo mapInfo, float weight) {
		this.mapInfo = mapInfo;
		this.weight = weight;
	}

	float getWeightedFCost(GridCell cell) {
		return weight * mapInfo.getGCost(cell) + mapInfo.getHCost(cell);
	}

	/**
	 * Perform a comparison of two GridCells by f cost. Compares cells on their
	 * f cost breaking ties on h.
	 * @param a grid cell 1
	 * @param b grid cell 2
	 * @return 0, -1 or 1 if a is equal to, less than or greater than b respectively
	 */
	public int compare(GridCell a, GridCell b) {

		// Compare total cost estimate.
		int fCompare = FloatUtil.compare(getWeightedFCost(a), getWeightedFCost(b));
		if (fCompare != 0) {
			return fCompare;
		}

		// Break ties on heuristic estimate.
		int hCompare = FloatUtil.compare(mapInfo.getHCost(a), mapInfo.getHCost(b));
		return (hCompare == 0) ? -1 : hCompare;
	}
}
