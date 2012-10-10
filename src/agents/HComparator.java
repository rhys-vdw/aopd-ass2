package agents;

import java.util.Comparator;
import pplanning.simviewer.model.GridCell;

public class HComparator implements Comparator<GridCell> {

	private FastDasMapInfo mapInfo;

	HComparator(FastDasMapInfo mapInfo) {
		this.mapInfo = mapInfo;
	}

	/**
	 * Perform a comparison of two GridCells by h cost.
	 * @param a grid cell 1
	 * @param b grid cell 2
	 * @return 0, -1 or 1 if a is equal to, less than or greater than b respectively
	 */
	public int compare(GridCell a, GridCell b) {
		return FloatUtil.compare(mapInfo.getHCost(a), mapInfo.getHCost(b));
	}
}
