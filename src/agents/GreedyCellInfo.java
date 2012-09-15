package agents;

import pplanning.simviewer.model.GridCell;

class GreedyCellInfo implements Comparable<GreedyCellInfo> {
		private GreedyCellInfo parent = null;
		private final GridCell cell;
		private float gCost;
		private float hCost;
		private float fCost;
		private CellSetMembership cellSetMembership;

		/** Read only. */
		public GridCell getCell() {
			return this.cell;
		}

		public GreedyCellInfo getParent() {
			return this.parent;
		}
		public void setParent(GreedyCellInfo parent) {
			this.parent = parent;
		}

		public float getGCost() {
			return this.gCost;
		}
		public void setGCost(float gCost) {
			this.gCost = gCost;
			//updateFCost();
		}

		public float getHCost() {
			return this.hCost;
		}

		public void setHCost(float hCost) {
			this.hCost = hCost;
			//updateFCost();
		}

		/** Read only. */
		public float getFCost() {
			return this.fCost;
		}

		public CellSetMembership getCellMembership() {
			return this.cellSetMembership;
		}

		public void setCellMembership(CellSetMembership cellMembership) {
			this.cellSetMembership = cellMembership;
		}

		/**
		 * Compare the f cost of this cell to another.
		 * @other the CellInfo to be compared
		 */
		public int compareTo(GreedyCellInfo other) {
			// compare f cost of either cell
			if (this.hCost < other.hCost) return -1;
			if (this.hCost > other.hCost) return  1;
			return 0;
		}

		void updateFCost() {
			this.fCost = gCost + hCost;
		}

		/**
		 * Create a new CellInfo for specified cell with specified g and h costs.
		 * @cell           the cell
		 * @gCost          the cost of the path from start to cell
		 * @hCost          the heuristic estimate of the remaining cost to the goal
		 * @cellMembership the set that this node should start in
		 * @parent         the parent node of this cell
		 */
		public GreedyCellInfo(GridCell cell, float gCost, float hCost,
				CellSetMembership cellMembership, GreedyCellInfo parent) 
		{
			this.cell = cell;
			this.gCost = gCost;
			this.hCost = hCost;
			this.parent = parent;
			this.cellSetMembership = cellMembership;
		}

		/**
		 * Create a new CellInfo for specified cell with specified g and h costs.
		 * @cell           the cell
		 * @gCost          the cost of the path from start to cell
		 * @hCost          the heuristic estimate of the remaining cost to the goal
		 * @cellMembership the set that this node should start in
		 */
		public GreedyCellInfo(GridCell cell, float gCost, float hCost, CellSetMembership cellMembership) {
			this.cell = cell;
			this.gCost = gCost;
			this.hCost = hCost;
			//updateFCost();
			this.cellSetMembership = cellMembership;
		}
	}
