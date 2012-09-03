package agents;

import pplanning.simviewer.model.GridCell;

class DasCellInfo implements Comparable<DasCellInfo> {
		private DasCellInfo parent = null;
		private final GridCell cell;
		private float gCost;
		private float hCost;
		private float fCost;
		private int dCheapestRaw;
		private CellSetMembership cellSetMembership;
		private int nExpansionNumber; // GS: should this be final?

		/**
		 * Set the estimated number of expansions from this node to the goal state.
		 * This is the raw estimate, not including mean step error.
		 *
		 * TODO: Should this be a float? I think int makes sense for a d estimate.
		 *
		 * @param dCheapestRaw the new d value
		 */
		public void setDCheapestRaw(int dCheapestRaw) {
			this.dCheapestRaw = dCheapestRaw;
		}

		public int getDCheapestRaw() {
			return this.dCheapestRaw;
		}

		public float getDCheapestWithError() {
			// TODO: return d^cheapest(cell)
			return 0;
		}

		/** Read only. */
		public GridCell getCell() {
			return this.cell;
		}

		public DasCellInfo getParent() {
			return this.parent;
		}
		public void setParent(DasCellInfo parent) {
			this.parent = parent;
		}

		public float getGCost() {
			return this.gCost;
		}
		public void setGCost(float gCost) {
			this.gCost = gCost;
			updateFCost();
		}

		public float getHCost() {
			return this.hCost;
		}

		public void setHCost(float hCost) {
			this.hCost = hCost;
			updateFCost();
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
		
		public int getExpansionNumber()
		{
			return(this.nExpansionNumber);
		}

		/**
		 * Compare the f cost of this cell to another.
		 * @other the CellInfo to be compared
		 */
		public int compareTo(DasCellInfo other) {
			// compare f cost of either cell
			if (this.fCost < other.fCost) return -1;
			if (this.fCost > other.fCost) return  1;
			// in the event of a tie, return the cell with the lowest h cost
			if (this.hCost < other.hCost) return -1;
			if (this.hCost > other.hCost) return  1;
			// both values are the same
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
		public DasCellInfo(GridCell cell, float gCost, float hCost,
				CellSetMembership cellMembership, DasCellInfo parent, int nExpansion) {
			this(cell, gCost, hCost, cellMembership, nExpansion);
			this.parent = parent;
		}

		/**
		 * Create a new CellInfo for specified cell with specified g and h costs.
		 * @cell           the cell
		 * @gCost          the cost of the path from start to cell
		 * @hCost          the heuristic estimate of the remaining cost to the goal
		 * @cellMembership the set that this node should start in
		 */
		public DasCellInfo(GridCell cell, float gCost, float hCost, CellSetMembership cellMembership, int nExpansion) {
			this.cell = cell;
			this.gCost = gCost;
			updateFCost();
			this.cellSetMembership = cellMembership;
			this.nExpansionNumber = nExpansion;
		}
	}
