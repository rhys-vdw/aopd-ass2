package agents;

import pplanning.simviewer.model.GridCell;

class CellInfo implements Comparable<CellInfo> {
		private CellInfo parent = null;
		private final GridCell cell;
		private float gCost;
		private float hCost;
		private float fCost;
		private Set set;

		/** Read only. */
		public GridCell getCell() {
			return this.cell;
		}

		public CellInfo getParent() {
			return this.parent;
		}
		public void setParent(CellInfo parent) {
			this.parent = parent;
		}

		public float getGCost() {
			return this.gCost;
		}
		public void setGCost(float gCost) {
			this.gCost = gCost;
		}

		public float getHCost() {
			return this.hCost;
		}

		public void setHCost(float hCost) {
			this.hCost = hCost;
		}

		/** Read only. */
		public float getFCost() {
			return this.gCost + this.hCost;
		}

		public Set getSet() {
			return this.set;
		}

		public void setSet(Set set) {
			this.set = set;
		}

		/**
		 * Compare the f cost of this cell to another.
		 * @other the CellInfo to be compared
		 */
		public int compareTo(CellInfo other) {
			if (this.fCost < other.fCost) return -1;
			if (this.fCost > other.fCost) return 1;
			return 0;
		}

		/**
		 * Create a new CellInfo for specified cell with specified g and h costs.
		 * @cell   the cell
		 * @gCost  the cost of the path from start to cell
		 * @hCost  the heuristic estimate of the remaining cost to the goal
		 * @set    the set that this node should start in
		 * @parent the parent node of this cell
		 */
		public CellInfo(GridCell cell, float gCost, float hCost, Set set, CellInfo parent) {
			this(cell, gCost, hCost, set);
			this.parent = parent;
		}

		/**
		 * Create a new CellInfo for specified cell with specified g and h costs.
		 * @cell  the cell
		 * @gCost the cost of the path from start to cell
		 * @hCost the heuristic estimate of the remaining cost to the goal
		 * @set   the set that this node should start in
		 */
		public CellInfo(GridCell cell, float gCost, float hCost, Set set) {
			this.cell = cell;
			this.gCost = gCost;
			this.fCost = gCost + hCost;
			this.set = set;
		}
	}
