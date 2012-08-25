package agents;
import java.util.ArrayList;
import java.util.PriorityQueue;
import java.util.Map;
import java.util.HashMap;
import java.util.Collection;
import java.util.Set;
import java.util.HashSet;

import au.rmit.ract.planning.pathplanning.entity.ComputedPlan;
import au.rmit.ract.planning.pathplanning.entity.State;
import au.rmit.ract.planning.pathplanning.entity.Plan;
import au.rmit.ract.planning.pathplanning.entity.SearchDomain;
import pplanning.interfaces.PlanningAgent;
import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridDomain;

public class RhysAStar implements PlanningAgent {
	// plan to execute
	private ComputedPlan plan;

	@Override
	public GridCell getNextMove(GridDomain map, GridCell start, GridCell goal,
			int stepLeft, long stepTime, long timeLeft) {

		try {

			if (plan == null) {
				plan = generatePlan(map, start, goal);
			}

			GridCell nextStep = (GridCell) plan.getNextStep();

			if (nextStep == null) System.out.println("next step is null!");
			return nextStep;

		} catch (Exception e) {
			e.printStackTrace();
			return start;
		}
	}

	private ComputedPlan generatePlan(GridDomain map, GridCell start,
			GridCell goal) {

		MapInfo mapInfo = new MapInfo(map);

		// initialize open set with start node
		mapInfo.add(start, 0f, map.hCost(start, goal));

		// repeat while states are left in open set
		while (mapInfo.isOpenEmpty() == false) {
			GridCell current = mapInfo.closeCheapestOpen();

			System.err.println(current);

			// if goal has been reached, return path
			if (current == goal) {
				System.out.println("found goal!");
				return mapInfo.computePlan(goal);
			}

			// iterate through neighboring nodes
			for (State neighbor : map.getSuccessors(current)) {

				// consider node if it can be entered and is not in closed list
				if (map.isBlocked(neighbor) == false &&
				    mapInfo.isClosed((GridCell) neighbor) == false) {

					// get g cost of neighbor
					float gCost = mapInfo.getGCost(current) + map.cost(current, neighbor);

					if (mapInfo.isOpen((GridCell) neighbor) == false) {
						// node not previously encountered, add it to the open set
						mapInfo.add((GridCell) neighbor, gCost, map.hCost(neighbor, goal), current);
					} else if (gCost < mapInfo.getGCost((GridCell) neighbor)) {
						// more direct route to node found, update it
						// NOTE: this can never happen with an admissible heuristic!
						assert false;
						/*
						mapInfo.setCosts(neighbor, gCost, map.hCost(neighbor, goal));
						mapInfo.setParent(neighbor, current);
						*/
					}
				}
			}
		}
		// no goal found
		return null;
	}

	// Do we want to show extra info? (e.g., close and open nodes, current path)
	@Override
	public Boolean showInfo() {
		return false;
	}

	@Override
	public ArrayList<GridCell> expandedNodes() {
		//return new ArrayList<GridCell>(closed);
		return null;
	}

	@Override
	public ArrayList<GridCell> unexpandedNodes() {
		/*
		ArrayList<GridCell> cells = new ArrayList<GridCell>(open.size());
		for (StateInfo nodeInfo : open) {
			cells.add((GridCell) nodeInfo.gegoal());
		}
		return cells;
		*/
		return null;
	}


	@Override
	public ComputedPlan getPath() {
		//return plan;
		return null;
	}

}
