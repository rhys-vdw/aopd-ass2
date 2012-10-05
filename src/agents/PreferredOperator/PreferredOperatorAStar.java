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

public class PreferredOperatorAStar implements PlanningAgent {

	private ComputedPlan plan;
	PreferredMapInfo mapInfo;

	private int stepNo = 0;

	private GridCell lastGoal = null;

	@Override
	public GridCell getNextMove(GridDomain map, GridCell start, GridCell goal,
			int stepLeft, long stepTime, long timeLeft) {

		try {
			GridCell nextStep = null;

      /*
			boolean replan =
				plan == null                     ||			// no last path stored, have yet notr planned before?
				map.getChangedEdges().size() > 0 ||	// map has had changes
				!lastGoal.equals(goal) || // Goal has changed (equals not implemented?)
				!plan.contains(start); // sNode is not in the path (sNode out of track)
        */

			//if (replan) {
			if (plan == null) {
				plan = generatePlan(map, start, goal);
				stepNo = 0;
				lastGoal = goal;
			}

			if (plan == null) {
				System.out.println("Could not find goal");
				return start;
			}

			if (stepNo < plan.getLength()) {
				nextStep = (GridCell) plan.getStep(stepNo++);

				if (nextStep == null) {
					Trace.print("next step is null!");
				}
			}

			return nextStep;

		}
		catch (Exception e) {
			e.printStackTrace();
			return start;
		}
	}

	private ComputedPlan generatePlan(GridDomain map, GridCell start,
			GridCell goal) {

		mapInfo = new PreferredMapInfo(map);

		// initialize open set with start node
		mapInfo.addStartCell(start, map.hCost(start, goal));

		// repeat while states are left in open set
		while (mapInfo.isOpenEmpty() == false) {
			GridCell current = mapInfo.closeCheapestOpen();

			// if goal has been reached, return path
			if (current == goal) {
				return mapInfo.computePlan(goal);
			}

			// iterate through neighboring nodes
			for (State state : map.getSuccessors(current)) {
				GridCell neighbor = (GridCell) state;

				// consider node if it can be entered and is not in closed list
				if (map.isBlocked(neighbor)) continue;

				// get g cost of neighbor
				float gCost = mapInfo.getGCost(current) + map.cost(current, neighbor);

				if (mapInfo.getSetMembership(neighbor) == CellSetMembership.NONE) {
					// Node not previously encountered, add it to the open set.
					mapInfo.add(neighbor, gCost, map.hCost(neighbor, goal), current);
				} else if (FloatUtil.compare(gCost, mapInfo.getGCost(neighbor)) == -1) {
					// Cheaper route to node found to node in the open set.
					mapInfo.setGCost(neighbor, gCost);
					mapInfo.setParent(neighbor, current);
				}
			}
		}
		// No goal found.
		return null;
	}

	// Do we want to show extra info? (e.g., close and open nodes, current path)
	@Override
	public Boolean showInfo() {
		return mapInfo != null;
	}

	@Override
	public ArrayList<GridCell> expandedNodes() {
		return mapInfo.getClosedArrayList();
	}

	@Override
	public ArrayList<GridCell> unexpandedNodes() {
		return mapInfo.getOpenArrayList();
	}


	@Override
	public ComputedPlan getPath() {
		return plan;
	}
}
