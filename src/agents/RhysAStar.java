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
	public GridCell getNextMove(GridDomain map, GridCell sState, GridCell tState,
			int stepLeft, long stepTime, long timeLeft) {

		try {

			if (plan == null) {
				plan = generatePlan(map, sState, tState);
			}

			GridCell nextStep = (GridCell) plan.getNextStep();

			//System.out.println("next step: " + nextStep);
			return (GridCell) nextStep;

		} catch (Exception e) {
			//e.printStackTrace();
			return sState;
		}
	}

	// list of nodes yet to be evaluated, sorted by f value
	// TODO: consider setting initial capacity
	PriorityQueue<StateInfo> open = new PriorityQueue<StateInfo>();

	private ComputedPlan generatePlan(GridDomain map, GridCell sState,
			GridCell tState) {

		// initialize open set with start node
		open.add(new StateInfo(sState, 0f, map.hCost(sState, tState)));

		// repeat while states are left in open set
		while (open.isEmpty() == false) {
			StateInfo current = open.peek();

			// if goal has been reached, return path
			if (current.getState() == tState) {
				ComputedPlan plan = new ComputedPlan();
				computePlan(plan, current);
				plan.setCost(current.getGCost());
				return plan;
			}

			// move current from open to closed set
			open.poll();
			closed.add((GridCell) current.getState());

			// iterate through neighboring nodes
			for (State neighbor : map.getSuccessors(current.getState())) {

				// consider node if it can be entered and is not in closed list
				if (map.isBlocked(neighbor) == false && closed.contains(neighbor) == false) {
					// get g cost of neighbor
					float gCost = current.getGCost() + map.cost(current.getState(), neighbor);

					// search for a StateInfo with node neighbor (this works because I have
					// overloaded StateInfo's equals method)
					StateInfo openState = findStateInfo(open, neighbor);
					if (openState == null) {
						// node not in open set, add it
						StateInfo newState = new StateInfo(neighbor, gCost,
								map.hCost(neighbor, tState));
						newState.setParent(current);
						open.add(newState);
					} else if (gCost < openState.getGCost()) {
						// more direct route to node found, update it
						openState.setGCost(gCost);
						openState.setHCost(map.hCost(neighbor, tState));
						openState.setParent(current);
					}
				}
			}
		}
		// no goal found
		return null;
	}

	ComputedPlan computePlan(ComputedPlan plan, StateInfo goal) {
		while (goal.getParent() != null) {
			plan.prependStep(goal.getState());
			goal = goal.getParent();
		}
		return plan;
	}

	// Do we want to show extra info? (e.g., close and open nodes, current path)
	@Override
	public Boolean showInfo() {
		return true;
	}

	@Override
	public ArrayList<GridCell> expandedNodes() {
		return new ArrayList<GridCell>(closed);
	}

	@Override
	public ArrayList<GridCell> unexpandedNodes() {
		ArrayList<GridCell> cells = new ArrayList<GridCell>(open.size());
		for (StateInfo nodeInfo : open) {
			cells.add((GridCell) nodeInfo.getState());
		}
		return cells;
	}


	@Override
	public ComputedPlan getPath() {
		//return plan;
		return null;
	}

}
