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


public class DeadlineAwareSearch implements PlanningAgent 
{
	private ComputedPlan plan;

	DasMapInfo mapInfo;
	
	// number of steps taken in current plan
	private int stepNo = 0;

	// goal state from previous call to getNextMove()
	private GridCell lastGoal = null;

	final private long MS_TO_NS_CONV_FACT = 1000000;

	@Override
	public GridCell getNextMove(GridDomain map, GridCell start, GridCell goal,
			int stepLeft, long stepTime, long timeLeft) {

		try {
			Trace.Enable(true);
			GridCell nextStep = null;

			// recalculate plan if:
			//  - no plan yet
			//  - environment has changed
			//  - goal has changed             TODO: check that equals actually works
			//  - current node is not in path  TODO: is this a linear search?
			//
			// NOTE: I think Sebastian explicitly said that the goal wont move and the
			// map wont change, so we could probably drop the last two conditions
			boolean bReplan =
				plan == null ||
				map.getChangedEdges().size() > 0 ||
				!lastGoal.equals(goal) ||
				!plan.contains(start);


			if (bReplan) 
			{
				long timeCurrent = System.nanoTime();
				// TODO: Should we provide a buffer, i.e. 1% of the time, for provision of the plan to the recipient
				long timeDeadline = timeCurrent + timeLeft * MS_TO_NS_CONV_FACT;
				Trace.print("current time (ns): " + timeCurrent);
				Trace.print("deadline: " + timeDeadline);
				plan = generatePlan(map, start, goal, timeDeadline);
				stepNo = 0;
				lastGoal = goal;
			}

			// take next step in plan
			if (stepNo < plan.getLength())
			{
				nextStep = (GridCell) plan.getStep(stepNo++);

				if (nextStep == null) 
				{
					Trace.print("next step is null!");
				}
			}

			return nextStep;

		} 
		catch (Exception e) 
		{
			e.printStackTrace();
			return start;
		}
	}

	/*
	 * Algorithm description
	 * 1) 	Initialise Open with starting state
	 * 2) 	Initialise Pruned with empty structure
	 * 3) 	Initialise Incumbent plan with NULL
	 * 4) 	while (current time < deadline)
	 * 		{
	 * 5)			if Open is not empty
	 * 			{
	 * 6)			max_reachable_depth = calculate_d_bound()
	 * 7)			state s = open.pop()
	 * 8) 			if goal(s) && s > incumbent
	 *  			{
	 * 9) 				incumbent = s
	 *   			}
	 * 10) 			else if cheapest_solution_depth < max_reachable_depth
	 *   			{
	 * 11) 				s' = for each child of s
	 *   				{
	 * 12) 					open.push(s') 
	 *   				}
	 *   			}
	 * 13) 			else
	 *   			{
	 * 14) 				pruned.push(s)
	 *   			}
	 *   		}
	 * 15) 		else
	 *   		{
	 * 16) 			recover_pruned_states(open, pruned)
	 *   		}
	 *   	}
	 *   
	 * 17) 	return incumbent	
	 */
	private ComputedPlan generatePlan(GridDomain map, GridCell start,
			GridCell goal, long timeDeadline) {

		ComputedPlan planIncumbent = null;
		int nMaxReachableDepth = Integer.MAX_VALUE;
		int nEstimatedDepth = Integer.MAX_VALUE;
		mapInfo = new DasMapInfo(map);

		// initialize open set with start node
		mapInfo.add(start, 0f, map.hCost(start, goal));

		while (System.nanoTime() < timeDeadline)
		{
		
			if (!mapInfo.isOpenEmpty())
			{
				
				nMaxReachableDepth = CalculateMaxReachableDepth();
				GridCell current = mapInfo.closeCheapestOpen();
				
				// If the current state is a goal state, and the cost to get there was cheaper
				// than that of the incumbent solution
				if ( current == goal)
				{
					Trace.print("found path to goal!");
					if ( (mapInfo.GetIncumbentPlan() == null) || 
						 (mapInfo.getGCost(goal) < mapInfo.GetIncumbentPlan().getCost()) )
					{
						Trace.print("new path to goal is an improvement!");
						mapInfo.computePlan(goal);
					}
					else
					{
						Trace.print("new path to goal is worse than incumbent!");
					}
						
				}
				else if (EstimateGoalDepth(current) < nMaxReachableDepth)
				{
					for (State neighbor : map.getSuccessors(current)) 
					{
						// consider node if it can be entered and is not in closed or pruned list
						if (map.isBlocked(neighbor) == false &&
						    mapInfo.isClosed((GridCell) neighbor) == false &&
						    mapInfo.isPruned((GridCell) neighbor) == false) 
						{
							
						}
					}
				}
				
			}
		// repeat while states are left in open set
		//while (mapInfo.isOpenEmpty() == false) {
		//	GridCell current = mapInfo.closeCheapestOpen();

			//Trace.print(current);

			// if goal has been reached, return path
//			if (current == goal) {
//				Trace.print("found goal!");
//				return mapInfo.computePlan(goal);
//			}
//
//			// iterate through neighboring nodes
//			for (State neighbor : map.getSuccessors(current)) {
//
//				// consider node if it can be entered and is not in closed list
//				if (map.isBlocked(neighbor) == false &&
//				    mapInfo.isClosed((GridCell) neighbor) == false) {
//
//					// get g cost of neighbor
//					float gCost = mapInfo.getGCost(current) + map.cost(current, neighbor);
//
//					if (mapInfo.isOpen((GridCell) neighbor) == false) {
//						// node not previously encountered, add it to the open set
//						mapInfo.add((GridCell) neighbor, gCost, map.hCost(neighbor, goal), current);
//					} else if (gCost < mapInfo.getGCost((GridCell) neighbor)) {
//						// more direct route to node found, update it
//						// NOTE: this can never happen with an admissible heuristic!
//						assert false;
//						/*
//							 mapInfo.setCosts(neighbor, gCost, map.hCost(neighbor, goal));
//							 mapInfo.setParent(neighbor, current);
//							 */
//					}
//				}
//			}
//		}
		// no goal found
		}
		return(planIncumbent);
	}
	



	/**
	 * Estimate the number of expansions required to move from one state to
	 * another in a gridworld where only four directional movement is permitted. 
	 * @param from the starting state
	 * @param to the goal state
	 */
	private int dCostManhattan(GridCell from, GridCell to) {
		return Math.abs(to.getCoord().getX() - from.getCoord().getX()) +
		       Math.abs(to.getCoord().getY() - from.getCoord().getY());
	}

	/**
	 * Estimate the number of expansions required to move from one state to
	 * another in a gridworld where diagonal movement is permitted. 
	 * @param from the starting state
	 * @param to the goal state
	 */
	private int dCostEuclidean(GridCell from, GridCell to) {
		return Math.max(Math.abs(to.getCoord().getX() - from.getCoord().getX()),
		                Math.abs(to.getCoord().getY() - from.getCoord().getY()));
		// GS: Why max here? Wouldn't it be c = sqrt(a^2 + b^2) ?
	}

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
	
	public int CalculateMaxReachableDepth()
	{
		int nMaxDepth = Integer.MAX_VALUE;
		
		
		
		return(nMaxDepth);
	}
	
	public int EstimateGoalDepth(GridCell _cell)
	{
		int nEstimatedGoalDepth = Integer.MAX_VALUE;
		
		return(nEstimatedGoalDepth);
	}
	

}
