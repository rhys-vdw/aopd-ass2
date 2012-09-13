package agents;

import java.util.ArrayList;

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

	final private long MS_TO_NS_CONV_FACT = 1000000;

	// Percentage of deadline to be used generating plan. (As opposed to moving
	// along the plan afterwards.)
	final private float SEARCH_TIME_FRACTION = 0.97f;

	// Should the open and closed sets be regenerated?
	boolean shouldUpdateOpen = false;
	boolean shouldUpdateClosed = false;

	// These values needs tuning!

	// r_default. Used before conExpansionIntervals has settled.
	// This is the number of expansions to perform before the sliding window is deemed 'settled'
	final private int SETTLING_EXPANSION_COUNT = 200;

	// This is the size of the sliding window, in entries.
	final private int EXPANSION_DELAY_WINDOW_LENGTH = 15;

	// Time in ns to use as the expected interval between expansions, before settling.
	// Shouldn't be used with the new refactoring.
	final private long SETTLING_EXPANSION_AVG_INTERVAL = 30;

	// Sliding window to calculate average single step error.
	private SlidingWindow expansionDelayWindow = new SlidingWindow(
			EXPANSION_DELAY_WINDOW_LENGTH);

	// r value
	private SlidingWindow expansionIntervalWindow = new SlidingWindow(
			EXPANSION_DELAY_WINDOW_LENGTH);

	@Override
	public GridCell getNextMove(GridDomain map, GridCell start, GridCell goal,
			int stepLeft, long stepTime, long timeLeft) {

		try {
			Trace.Enable(true);

			// If there is no plan, generate one.
			if (plan == null)
			{
				// a new plan has been generated, update open and closed debug sets.
				shouldUpdateOpen = true;
				shouldUpdateClosed = true;

				// TODO: base search buffer on the length of the solution.
				long timeCurrent = System.nanoTime();
				long searchTime = (long) ((timeLeft * MS_TO_NS_CONV_FACT) * SEARCH_TIME_FRACTION);
				long timeDeadline = timeCurrent + searchTime;

				Trace.print("current time (ns): " + timeCurrent);
				Trace.print("deadline: " + timeDeadline);
				Trace.Enable(false);
				plan = generatePlan(map, start, goal, timeDeadline);

				// If plan was not found, return start node.
				if (plan == null)
				{
					Trace.print("No plan found within deadline");
					return start;
				}

				// Plan was found, reset step count.
				stepNo = 0;
			}

			// Check if path has been exhausted.
			if (stepNo >= plan.getLength()) {
				return start;
			}

			// Return the next step in the path.
			return (GridCell) plan.getStep(stepNo++);
		}
		catch (Exception e)
		{
			// Catch all exceptions before the propagate into Apparate.
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
	 * 5)		if Open is not empty
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

		System.out.println("Generating a new plan");

		// Map info exists outside of this function so that its open and closed
		// sets for debug display.
		mapInfo = new DasMapInfo(map);

		// Track the number of expansions performed -  e_curr value
		// TODO: investigate refactoring this to long to avoid potential truncactions in operations
		int expansionCount = 0;

		// Initialize open set with start node.
		float hCost = map.hCost(start, goal);
		int dCost = dCostManhattan((GridCell)start, goal);
		mapInfo.addStartCell(start, hCost, dCost);

		ComputedPlan incumbentPlan = null;

		// Continue until time has run out
		while (System.nanoTime() < timeDeadline)
		{
			// TODO: is this a good inital value?
			long prevExpansionTime = System.nanoTime();

			if (!mapInfo.isOpenEmpty()) {
				GridCell current = mapInfo.closeCheapestOpen();

				// If this node has a higher g cost than the incumbent plan, discard it.
				if (incumbentPlan != null
						&& mapInfo.getGCost(current) > incumbentPlan.getCost()) {
					continue;
				}

				//System.out.println(current.getCoord() +" " +  mapInfo.getFCost(current));
				// If the current state is a goal state, and the cost to get there was cheaper
				// than that of the incumbent solution
				if (current == goal)
				{
					System.out.println("Found cheaper path to goal!");
					//TODO: this is a potentially expensive operation!
					incumbentPlan = mapInfo.computePlan(goal);
					// The below hack is to test finding the first goal!
					//return incumbentPlan;
				}
				else if (expansionCount <= SETTLING_EXPANSION_COUNT ||
						(mapInfo.getDCheapestWithError(current) < calculateMaxReachableDepth(timeDeadline)))
				{
					//Trace.print("(reachable) d_cheapest: " + estimateGoalDepth(current) + " d_max: " + dMax);

					// Expand current node. TODO: move this into its own method.
					for (State neighbor : map.getSuccessors(current))
					{
						// consider node if it can be entered and is not in closed or pruned list
						if (map.isBlocked(neighbor) == false &&
						    mapInfo.isClosed((GridCell) neighbor) == false &&
						    mapInfo.isPruned((GridCell) neighbor) == false)
						{
							if (mapInfo.isOpen((GridCell) neighbor) == false)
							{
								float neighborGCost = mapInfo.getGCost(current) + map.cost(current, neighbor);
								float neighborHCost = map.hCost(neighbor, goal);

								//System.out.println("g: " + fNeighborGCost + " h" + fNeighborHCost);
								// TODO: this is currently assuming manhattan grid world. See Issue #11
								int neighborDCheapestRaw = (int) dCostManhattan((GridCell)neighbor, goal);
								//System.out.println("d" + nNeighbourDCost);
								// Add the neighbor to the open set!
								mapInfo.add((GridCell) neighbor, neighborGCost, neighborHCost,
										neighborDCheapestRaw, expansionCount, current);
							}
							// Do we need the following case handling? Is the above enough to add s' to open? Step 12 of algorithm
							/* else if (gCost < mapInfo.getGCost((GridCell) neighbor)) */
						}
					}
					// Increment number of expansions.
					expansionCount++;

					// Insert expansion delay into sliding window.
					int expansionDelay = expansionCount - mapInfo.getExpansionNumber(current);
					expansionDelayWindow.push(expansionDelay);

					// Calculate expansion interval.
					long currentTime = System.nanoTime();
					long expansionInterval = currentTime - prevExpansionTime;
					prevExpansionTime = currentTime;

					// Insert expansion interval into sliding window.
					expansionIntervalWindow.push(expansionInterval);
				}
				else
				{
					mapInfo.pruneCell(current);
				}
			}
			else
			{
				// Open list is empty, so we need to repopulate it.
				if (!mapInfo.isPrunedEmpty())
				{
					int exp = calculateExpansionsRemaining(timeDeadline);
					mapInfo.recoverPrunedStates(exp);
					expansionDelayWindow.reset();
					expansionIntervalWindow.reset();
				}
				else
				{
					System.out.println("Pruned and open are empty");
					break;
				}
				//System.out.print(exp);

			}
		}
		//System.out.println("Returning solution with " + mapInfo.GetIncumbentPlan().getLength() + " nodes");
		return incumbentPlan;
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
	}

	/**
	 * Estimate the number of expansions that can be performed before the deadline (dMax).
	 * @param timeDeadline the time that a solution must be found by (ns)
	 * @return estimated number of expansions or dMax
	 */
	public int calculateMaxReachableDepth(long timeDeadline)
	{
		double avgExpansionDelay = expansionDelayWindow.getAvg();

		int dMax = (int) (calculateExpansionsRemaining(timeDeadline) / avgExpansionDelay);

		Trace.print(dMax + " maximum reachable depth");
		return dMax;
	}

	/**
	 * Get the predicted number of expansions that can be performed in the
	 * specified time. The estimate is based on the current average expansion rate
	 * and the time remaining.
	 *
	 *       exp = t * r
	 *
	 *       where t is time remaining
	 *         and r is the current average expansion rate
	 *
	 * @param timeDeadline the time of the deadline, in nanoseconds
	 * @return expansions remaining
	 */
	public int calculateExpansionsRemaining(long timeDeadline)
	{
		long timeRemaining = timeDeadline - System.nanoTime();
		float averageInterval = expansionIntervalWindow.getAvg();
		float averageRate = 1 / averageInterval;

		int exp = (int) (timeRemaining * averageRate);

		Trace.print("Calculating expansions remaining:" +
				"\nexpansions remaining: " + exp +
				"\naverage expansion rate: " + averageRate);

		return exp;
	}

	// -- Apparate Debug Output --

	private ArrayList<GridCell> closedNodes;
	private ArrayList<GridCell> openNodes;

	@Override
	public Boolean showInfo() {
		//return false;
		return mapInfo != null;
	}

	@Override
	public ArrayList<GridCell> expandedNodes() {
		if (shouldUpdateClosed) {
			shouldUpdateClosed = false;
			closedNodes = mapInfo.getClosedArrayList();
		}
		return closedNodes;
	}

	@Override
	public ArrayList<GridCell> unexpandedNodes() {
		if (shouldUpdateOpen) {
			shouldUpdateOpen = false;
			openNodes = mapInfo.getOpenArrayList();
		}
		return openNodes;
	}


	@Override
	public ComputedPlan getPath() {
		return plan;
	}
}
