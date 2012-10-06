package agents;

import java.util.ArrayList;

import au.rmit.ract.planning.pathplanning.entity.ComputedPlan;
import au.rmit.ract.planning.pathplanning.entity.State;
import au.rmit.ract.planning.pathplanning.entity.Plan;
import au.rmit.ract.planning.pathplanning.entity.SearchDomain;
import pplanning.interfaces.PlanningAgent;
import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridDomain;
import pplanning.simviewer.model.SuccessorIterator;

import java.io.PrintStream;
import java.io.OutputStream;
import java.lang.management.ManagementFactory;
import java.lang.management.ThreadMXBean;

public class DeadlineAwareSearch implements PlanningAgent
{
	private ComputedPlan plan;

	//DasMapInfo mapInfo;
	FastDasMapInfo mapInfo;

	// number of steps taken in current plan
	private int stepNo = 0;

	final private long MS_TO_NS_CONV_FACT = 1000000;

	// Percentage of deadline to be used generating plan. (As opposed to moving
	// along the plan afterwards.)
	final private long SEARCH_END_TIME_OFFSET = 20000000;

	// Should the open and closed sets be regenerated?
	boolean shouldUpdateOpen = false;
	boolean shouldUpdateClosed = false;

	private enum GridType { MANHATTAN, CHESSBOARD }

	private DistanceCalculator distanceCalculator = null;

	// r_default. Used before conExpansionIntervals has settled.
	// This is the number of expansions to perform before the sliding window is deemed 'settled'
	final private int SETTLING_EXPANSION_COUNT = 200;

	// Updating count that needs to be reached to indicate that we are settled.
	private int expansionCountForSettling = SETTLING_EXPANSION_COUNT;


	// This is the size of the sliding window, in entries.
	final private int EXPANSION_DELAY_WINDOW_LENGTH = 10;

	// Sliding window to calculate average single step error.
	private SlidingWindow expansionDelayWindow = new SlidingWindow(
			EXPANSION_DELAY_WINDOW_LENGTH);

	private SlidingWindow expansionTimeWindow = new SlidingWindow(
			EXPANSION_DELAY_WINDOW_LENGTH);

	// For timing
	final ThreadMXBean threadMX = ManagementFactory.getThreadMXBean();

	long timeAtLastExpansion;
	//long countExpansionsAtLastDifferentMeasurement;
	//long timePerExpansion = 1;

	private int expansionCount = 0;

	private boolean foundDASSolution = false;

	private GridCell lastGoal = null;

	@Override
	public GridCell getNextMove(GridDomain map, GridCell start, GridCell goal,
			int stepLeft, long stepTime, long timeLeft) {

		try {
			if (distanceCalculator == null)
			{
				GridType gridType = checkGridType(map);
				if (gridType == GridType.CHESSBOARD)
				{
					distanceCalculator = new ChessboardDistanceCalculator();
				}
				else if (gridType == GridType.MANHATTAN)
				{
					distanceCalculator = new ManhattanDistanceCalculator();
				}
			}
			//System.out.println("timeleft = " + timeLeft + " start = " + start);

			boolean bReplan =
					plan == null ||			// no last path stored, have yet notr planned before?
					map.getChangedEdges().size() > 0 ||	// map has had changes
					!lastGoal.equals(goal);
					
					

			if (bReplan)
			// If there is no plan, generate one.
			{

				// TODO: base search buffer on the length of the solution.
				long timeCurrent = threadMX.getCurrentThreadCpuTime();
				long searchTime = (long) ((timeLeft * MS_TO_NS_CONV_FACT) - SEARCH_END_TIME_OFFSET);
				long timeDeadline = timeCurrent + searchTime;

				// a new plan has been generated, update open and closed debug sets.
				shouldUpdateOpen = true;
				shouldUpdateClosed = true;

				plan = generatePlan(map, start, goal, timeDeadline);

				// If plan was not found, return start node.
				if (plan == null)
				{
					Trace.print("No plan found within deadline");
					return start;
				}

				// Plan was found, reset step count.
				stepNo = 0;
				lastGoal = goal;
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

		//System.out.println("Generating a new plan");

		// Map info exists outside of this function so that its open and closed
		// sets for debug display.
		//mapInfo = new DasMapInfo(map);
		mapInfo = new FastDasMapInfo(map);

		// Track the number of expansions performed -  e_curr value
		// TODO: investigate refactoring this to long to avoid potential truncactions in operations


		// Initialize open set with start node.
		int hCost = (int)map.hCost(start, goal);
		int dCost = distanceCalculator.dCost(start, goal);


		ComputedPlan incumbentPlan = null;
		incumbentPlan = speedierSearch(map, start,goal);

		assert threadMX.isCurrentThreadCpuTimeSupported();
		threadMX.setThreadCpuTimeEnabled(true);

		long timeAfterGreedy = threadMX.getCurrentThreadCpuTime();
		long timeUntilDeadline = (timeDeadline - timeAfterGreedy);

//		System.out.println("time after greedy: " + timeAfterGreedy);
//		System.out.println("time left: " + timeUntilDeadline);

		mapInfo.addStartCell(start, hCost, dCost);

		timeAtLastExpansion = threadMX.getCurrentThreadCpuTime();
		float dCheapestWithError = 1;
		int dMax = 2000;
		// Continue until time has run out
		while (timeUntilDeadline > 0)
		{
			//System.out.println("\n************STARTING NEW ITERATION*****************\n");
			//System.out.println("expansionCount/Settling = " + expansionCount + " / " + expansionCountForSettling);
			if (!mapInfo.isOpenEmpty()) 
			{

				GridCell current = mapInfo.closeCheapestOpen();
				//System.out.println("Closing " + current);
//				System.out.println("h: " +  mapInfo.getHCost(current) + " g: " + mapInfo.getGCost(current));

				// If this node has a higher g cost than the incumbent plan, discard it.
				if (incumbentPlan != null
						&& mapInfo.getGCost(current) > incumbentPlan.getCost()) {
					//System.out.println("Not bothering to explore cell " + current);
					timeUntilDeadline = timeDeadline - threadMX.getCurrentThreadCpuTime();
					continue;
				}

				// TODO: Moved these calculations up here, for debugging purposes.. They should really be calculated under the elsif case,
				// for maintainability
				if (expansionCount > expansionCountForSettling)
				{

					dCheapestWithError = mapInfo.getDCheapestWithError(current);
					dMax = calculateMaxReachableDepth(timeDeadline);

//					System.out.println("d^cheapest = " + dCheapestWithError +
//					           "\ndMax = " + dMax);
				}

				// If the current state is a goal state, and the cost to get there was cheaper
				// than that of the incumbent solution
				if (current == goal)
				{
					System.out.println("DAS Found path to goal! cost = " + mapInfo.getGCost(current));
					foundDASSolution = true;
					incumbentPlan = mapInfo.computePlan(goal);
					//return(incumbentPlan);
				}
				else if ( (expansionCount <= expansionCountForSettling) ||
						(dCheapestWithError < dMax))
				{

					// Generate all neighboring cells.
					//System.out.println("-----> current = " + current);
					SuccessorIterator neighborIter = map.getNextSuccessor(current);
					GridCell neighbor;
					
					while ((neighbor = neighborIter.next()) != null) 
					{
						//System.out.println(neighbor);
						generateCell(map, goal, current, neighbor);
					}

					// Increment number of expansions.
					expansionCount++;

					// Insert expansion delay into sliding window.
					int expansionDelay = expansionCount - mapInfo.getExpansionNumber(current);
					expansionDelayWindow.push(expansionDelay);

					// Calculate expansion interval.
					long timeCurrent = threadMX.getCurrentThreadCpuTime();
					long expansionTimeDelta = timeCurrent - timeAtLastExpansion;
					expansionTimeWindow.push(expansionTimeDelta);
					timeAtLastExpansion = timeCurrent;


				}
				else /* expansionCount > settlingCount && dCheapest > dMax */
				{
//					System.out.println("Pruning " + current.getCoord());
//					System.out.println("Pruning cell, expansion count = " + expansionCount + ", settleCount = " + expansionCountForSettling);
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
					expansionTimeWindow.reset();

					expansionCountForSettling = expansionCount + SETTLING_EXPANSION_COUNT;
					//System.out.println("Depruning - new settling limit = " + expansionCountForSettling);
				}
				else
				{
					System.out.println("Pruned and open are empty");
					break;
				}

				
			}
			timeUntilDeadline = timeDeadline - threadMX.getCurrentThreadCpuTime();
			//System.out.println("Time left: " + timeUntilDeadline);
		}
		//System.out.println("Returning solution with " + incumbentPlan.getLength() + " nodes");


		// This is where we make a hybrid speedier/DAS plan!
		if (!foundDASSolution && incumbentPlan != null)
		{
			ComputedPlan pathNew = new ComputedPlan();
			// Combined the DAS partial plan with the greedy solution
			for (int iterSteps = incumbentPlan.getLength()-1;
					iterSteps >= 0 ; iterSteps--)
			{

				GridCell cell = (GridCell) incumbentPlan.getStep(iterSteps);
				pathNew.prependStep(cell);
				if (mapInfo.cellExists(cell))
				{
					// We have an improved solution! Get the upstream from the DAS mapInfo!
					while (cell != null) {
						//System.out.println("Prepending " + cell);
						pathNew.prependStep(cell);
						cell = mapInfo.getParent(cell);
					}
					return pathNew;
				}
			}
			return incumbentPlan;

		}
		else
		{
			// Return the DAS solution
			return incumbentPlan;
		}
	}

	private void generateCell(GridDomain map, GridCell goal, GridCell parent, GridCell cell)
	{

			//System.out.println("Generating " + cell + " from parent: " + parent);
//					", expansionCount = " + expansionCount + " parent exp: " + mapInfo.getExpansionNumber(current));
			// consider node if it can be entered and is not in closed or pruned list
			if (map.isBlocked(cell) == false)
			{
				int gCost = mapInfo.getGCost(parent) + (int)map.cost(parent, cell);
				int hCost = (int)map.hCost(cell, goal);
				
				int dCheapestRaw = distanceCalculator.dCost(cell, goal);

				if (!mapInfo.cellExists(cell))
				{
					// Node has not been seen before, add it to the open set.
					mapInfo.add(cell, gCost, hCost, dCheapestRaw, expansionCount, parent);
//					System.out.println("child added has g: " + mapInfo.getGCost(cell) +
//					           " h: " + mapInfo.getHCost(cell) +
//					           " f: " + mapInfo.getFCost(cell) +
//					           " d^cheapest: " + mapInfo.getDCheapestWithError(cell));
				}
				else if (gCost < mapInfo.getGCost(cell))
				{
					// Shorter path to node found.
					mapInfo.setPathToCell(cell, gCost, expansionCount, parent);

					// If node was closed, put it back into the open list. The new cost
					// might make it viable. Pruned cells needn't be reopened as their
					// dCheapest value is unaffected.
					if (mapInfo.isClosed(cell))
					{
						mapInfo.reopenCell(cell);
					}
				}
			}
		}

	/**
	 * Estimate the number of expansions that can be performed before the deadline (dMax).
	 * @param timeDeadline the time that a solution must be found by (ns)
	 * @return estimated number of expansions or dMax
	 */
	public int calculateMaxReachableDepth(long timeDeadline)
	{
		double avgExpansionDelay = expansionDelayWindow.getAvg();

		int exp = calculateExpansionsRemaining(timeDeadline);
		int dMax = (int) (exp / avgExpansionDelay);

//		System.out.println("\n---------calculateMaxReachableDepth---------\n " +
//				"\nexpansions remaining: " + exp +
//				"\navgExpansionDelay: " + avgExpansionDelay +
//				"\n dMax: " + dMax);
		
		//System.out.println("dMax: " + dMax);
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
		long timeRemaining = timeDeadline - threadMX.getCurrentThreadCpuTime();
//		expansionTimeWindow.printAll();
		float averageExpTime = expansionTimeWindow.getAvg();

		int exp = (int) (timeRemaining / averageExpTime);

//		System.out.println("\n------calculateExpansionsRemaining-------\n" +
//				"\ntime remaining: " + timeRemaining +
//				"\ntime per expansion " + averageExpTime +
//				//"\naverage expansion rate: " + averageRate +
//				"\nexpansions remaining: " + exp + "\n");

		return exp;
	}

	private ComputedPlan speedierSearch(GridDomain map, GridCell start, GridCell goal)
	{

		GreedyMapInfo mapInfo = new GreedyMapInfo(map);
		float hCost = map.hCost(start, goal);
		mapInfo.add(start, 0, hCost);

		ComputedPlan incumbentPlan = null;

		while (mapInfo.isOpenEmpty() == false)
		{
			GridCell current = mapInfo.closeCheapestOpen();
			if (current == goal)
			{
				System.out.println("Goal found with speedier search, GCost " + mapInfo.getGCost(current));
				incumbentPlan = mapInfo.computePlan(current);
				break;
			}

			for (State neighbor : map.getSuccessors(current))
			{
				if (map.isBlocked(neighbor) == false &&
						mapInfo.isClosed((GridCell) neighbor) == false &&
						mapInfo.isOpen((GridCell)neighbor) == false)
				{
					float hNeighbor = map.hCost(neighbor, goal);
					float gNeighbor = mapInfo.getGCost((GridCell)current) + map.cost(current, neighbor);
					mapInfo.add((GridCell) neighbor, gNeighbor, hNeighbor, current);
				}
			}
		}

		return(incumbentPlan);
	}



	// -- Apparate Debug Output --

	private ArrayList<GridCell> closedNodes;
	private ArrayList<GridCell> prunedNodes;
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

			//closedNodes.addAll(prunedNodes);
		}
		return closedNodes;
	}

	@Override
	public ArrayList<GridCell> unexpandedNodes() {
		if (shouldUpdateOpen) {
			shouldUpdateOpen = false;
			openNodes = mapInfo.getOpenArrayList();
			prunedNodes = mapInfo.getPrunedArrayList();
			openNodes.addAll(prunedNodes);
		}
		return openNodes;
	}


	@Override
	public ComputedPlan getPath() {
		return plan;
	}

	/**
	 * Determine whether grid is four or eight directional. This is expensive,
	 * only ever call it once per execution.
	 * TODO: Is there a better way to do this?
	 */
	private GridType checkGridType(GridDomain map) {
		// Get center cell (so that it is not on the perimeter)
		GridCell cell = map.getCell(map.getWidth() / 2, map.getHeight() / 2);

		// Judge map type on number of successors.
		int successorCount = map.getSuccessors(cell).size();
		if (successorCount == 8) {
			return GridType.CHESSBOARD;
		}

		assert successorCount == 4;

		return GridType.MANHATTAN;
	}
}

