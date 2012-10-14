package agents;

import java.util.ArrayList;

import au.rmit.ract.planning.pathplanning.entity.ComputedPlan;
import au.rmit.ract.planning.pathplanning.entity.State;
import pplanning.interfaces.PlanningAgent;
import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridDomain;
import pplanning.simviewer.model.SuccessorIterator;

// This is actually DeadlineAwareSearch!
public class MyCoolAgent implements PlanningAgent
{
	// Store our persistent solutions.
	private ComputedPlan plan;

	// The old map info class with dynamic node allocation
	//DasMapInfo mapInfo;

	// Use the map info class with static allocation!
	FastDasMapInfo mapInfo;

	// Number of steps taken in current plan
	private int stepNo = 0;

	// Conversion factor for MS (apparate interface) to nanosecond (our internal timing unit)
	final private long MS_TO_NS_CONV_FACT = 1000000;

	// Static amount of time to be used generating plan. (As opposed to moving
	// along the plan afterwards.)
	final private long SEARCH_END_TIME_OFFSET = 20000000; // 20ms

	// Should the open and closed sets (graphics only!! - not in the search) be regenerated?
	boolean shouldUpdateOpen = false;
	boolean shouldUpdateClosed = false;

	// The following members are used to determine the depth estimate (d^cheapest) heuristic to use
	private DistanceCalculator distanceCalculator = null;

	// r_default. Used before conExpansionIntervals has settled.
	// This is the number of expansions to perform before the sliding window is deemed 'settled'
	final private int SETTLING_EXPANSION_COUNT = 10;

	// A count limit for settling that gets reset every time we deprune nodes
	// This number needs to be reached to indicate that we are settled and have
	// a "reliable" performance estimate
	private int expansionCountForSettling = SETTLING_EXPANSION_COUNT;

	// This is the size of the sliding window, in entries.
	final private int EXPANSION_DELAY_WINDOW_LENGTH = 10;

	// Sliding windows to calculate average single step error.
	private SlidingWindow expansionDelayWindow = new SlidingWindow(
			EXPANSION_DELAY_WINDOW_LENGTH);
	private SlidingWindow expansionTimeWindow = new SlidingWindow(
			EXPANSION_DELAY_WINDOW_LENGTH);

	// Time snapshot at the last expansion performed. Used to perform timing estimates.
	long timeAtLastExpansion;

	// Current expansion number (E_curr)
	private int expansionCount = 0;

	// C Timer called via JNI Interface.
	HRTimer timer = new HRTimer();

	// Flag that DAS has found it's first solution. Useful for debugging and for triggering
	// code to execute after first solution found.
	private boolean foundDASSolution = false;

	// Track if the goal has moved
	private GridCell lastGoal = null;

	// The current deadline in nanoseconds
	private long timeDeadline = 0;

	// The time remaining on the previous call to getNextMove from Apparate. In milliseconds.
	private long previousTimeLeft = 0;


	/**
	 * This is the interface function called by Apparate.
	 */
	@Override
	public GridCell getNextMove(GridDomain map, GridCell start, GridCell goal,
			int stepLeft, long stepTime, long timeLeft) {

		try {

			if (distanceCalculator == null) {
				distanceCalculator = GridUtil.createDistanceCalculator(map);
			}

			/* We need to replan iff
			 *  - we dont have a plan
			 *  - the map has changed
			 *  - the goal has moved
			 *  - the time remaining has been increased
			 *
			 * Note the assumption that a time increase is intended to give us enough
			 * time for an improved plan!
			 * This is not necessarily a valid assumption, but determining whether to
			 * replan is a separate issue!
			 */
			boolean bReplan =
					plan == null ||
					map.getChangedEdges().size() > 0 ||
					!lastGoal.equals(goal) ||
					timeLeft > previousTimeLeft;


			if (bReplan)
			{

				// TODO: base search buffer on the length of the solution. This is a whole other issue!
				previousTimeLeft = timeLeft;
				long timeCurrent = timer.getCurrentNanotime();
				long searchTime = (long) ((timeLeft * MS_TO_NS_CONV_FACT) - SEARCH_END_TIME_OFFSET);

				// Initialise the deadline, which is the time by which we must return a solution
				timeDeadline = timeCurrent + searchTime;

				// a new plan has been generated, update open and closed debug sets.
				shouldUpdateOpen = true;
				shouldUpdateClosed = true;

				plan = generatePlan(map, start, goal);

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
			// Catch all exceptions before the propagation into Apparate.
			e.printStackTrace();
			return start;
		}
	}

	/*
	 * Algorithm description
	 * This follows the paper of Dionne, Taylor, Ruml [2011] - Deadline-Aware Search
	 * 1)    Initialise Open with starting state
	 * 2)    Initialise Pruned with empty structure
	 * 3)    Initialise Incumbent plan with NULL
	 * 4)    while (current time < deadline)
	 *       {
	 * 5)        if Open is not empty
	 *           {
	 * 6)             max_reachable_depth = calculate_d_bound()
	 * 7)             state s = open.pop()
	 * 8)             if goal(s) && s > incumbent
	 *                {
	 * 9)                 incumbent = s
	 *                }
	 * 10)            else if cheapest_solution_depth < max_reachable_depth
	 *                {
	 * 11)                s' = for each child of s
	 *                    {
	 * 12)                    open.push(s')
	 *                    }
	 *                }
	 * 13)            else
	 *                {
	 * 14)                pruned.push(s)
	 *                }
	 *            }
	 * 15)        else
	 *            {
	 * 16)            recover_pruned_states(open, pruned)
	 *            }
	 *       }
	 * 17)   return incumbent
	 */
	private ComputedPlan generatePlan(GridDomain map, GridCell start,
			GridCell goal)
	{
		//mapInfo = new DasMapInfo(map);

		// TODO: Investigate potential performance improvement by moving this construction outside of the
		// application execution
		mapInfo = new FastDasMapInfo(map);

		// Construct an initial greedy plan
		ComputedPlan incumbentPlan = null;
		incumbentPlan = speedierSearch(map, start,goal);

		// Initialize open set with start node.
		int hCost = (int)map.hCost(start, goal);
		int dCost = distanceCalculator.dCost(start, goal);
		mapInfo.addStartCell(start, hCost, dCost);

		timeAtLastExpansion = timer.getCurrentNanotime();

		// Arbitrary initialisations! Should have no effect on behaviour.
		float dCheapestWithError = 1;
		int dMax = 2000;

		/* Here is our main DAS application loop
		 * We run until our time has expired
		 */
		while (timeDeadline - timer.getCurrentNanotime() > 0)
		{
			if (!mapInfo.isOpenEmpty())
			{
				/* There are nodes in the open set! Grab the one with the lowest f(n).
				 * current is now the current node being examined
				 */

				GridCell current = mapInfo.closeCheapestOpen();


				// GS: comment out this code so that DAS solutions can be visualised!

				if (incumbentPlan != null
						&& mapInfo.getGCost(current) > incumbentPlan.getCost())
				{
					// If this node has a higher g cost than the incumbent plan, discard it.
					continue;
				}

				// Check if we have finished settling, and can calculate dmax
				if (expansionCount > expansionCountForSettling)
				{
					dCheapestWithError = mapInfo.getDCheapestWithError(current);
					dMax = calculateMaxReachableDepth();
				}

				// If the current state is a goal state
				if (current == goal)
				{
					System.out.println("DAS Found path to goal! cost = " + mapInfo.getGCost(current));

					 /* First, check if this is the first solution that DAS has found.
					  * This can be useful for several 'tricks' not included in the DAS
					  * Algorithm including switching how the pruned set is sorted.
					  * We switch from a h(n) sort, to f(n) sort
					  */
					if (!foundDASSolution)
					{
						// If this is the first time DAS has found a solution, we should switch
						// the pruned list to sort by f(n) instead of h(n) to converge on optimal
						mapInfo.NotifySolutionFound();
						foundDASSolution = true;
					}

					// If this is an improved solution, compute the path, ready to give it back to Apparate
					if ( incumbentPlan == null ||
							mapInfo.getGCost(current) < incumbentPlan.getCost())
					{
						incumbentPlan = mapInfo.computePlan(goal);
					}
				}
				else if ( (expansionCount <= expansionCountForSettling) ||
						(dCheapestWithError <= dMax)) // <?
				{
					/* We have deemed that the expected length of the cheapest solution (d^cheapest)
					 * for the current node is within our performance bounds (d_max)
					 */

					// Generate all neighboring cells.
					SuccessorIterator neighborIter = map.getNextSuccessor(current);
					GridCell neighbor;

					while ((neighbor = neighborIter.next()) != null)
					{
						//System.out.println(neighbor);
						generateCell(map, goal, current, neighbor);
					}

					/* Here, we calculate the expansion delay, which is our vacillation evaluation
					 * The expansion delay is equal to the current expansion number (e_curr), minus
					 * that which was appended to the current node when it was generated.
					 * A low number means that the search is not vacillating
					 * A high number means that the search is vacillating
					 *
					 * This is the first part of our performance estimate
					 */

					int expansionDelay = expansionCount - mapInfo.getExpansionNumber(current);

					// Insert expansion delay into sliding window.
					expansionDelayWindow.push(expansionDelay);


					/* Next we calculate our delta time between expansions and insert the delta into
					 * the sliding window for an average time delta.
					 *
					 * This is the second part of our performance estimate
					 */

					long timeCurrent = 	timer.getCurrentNanotime();
					long expansionTimeDelta = timeCurrent - timeAtLastExpansion;

					// Insert time delta into sliding window.
					expansionTimeWindow.push(expansionTimeDelta);

					timeAtLastExpansion = timeCurrent;

					// Increment number of expansions.
					expansionCount++;
				}
				else /* expansionCount > settlingCount && dCheapest > dMax */
				{

					/* The goal is deemed unreachable from this node, if we pursue our current
					 * search behaviour.
					 *
					 * Adjust our search behaviour by pruning this node
					 */
					mapInfo.pruneCell(current);

				}
			}
			else
			{
				// Open list is empty, so we need to repopulate it.
				if (!mapInfo.isPrunedEmpty())
				{
					expansionCountForSettling = SETTLING_EXPANSION_COUNT + expansionCount;
					int exp = calculateExpansionsRemaining();
					mapInfo.recoverPrunedStates(exp);

					// Reset our performance metrics
					expansionDelayWindow.reset();
					expansionTimeWindow.reset();
				}
				else
				{
					// Search space has been exhausted
					System.out.println("Pruned and open are empty");
					break;
				}
			}
		}


		/**
		 *  This is where we make a hybrid speedier/DAS plan!
		 *  By removing the foundDASSolution check, we can also hybridise subsequent DAS search trees!!
		 *  The main intent of this is when DAS has not returned a good quality solution, but a previous
		 *  search may hold the key to an improved solution
		 */
		if (/*!foundDASSolution && */ incumbentPlan != null)
		{
			ComputedPlan pathNew = new ComputedPlan();
			int pathCost = 0;
			int countGreedy = 0;
			int countDAS = 0;

			// Used to track the cost between start point and current node
			boolean DASPathToNodeIsCheaper = false;

			// Combined the DAS partial plan with the greedy solution
			for (int iterSteps = incumbentPlan.getLength()-1;
					iterSteps >= 0 ; iterSteps--)
			{
				// Step backwards through each step of the greedy search that found the goal
				GridCell cell = (GridCell) incumbentPlan.getStep(iterSteps);
				pathNew.prependStep(cell);
				pathCost += cell.getCellCost();
				countGreedy++;
				if (mapInfo.cellExists(cell))
				{
					// We have hooked up with the DAS partial solution!
					// Get the upstream from the DAS mapInfo IFF it is cheaper from this point
					DASPathToNodeIsCheaper = mapInfo.getGCost(cell) + pathCost
							< incumbentPlan.getCost();
					if (DASPathToNodeIsCheaper)
					{
						while (cell != null)
						{
							pathCost += cell.getCellCost();
							pathNew.prependStep(cell);
							cell = mapInfo.getParent(cell);

							countDAS++;
						}

						pathNew.setCost(pathCost);
						System.out.println("pathNew cost: " + pathCost +
								" incumbentPlan: " + incumbentPlan.getCost());

						System.out.println("Hybrid solution found! Greedy nodes: "
						+ countGreedy + " DAS nodes: " + countDAS + " Cost: " + pathCost);
						incumbentPlan = pathNew;

						break;
					}

				}
			}
			return incumbentPlan;
		}
		else
		{
			// Return null - no solution was found
			return incumbentPlan;
		}
	}

	/**
	 * This function is to generate a nominated node, and put it on the open set
	 * We initialise the appropriate values here
	 * @param map
	 * @param goal
	 * @param parent
	 * @param cell
	 */
	private void generateCell(GridDomain map, GridCell goal, GridCell parent, GridCell cell)
	{
			// consider node if it can be entered
			if (map.isBlocked(cell) == false)
			{
				// Set the G cost equal to the G cost of a nodes parent + the transit cost of this node
				int gCost = mapInfo.getGCost(parent) + (int)map.cost(parent, cell);

				// Set the H cost to the Apparate provided H estimate
				int hCost = (int)map.hCost(cell, goal);

				// d_cheapest cannot be assumed to be the same as h..
				int dCheapestRaw = distanceCalculator.dCost(cell, goal);

				// If we already have this cell in our open, closed, or pruned list, ignore it...
				if (!mapInfo.cellExists(cell))
				{
					// Node has not been seen before, add it to the open set.
					mapInfo.add(cell, gCost, hCost, dCheapestRaw, expansionCount, parent);
				}
				// ... Unless it is a new and improved path to an existing cell!
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
	public int calculateMaxReachableDepth()
	{
		double avgExpansionDelay = expansionDelayWindow.getAvg();

		int exp = calculateExpansionsRemaining();
		int dMax = (int) (exp / avgExpansionDelay);

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
	public int calculateExpansionsRemaining()
	{
//		expansionTimeWindow.printAll();
		float averageExpTime = expansionTimeWindow.getAvg();

		int exp = (int) ( (timeDeadline - timer.getCurrentNanotime()) / averageExpTime);

		return exp;
	}

	/**
	 * This is a greedy search to quickly rush to get an incumbent solution
	 * It's intent is to be as fast as possible, with no thought to the quality of the solution
	 * @param map
	 * @param start
	 * @param goal
	 * @return
	 */
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

		return incumbentPlan;
	}

	/**
	 * The following functions are purely for Apparate debug output
	 */

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

}

