package agents;
import java.util.ArrayList;
import java.lang.management.ManagementFactory;
import java.lang.management.ThreadMXBean;

import au.rmit.ract.planning.pathplanning.entity.ComputedPlan;
import au.rmit.ract.planning.pathplanning.entity.State;
import au.rmit.ract.planning.pathplanning.entity.Plan;
import au.rmit.ract.planning.pathplanning.entity.SearchDomain;
import pplanning.interfaces.PlanningAgent;
import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridDomain;

public class RestartingWeightedAStar implements PlanningAgent {
	// plan to execute
	private ComputedPlan plan;

	RwaMapInfo mapInfo;

	private int stepNo = 0;

	// For timing.
	final ThreadMXBean threadMX = ManagementFactory.getThreadMXBean();

	private final float INITIAL_H_WEIGHT = 5.0f;
	private final float DECAY_RATE = 0.6f;

	private final long MS_TO_NS = 1000000;
	
	final private long SEARCH_END_TIME_OFFSET = 20000000;

	@Override
	public GridCell getNextMove(GridDomain map, GridCell start, GridCell goal,
			int stepLeft, long stepTime, long timeLeft) {

		try {
			Trace.Enable(true);
			GridCell nextStep = null;

			if (plan == null) {
				long currentTime = threadMX.getCurrentThreadCpuTime();
				long searchTime = timeLeft * MS_TO_NS - SEARCH_END_TIME_OFFSET;

				plan = generatePlan(map, start, goal, currentTime + searchTime);
				stepNo = 0;
			}

			if (plan == null) {
				System.out.println("Could not find goal");
				return start;
			}

			if (stepNo < plan.getLength()) {
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

	private ComputedPlan generatePlan(GridDomain map, GridCell start,
			GridCell goal, long deadlineTime) {

		// Initialize timer stuff.
		assert threadMX.isCurrentThreadCpuTimeSupported();
		threadMX.setThreadCpuTimeEnabled(true);

		// Incumbent plan.
		ComputedPlan solution = null;

		// Incumbent plan length.
		float bound = Float.POSITIVE_INFINITY;

		mapInfo = new RwaMapInfo(map, INITIAL_H_WEIGHT);

		// Initialize open set with start node.
		mapInfo.add(start, 0f, map.hCost(start, goal), null);

		// Repeat while there are states in open set and time left.
		while (mapInfo.isOpenEmpty() == false &&
				threadMX.getCurrentThreadCpuTime() < deadlineTime) {

			GridCell current = mapInfo.closeCheapestOpen();

			boolean goalFound = false;

			// Expand node.
			for (State stateIter : map.getSuccessors(current)) {
				GridCell neighbor = (GridCell) stateIter;

				// Only consider a node if it can be traversed.
				if (map.isBlocked(neighbor) == false) {

					// Get cost from start to neighbor (g cost).
					float gCost = mapInfo.getGCost(current) + map.cost(current, neighbor);

					// Ignore if path to node is more expensive than incumbent solution.
					if (gCost > bound) {
						continue;
					}

					// Check if solution has been found.
					if (neighbor == goal) {
						goalFound = true;
					}

					if (mapInfo.cellExists(neighbor) == false) {

						// Node has not previously been encountered, add to open set.
						mapInfo.add(neighbor, gCost, map.hCost(neighbor, goal), current);

					} else if (mapInfo.isSeen(neighbor)) {

						// Node has been seen already, update its g cost.
						if (gCost < mapInfo.getGCost(neighbor)) {
							mapInfo.setGCost(neighbor, gCost);
							mapInfo.setParent(neighbor, current);
						}
						mapInfo.reopenCell(neighbor);

					} else if (gCost < mapInfo.getGCost(neighbor)) {

						// Node is already in open or closed set, but a shorter path to it
						// has been found.
						mapInfo.setGCost(neighbor, gCost);
						mapInfo.setParent(neighbor, current);

						// If cell is not already in open list, add it.
						if (mapInfo.isOpen(neighbor) == false) {
							mapInfo.reopenCell(neighbor);
						}

					}
				}
			}

			if (goalFound) {
				Trace.print("found goal!");

				// Store incumbent.
				solution = mapInfo.computePlan(goal);
				bound = solution.getLength();

				// Clear open and closed sets.
				mapInfo.moveAllToSeen();

				// Update weight.
				mapInfo.setHWeight(Math.max(1.0f, mapInfo.getHWeight() * DECAY_RATE));

				Trace.print("new weight = " + mapInfo.getHWeight());

				// Add start cell back into open set.
				mapInfo.reopenCell(start);
			}
		}

		// Return incumbent solution or null if none has been found.
		return solution;
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
