package agents;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

import au.rmit.ract.planning.pathplanning.entity.ComputedPlan;
import au.rmit.ract.planning.pathplanning.entity.State;
import au.rmit.ract.planning.pathplanning.entity.SearchNode;
import pplanning.interfaces.PlanningAgent;
import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridDomain;

public class BenchmarkAgent implements PlanningAgent {

	boolean hasRun = false;
	int run = 1;

	int testSize = 100000;

	float[] hCosts = new float[testSize];
	int[] hCostsI = new int[testSize];


	@Override
	public GridCell getNextMove(GridDomain map, GridCell sState, GridCell gState, int stepLeft,
			long stepTime, long timeLeft) {

		//if (hasRun) return sState;

		GridCell cell;

		System.out.println("Run " + run);

		long start = System.nanoTime();
		for (int i = 0; i < testSize; i++) {
			hCostsI[i] = i;
		}
		System.out.println("bshit - test ended after " + (System.nanoTime() - start) + " ns");

		start = System.nanoTime();
		for (int i = 0; i < testSize; i++) {
			cell = (GridCell) map.getSuccessors(sState).get(0);
			hCostsI[i] = dCostManhattan(sState, cell);
		}
		System.out.println("dCost - test ended after " + (System.nanoTime() - start) + " ns");

		start = System.nanoTime();
		for (int i = 0; i < testSize; i++) {
			cell = (GridCell) map.getSuccessors(sState).get(0);
			hCosts[i] = map.hCost(sState, cell);
		}
		System.out.println("hCost - test ended after " + (System.nanoTime() - start) + " ns");

		start = System.nanoTime();
		for (int i = 0; i < testSize; i++) {
			cell = (GridCell) map.getSuccessors(sState).get(0);
			hCostsI[i] = dCostManhattan(sState, cell);
		}
		System.out.println("dCost - test ended after " + (System.nanoTime() - start) + " ns");

		run++;
		return sState;
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

			// Do we want to show extra info? (e.g., close and open nodes, current path)
			@Override
			public Boolean showInfo() {
				// TODO Auto-generated method stub
				return false;
			}

			@Override
			public ArrayList<GridCell> expandedNodes() {
				// TODO Auto-generated method stub
				return null;
			}

			@Override
			public ArrayList<GridCell> unexpandedNodes() {
				// TODO Auto-generated method stub
				return null;
			}


			@Override
			public ComputedPlan getPath() {
				// TODO Auto-generated method stub
				return null;
			}

}
