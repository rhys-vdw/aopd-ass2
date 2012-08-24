package agents;
import java.util.ArrayList;
import java.util.Random;

import au.rmit.ract.planning.pathplanning.entity.ComputedPlan;
import au.rmit.ract.planning.pathplanning.entity.State;
import au.rmit.ract.planning.pathplanning.entity.Plan;
import au.rmit.ract.planning.pathplanning.entity.SearchNode;
import pplanning.interfaces.PlanningAgent;
import pplanning.simviewer.model.GridCell;
import pplanning.simviewer.model.GridDomain;


public class MyCoolAgent implements PlanningAgent {

	// You need to re-implement this method with your own strategy
	@Override
	public GridCell getNextMove(GridDomain map, GridCell sState, GridCell gState, int stepLeft,
			long stepTime, long timeLeft) {

		// Take a random step to any place around (could even be blocked! like a tree for example)
		Random random = new Random();		
		GridCell nextCell = (GridCell) map.getPredecessors(sState).get(random.nextInt(map.getPredecessors(sState).size()));
		
		System.out.println("I'll try to move randomly to "+ nextCell.toString() + " (Time left: " + timeLeft + "ms; Steps left: " + stepLeft);

		return nextCell;
	}

	
	
	
	
	
	// You can if you want implement the methods below

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
