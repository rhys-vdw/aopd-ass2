import pplanning.simviewer.model.GridCell;

interface DistanceCalculator {
  public int dCost(GridCell from, GridCell to);
}
