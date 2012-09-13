package agents;

public enum CellSetMembership 
{ 
	OPEN   ("open set"), 
	CLOSED ("closed set"),
	PRUNED ("pruned set");

	private String name;

	private CellSetMembership(String name) {
		this.name = name;
	}

	@Override
	public String toString() {
		return name;
	}
}
