package agents;

public class FloatUtil {
	private final static float EPSILON = 0.001f; // used for floating point comparisons

	/**
	 * Perform an approximate comparison of two floating point values.
	 * @param a value 1
	 * @param b value 2
	 * @return 0, -1 or 1 if a is equal to, less than or greater than b respectively
	 */
	public static int compare(float a, float b) {
		if (Math.abs(a - b) < EPSILON) {
			return 0;
		}
		return (a > b) ? 1 : -1;
	}
}
