
package agents;

import java.util.Arrays;

/**
 *
 * @author geoff
 *
 * This class is necessary because java does not have a straight queue container!
 * We want a circular queue, where, if the container is 'full', addition of a new object
 * will cause the oldest item to be removed, and the new item, to be inserted at the
 * beginning.
 *
 * Would have preferred to define the class using generics, but you cant instantiate a generic array in java!
 */
public class SlidingWindow
{

	private int m_size;

	private long[] m_conData;

	private long m_sum;

	private int m_oldest;

	private int m_count;

	SlidingWindow(int _size)
	{
		m_size = _size;
		m_conData = new long[_size];
		m_oldest = 0;
		m_count = 0;
	}

	void push(long _newEntry)
	{
		if (m_count < m_size)
		{
			m_count++;
		}
		else
		{
			long removedEntry = m_conData[m_oldest];
			m_sum -= removedEntry;
			//System.out.print("removing " + removedEntry);
		}
		// Replace the previous oldest entry with the new entry, and update the oldest pointer to the next oldest
		m_conData[m_oldest] = _newEntry;
		if (m_oldest < m_size-1)
		{
			m_oldest++;
		}
		else
		{
			m_oldest = 0;
		}

		// Add the new value to the sum
		m_sum += _newEntry;
		//System.out.print("adding " + _newEntry + " sum = " + m_sum);


	}

	public void printAll()
	{
		for (long n : m_conData)
		{
			System.out.print(n + " ");
		}

	}

	public float getAvg()
	{
		// Should only be called when settled!
		// TODO: probably a better interface than this, but as long as it is remembered to
		// check first!

		return (m_count > 0) ? (float)m_sum / m_count : 0;
	}

	// Used to trigger a "settling" state
	public void reset()
	{
		m_count = 0;
		m_oldest = 0;
		m_sum = 0;
	}

	/**
	 * Test harness to test the sliding function
	 * @param args
	 */
	public static void main(String[] args)
	{
		SlidingWindow sw = new SlidingWindow(10);

		for (long n = 0; n < 1000; n++)
		{
			sw.push(n);
			System.out.print(":Window:");
			sw.printAll();
			System.out.print(" Avg = " + sw.getAvg());
			System.out.println();
		}
	}
}
