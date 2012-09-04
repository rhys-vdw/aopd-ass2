
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
 * @param <T>
 */

public class SlidingWindow
{
	
	private int m_sz;

	private Long[] m_conData;
	
	private long m_sum;
	
	SlidingWindow(int _size) 
	{
		m_sz = _size;
		m_conData = new Long[_size];
		
		// Give an initial population - probably should be done by application!
		// Populate with 0's to start with
		Arrays.fill(m_conData, 0l);
	}
	
	void Push(long _newEntry)
	{
		// Maintain the sum, for easy computation of the average.
		// Reduce the sum of all values by the last value in the array
		m_sum -= m_conData[m_sz-1];
		// Add the new value to the sum
		m_sum += _newEntry;
		
		for (int n = m_conData.length -1; n > 0; n--)
		{
			m_conData[n] = m_conData[n-1];
		}
		m_conData[0] = new Long(_newEntry);
	}
	
	public void PrintAll()
	{
		for (Long n : m_conData)
		{
			System.out.print(n + " ");
		}

	}
	
	public double getAvg()
	{
		return(m_sum/m_sz);
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
			sw.Push(n);
			System.out.print(":Window:");
			sw.PrintAll();
			System.out.print(" Avg = " + sw.getAvg());
			System.out.println();
		}
	}
}

// Old implementation
//public class SlidingWindow<T> extends LinkedList<T>
//{
//
//	int m_sz;
//	
//	SlidingWindow(int _sz)
//	{
//		super();
//		m_sz = _sz;	
//	}
//	
//	@Override
//	public boolean add(T _obj)
//	{
//		boolean bRetVal = false;
//		super.add(_obj);
//		while (size() > m_sz)
//		{
//			super.remove();
//		}
//		
//		return(true);
//	}
//	
//	
//}
