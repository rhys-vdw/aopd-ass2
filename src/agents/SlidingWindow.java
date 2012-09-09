
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
	
	private int m_size;

	private long[] m_conData;
	
	private long m_sum;
	
	private int m_oldest;
	
	private int m_count;
	
	// Total number of entries that have been stored in this window.
	private long m_countTotal;
	
	// Total number of entries that need to have passed through this window, before we 
	// start to use the values contained within
	private int m_initialisationThreshold;
	
	// Default average used before the entries have settled. Used in conjunction with m_initialisationThreshold
	private float m_initialAverage;
	
	SlidingWindow(int _size, int _initialisationThreshold, float _initialAverage) 
	{
		m_size = _size;
		m_conData = new long[_size];
		m_oldest = 0;
		m_count = 0;
		m_initialisationThreshold = _initialisationThreshold;
		m_initialAverage = _initialAverage;
		m_countTotal = 0;
	}
	
	void Push(long _newEntry)
	{
		++m_countTotal;
		if (m_count < m_size)
		{
			m_count++;
		}
		else
		{
			long removedEntry = m_conData[m_oldest];
			m_sum -= removedEntry;
			System.out.print("removing " + removedEntry);
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
		System.out.print("adding " + _newEntry + " sum = " + m_sum);


	}
	
	public void PrintAll()
	{
		for (long n : m_conData)
		{
			System.out.print(n + " ");
		}

	}
	
	public float getAvg()
	{
		float fAverage = 0.0f;
		if (m_countTotal < m_initialisationThreshold)
		{
			fAverage = m_initialAverage;
		}
		else
		{
			fAverage = (float)(m_sum/m_count);
			System.out.print("sliding window computed avg: " + m_sum + " / " + m_count + " = " + fAverage);
		}
			
		return(fAverage);
	}
	
	/**
	 * Test harness to test the sliding function
	 * @param args
	 */
	public static void main(String[] args) 
	{
		SlidingWindow sw = new SlidingWindow(10,20,105);
		
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
