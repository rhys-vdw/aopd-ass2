
package agents;

import java.util.Queue;
import java.util.LinkedList;

/**
 * 
 * @author geoff
 *
 * This class is necessary because java does not have a straight queue container!
 * We want a circular queue, where, if the container is 'full', addition of a new object
 * will cause the oldest item to be removed, and the new item, to be inserted at the 
 * beginning.
 * This may not be the quickest or most robust way of doing it at the moment.
 * @param <T>
 */
public class SlidingWindow<T> extends LinkedList<T>
{

	int m_sz;
	
	SlidingWindow(int _sz)
	{
		super();
		m_sz = _sz;	
	}
	
	@Override
	public boolean add(T _obj)
	{
		boolean bRetVal = false;
		super.add(_obj);
		while (size() > m_sz)
		{
			super.remove();
		}
		
		return(true);
	}
	
	
}
