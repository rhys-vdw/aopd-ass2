

package agents;

import java.lang.String;


/*
 * Singleton class for printout debug data
 * Should have some options for debug level, ie, to remove traces from performance critical parts
 * but keep traces for non critical sections
 */
final class Trace 
{
	static private boolean m_bEnable = false;
	
	static public void print(String _str)
	{
		if (m_bEnable)
		{
			System.out.println(_str);
		}
	}
	
	// Better way of doing this rather than a copy for each object, primitive, string, etc?
	static public void print(Object _obj)
	{
		if (m_bEnable)
		{
			System.out.println(_obj);
		}
	}

	static public void Enable(boolean _bEnable)
	{
		m_bEnable = _bEnable;
	}
}
