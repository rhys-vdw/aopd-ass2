package agents;
import java.lang.management.ManagementFactory;
import java.lang.management.ThreadMXBean;
import java.lang.Math;

public class HRTimer 
{
	private native void print();
	
	public native long getCurrentNanotime();
	final static ThreadMXBean threadMX = ManagementFactory.getThreadMXBean();
	public static void main(String[] args)
	{
		assert threadMX.isCurrentThreadCpuTimeSupported();
		threadMX.setThreadCpuTimeEnabled(true);
		HRTimer timer = new HRTimer();
		
		long conDiffsMX[] = new long[1000];
		long conDiffsC[] = new long[1000];

		for (int n = 0; n < 1000; n++)
		{
			conDiffsC[n] = Math.abs(timer.getCurrentNanotime() - timer.getCurrentNanotime());
			conDiffsMX[n] = Math.abs(threadMX.getCurrentThreadCpuTime() - threadMX.getCurrentThreadCpuTime());
			
		}
		
		long totalMX = 0 ;
		long totalC = 0;
		
		System.out.println("Average difference between two calls of timing function of 1000 runs");
		for (int n = 0; n < 1000; n++)
		{
			totalMX += conDiffsMX[n];
			totalC += conDiffsC[n];
			// Tried around both ways - makes no difference.. C is faster :)
			//System.out.println("C: " + conDiffsC[n]);
			//System.out.println("MX: " + conDiffsMX[n]);
			
		}
		System.out.println("C: " + totalC/1000);
		System.out.println("ThreadMX: " + totalMX/1000);
	}

	static
	{
		try
		{
			System.loadLibrary("HRTimer");
		}
		catch (Throwable e)
		{
			System.out.println("native lib not found HRTimer " + System.getProperty("java.library.path"));
			//throw e;
		}
		
	}
}
