package agents;

public class HRTimer 
{
	private native void print();
	
	public native long getCurrentNanotime();

	public static void main(String[] args)
	{
		HRTimer timer = new HRTimer();
		timer.print();
		timer.print();
		timer.print();
		timer.print();
		timer.print();
		System.out.println(timer.getCurrentNanotime());
		System.out.println(timer.getCurrentNanotime());
		System.out.println(timer.getCurrentNanotime());
		System.out.println(timer.getCurrentNanotime());
		System.out.println(timer.getCurrentNanotime());
		System.out.println(timer.getCurrentNanotime());
		timer.print();
		timer.print();
		timer.print();
		timer.print();
		timer.print();
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
