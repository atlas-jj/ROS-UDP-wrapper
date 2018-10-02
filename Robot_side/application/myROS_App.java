package application;


import javax.inject.Inject;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLED;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLEDSize;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.flexfellow.FlexFellow;
import com.kuka.generated.ioAccess.FlexFellowIOGroup;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.grippertoolbox.gripper.zimmer.ZimmerR840;
import static com.kuka.grippertoolbox.gripper.zimmer.ZimmerR840.*;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import ROS_Server.UDP_Server_Thread;
/**
* import your function factories, handlers here
*/
import ROS_CMD_Handler.ExampleHandler;
import ROS_CMD_Handler.Grasp_handler;

/**
 * UDP_Client_ROS wrapper
 * <p>
 * This application runs as a UDP client, it's an interface between iiwa robot and the remote ROS nodes.
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class myROS_App extends RoboticsAPIApplication {
	
	private UDP_Server_Thread server = null; // thread
	private int port = 30002; //port for UDP client
	private byte[] receive_data = new byte[1024];
	private byte[] send_data = new byte[1024]; 
	private String prevRecvStr="";
    private String curRecvStr="";
    private String endTag="@l@";
    private String cmdString="";
    private boolean done=false;
    private boolean _ExeLock = true;
	
	//region 
	@Inject
	private ZimmerR840 gripper;
	@Inject
	private LBR _lbr;
	
	@Inject
	private FlexFellowIOGroup _IOGroupFellow; 
	
	@Inject
	private  MediaFlangeIOGroup _IOGroupflange;
	
	@Inject
	private FlexFellow flexFELLOW_1;
	
	private Tool _toolZimmerGripper;
	//endregion
	@Override
	public void initialize() {
		// initialize your application here
		//region 
		//initialize your tool here. not needed for general UDP wrappers, 
		//may be needed to be passed to your command handler classes.
		_toolZimmerGripper = getApplicationData().createFromTemplate("ZimmerR840EC02A01");
		_toolZimmerGripper.attachTo(_lbr.getFlange());
		//endregion
		
		//region define the side buttons.
		IUserKeyBar socketKeyBar = getApplicationUI().createUserKeyBar("sockect_dispose");
		IUserKeyListener socketCloseListener = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey socketCloseKey , UserKeyEvent event){
				if (event == UserKeyEvent.KeyDown) {
					socketCloseKey.setText(UserKeyAlignment.MiddleLeft, "Close");
					socketCloseKey.setLED(UserKeyAlignment.BottomMiddle, UserKeyLED.Red, UserKeyLEDSize.Small);
					socketCloseKey.setLED(UserKeyAlignment.BottomLeft, UserKeyLED.Grey, UserKeyLEDSize.Small);
					done=true; // exit the while loop >> close the socket
					dispose();

				}
				else
					socketCloseKey.setText(UserKeyAlignment.MiddleLeft, "Open");
					socketCloseKey.setLED(UserKeyAlignment.BottomLeft, UserKeyLED.Green, UserKeyLEDSize.Small);
			}

		};
		
		IUserKey myKey = socketKeyBar.addUserKey(0, socketCloseListener, true);
		myKey.setLED(UserKeyAlignment.TopLeft, UserKeyLED.Green, UserKeyLEDSize.Small);
		socketKeyBar.publish();
		//end region
	}

	@Override 
	public void dispose()
	{
		_ExeLock = false ; 
		getLogger().info(" Closing Sockets in Dispose Block"); 
        if(server.isAlive())
        	server.dispose();
		if(server != null)
			server.kill();

		super.dispose();
	} // dispose
	
	public void processRecv()
	{
		String recvStr = server.getString(); // get data from client 
		if(recvStr!=curRecvStr&&recvStr.length()>0)
		{
		   prevRecvStr=curRecvStr;
		   curRecvStr=recvStr;
		   cmdString+=recvStr;
		   getLogger().info("str received!: "+recvStr);////////////////////////////////////////////////////////////
		}
	}
	
	public enum CMDs{
		default_cmd,
		example_cmd1,
		test_connection,
		grasp;
	}
	
	
	public CMDs queryCMDs(String cmd)
	{
		if(cmdString.contains("cmd_example1"))
			return CMDs.example_cmd1;
		if(cmdString.contains("grasp"))
			return CMDs.grasp;
		if(cmdString.contains("test_conn"))
			return CMDs.test_connection;
		return CMDs.default_cmd;
	}
	
	public void ExecuteCommand(String cmd) throws IOException
	{
		String strBody=cmdString.split(endTag)[0];
		//[cmd_str]:[cmd_type]:[data]
		String cmd_type = strBody.split(":")[1];
		String cmd_data = strBody.split(":")[2];
		
		CMDs myCMD = queryCMDs(cmdString);
		//report command execution status... to the remote ROS node.
    	server.sendMsg("cmd_example:"+cmd_type+ ":0:"+"start");
		switch(myCMD){
		//TODO: your command handler here.
		case example_cmd1:
			ExampleHandler eh = new ExampleHandler(getApplicationData(), server, _lbr, _IOGroupFellow , gripper, flexFELLOW_1, _IOGroupflange);
			eh.run();
			break;
		case grasp:
			//some code for executing the command here
			Grasp_handler gh = new Grasp_handler(getApplicationData(), server, _lbr, _IOGroupFellow , gripper, flexFELLOW_1, _IOGroupflange);
			gh.run();
			break;
		case test_connection:
			server.sendMsg("echo_test:1:0:start");
		default:
			break;
		}
		
		//report command execution status... to the remote ROS node.
		server.sendMsg("cmd_example:"+cmd_type+ ":0:"+"end");
	}
	
	@Override
	public void run() {
		// your application execution starts here
		_IOGroupFellow.setYellowLight(false);
     	_IOGroupflange.setLEDBlue(false);
     	
     	//region initialize robot position.
     	//_lbr.move(ptp(CenterPosition).setMode(modeHard).setJointVelocityRel(0.1));
     	//end region
     	
     	try {
     		server = new UDP_Server_Thread(port);//define the port number
     		server.start(); 
			getLogger().info("UDP server started! Listening port: " + port);
     	}
     	catch (IOException e) {
			
			e.printStackTrace();
		}
		
     	//now run the communication loop.
     	boolean packageEnding=false;
		processRecv();
		//client.sendMsg("initial sending test");
		getLogger().info("enter into while loop." );
		try{
			int hearbeatCount = 0;
			while(!done)
			{	
				processRecv();
				//getLogger().info(curRecvStr);
				if(curRecvStr.equals("esc"))
				{	
				   done=true;
				   getLogger().info("esc received, app will exit." );
				}
		
				if(cmdString.contains(endTag))
					packageEnding=true;
				if(packageEnding)
				{
					Runnable exe_thread = create_exe_cmd(cmdString);
					new Thread(exe_thread).start();
					//ExecuteCommand(cmdString);
					cmdString="";
					packageEnding=false;
				}
				
				//send heart beat
				//if(hearbeatCount/ )
				//Thread.sleep(200);
		        ThreadUtil.milliSleep(50);
			} //end of while loop
		}
		catch (Exception e) {
			
			e.printStackTrace();
		}
		
		server.dispose();
	}
	
	public Runnable create_exe_cmd(final String paramStr)
	{
		Runnable newThread = new Runnable(){
			public void run(){
				try {
					ExecuteCommand(paramStr);
				} catch (IOException e) {
					
					e.printStackTrace();
				}
			}
		};
		return newThread;
	}
}
