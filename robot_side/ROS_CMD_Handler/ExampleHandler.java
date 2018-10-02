package ROS_CMD_Handler;
import java.io.IOException;

import javax.inject.Inject;

import com.kuka.roboticsAPI.applicationModel.IApplicationData;//**


import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.redundancy.IRedundancyCollection;
import com.kuka.generated.flexfellow.FlexFellow;
import com.kuka.generated.ioAccess.FlexFellowIOGroup;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.grippertoolbox.gripper.zimmer.ZimmerR840;

public class ExampleHandler {
	//predefined command property
	private String cmd_str = "cmd_example1";
	private int cmd_type = 1; //0 status ; 1 action
	
	@Inject
	private IApplicationData iapp;
	
	@Inject
	private UDP_Server_Thread server;
	
	@Inject
	private ZimmerR840 gripper;
	
	@Inject
	private FlexFellowIOGroup IOGroupFellow; 
	
	@Inject
	private FlexFellow flexFELLOW_1;
	
	@Inject
	private LBR lbr;
	
	@Inject
	private MediaFlangeIOGroup flangeIOGroup;
	
    private Tool _toolAttachedToLBR;
    
    //initialize codes in the constructor
    public ExampleHandler(IApplicationData _iapp, UDP_Server_Thread _server, LBR _lbr,FlexFellowIOGroup _IOGroupFellow, ZimmerR840 _gripper, FlexFellow _flexFELLOW_1, MediaFlangeIOGroup _flangeIOGroup) throws IOException{
    	// initialize your application here
    	iapp = _iapp;
    	server = _server;
    	lbr = _lbr;
    	gripper = _gripper;
    	IOGroupFellow = _IOGroupFellow;
    	flexFELLOW_1 = _flexFELLOW_1;
    	flangeIOGroup = _flangeIOGroup;
    	
    	_toolAttachedToLBR = iapp.createFromTemplate("ZimmerR840EC02A01");
    	_toolAttachedToLBR.attachTo(lbr.getFlange());
	}
    
    //execution of the commands
    public void run()
    {
    	//report command execution status... to the remote ROS node.
    	server.sendMsg(cmd_str+":"+cmd_type+ ":0:"+"an example to report command status.");
    	
    	//execute your command here.
    	ObjectFrame obj = iapp.getFrame("/wp4_pre_grasp_top");
		_toolAttachedToLBR.getFrame("FingerTip").move(ptp(obj).setJointVelocityRel( 0.1 ));
		
		//report command execution results... to the remote ROS node.
    	server.sendMsg(cmd_str+":"+cmd_type+ ":1:"+"an example to report command results.");
    }
    
}
