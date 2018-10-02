package ROS_CMD_Handler;


import java.net.*;
import java.io.*;

public class UDP_Server_Thread extends Thread {
	private String endTag="@l@";
	protected DatagramSocket socket = null;
	protected String recvStr = new String();
	protected Boolean end = false;
	protected long LastAccess = System.currentTimeMillis();
	byte[] send_data = new byte[1024];
	protected DatagramPacket recvPacket;
	protected InetAddress remoteIp;
	protected int remotePort;
	protected int thisPort;
	public boolean hasReceived = false;
		
	public String getString(){
		String rtStr;
		synchronized(this){
			rtStr = recvStr;//.split(" ");
			//System.out.printf("receive length data:%d\n" , rtStr);
			LastAccess = System.currentTimeMillis();
		}
		return rtStr;
	}
	
	public void sendtxt(String sendStr)
	{
		try{
			sendStr +=endTag;
			send_data = sendStr.getBytes();
			//InetAddress remoteIp= InetAddress.getByName("172.31.1.148");
			
			DatagramPacket sendPacket = new DatagramPacket(send_data , send_data.length , remoteIp , 1200 );
			socket.send(sendPacket);
		}
		catch(IOException e)
		{
			e.printStackTrace();
		}
	}
	
	public boolean sendMsg(String sendStr){
		sendStr +=endTag;
		boolean isSuccess = false;
		synchronized(this){
		send_data = sendStr.getBytes();
		DatagramPacket sendPacket = new DatagramPacket(send_data , send_data.length , remoteIp , remotePort );
		try {
			socket.send(sendPacket);
			//System.out.printf("sendStrwwww=%s\n" ,sendStr);
			isSuccess = true;
		} catch (IOException e) {
			e.printStackTrace();
		}
		}
		return isSuccess;
	}
	
	public UDP_Server_Thread() throws IOException{
		this("UDP_Server_Thread");
		thisPort=30003;
	}
	
	public UDP_Server_Thread(String name) throws IOException{
		super(name);
	  	
	}
	
	public UDP_Server_Thread(int _port) throws IOException{
		thisPort=_port;
	}
	
	public void kill()
	{
		end = true;
	}

	public void dispose() {
		if(socket != null)
		{	
			if(!(socket.isClosed()))
				socket.close();
		}

		System.out.println("Disposing Client");
	} 


	public void run() {
		   byte[] recvBuf = new byte[2000];
		  	try {
		  		socket = new DatagramSocket(thisPort);
		  		socket.setSoTimeout(0);//infinite timeout.
		    } catch (IOException ex) {
		          System.err.println("Can't setup server on this port number. ");
		          System.err.println(ex);
		          
		    }		
		   
		   while( !end ){
			   recvPacket = new DatagramPacket(recvBuf , recvBuf.length);
				try{
					socket.receive(recvPacket);
					
					hasReceived = true;
					//System.out.printf("getLocalPort:%d\n" ,socket.getLocalPort() );
					//System.out.printf("getLocalAddress:%s\n" ,socket.getLocalAddress() );
					
					//System.out.printf("IP:\n" + recvPacket.getAddress().getHostAddress() );
					//System.out.printf("port:%d\n" , recvPacket.getPort() );
					remoteIp=recvPacket.getAddress();
					remotePort=recvPacket.getPort();
				}catch(SocketTimeoutException e){
					System.out.println("Socket Timeout!");
					break;
				}
				catch(Exception e){
					System.out.println("Error! Closing Socket ");
					e.printStackTrace();
					break;
				}

				synchronized(this){
					recvStr = new String(recvPacket.getData() , 0 , recvPacket.getLength());
					//System.out.println("receive string:" + recvStr);
					
					if (System.currentTimeMillis()-LastAccess > 10000){
						break;
					}
				}
		   } // while
		   
			if(socket != null)
			{	
				if(!(socket.isClosed()))
				{
					socket.close();
				}
			}		   
		   System.out.println("Exiting Client");

	} // run 
} // class 