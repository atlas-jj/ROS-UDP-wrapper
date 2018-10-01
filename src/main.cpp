
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
//#include <opencv2/opencv.hpp>

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <exception>
#include <vector>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <ctime>

//#include <boost/asio/use_future.hpp>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>

#include "math_helper.h"
#include "string_convertor.h"
#include "colormod.h"

#define BUFLEN 272000  //Max length of buffer
int maxPackageSize=1000;
using boost::asio::ip::udp;
enum { max_length = 1024 };

//using namespace cv;
using namespace std;
Color::Modifier c_red(Color::FG_RED);
Color::Modifier c_yellow(Color::FG_YELLOW);
Color::Modifier c_green(Color::FG_GREEN);
Color::Modifier c_default(Color::FG_DEFAULT);

ros::Publisher pubP;
ros::Publisher pubCommand;
boost::asio::io_service io_service;
udp::socket s(io_service, udp::endpoint(udp::v4(), 0));

int data_size=0;
string endTag="@l@";

udp::resolver resolver(io_service);
udp::resolver::iterator iterator4 ;

bool connected = false;
time_t lastConnected =  time(0);
double reConnDetectCycle = 20; //detect connection status and decide reConnection threshold.

bool packageValid(string inputStr)
{
   if(inputStr.find(endTag) != std::string::npos)
     return true;
   else
     return false;
}

void sendStr(string sendback_cmd)
{
  sendback_cmd +=endTag;
  try {
    //get robot position x,y,z
    int myArrayLength=sendback_cmd.size();
    char myArray[myArrayLength];//as 1 char space for null is also required
    strcpy(myArray, sendback_cmd.c_str());

    boost::asio::socket_base::send_buffer_size option(BUFLEN);//BUFLEN
    if(myArrayLength*8*8>=212992) //3328
           cout<<"warning: data length beyond UDP datagram."<<endl;
    //if data length is larger the UDP datagram size, split it into several packages.
    int packageNum=myArrayLength/maxPackageSize;
    if(packageNum==0)
      s.send_to(boost::asio::buffer(myArray, myArrayLength), *iterator4);
    else
      {
        for(int i=0;i<packageNum+1;i++)
        {
            //get different package
          int thisPackageSize=maxPackageSize;
          if(i==packageNum)
              thisPackageSize=myArrayLength-maxPackageSize*packageNum;
          char toSendArray[thisPackageSize];
          int startIndex=0+i*maxPackageSize;
          for(int m=0;m<thisPackageSize;m++)
              toSendArray[m]=myArray[startIndex+m];
          s.send_to(boost::asio::buffer(toSendArray, thisPackageSize), *iterator4);
          //cout<<toSendArray<<'\0'<<endl;
          boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
      }
    std::cout << "sent txt:  "<<sendback_cmd<<endl;
  }
  catch(exception &e) {
   std::cout << "Catch an exception: " << e.what() << std::endl;
 }
}

void commandsCallback(const std_msgs::String::ConstPtr& msg)
{
  //send the command to remote robot
  cout<<"recved: "<<endl;//<<msg->data<<endl;
  if(msg->data!="")
  {
    sendStr(msg->data);
    //clear rr/commands
    std_msgs::String msg;
    msg.data = "";
    pubCommand.publish(msg);//clear rr/commands
  }
}


string recvr_txt()
{
  boost::this_thread::sleep(boost::posix_time::milliseconds(10));
  char in_reply[max_length];
  udp::endpoint sender_endpoint;
  // udp::endpoint listen_endpoint(
  //       boost::asio::ip::address::from_string(UDP_SERVER_IP),
  //       std::atoi(PORT));
  s.non_blocking(true);
  size_t reply_length = 0;
  try{
    reply_length = s.receive_from(boost::asio::buffer(in_reply, max_length), sender_endpoint);
  }
  catch(exception &e) {
    //std::cout << "Catch an exception: " << e.what() << std::endl;
  }

  if(reply_length>0)
  {
     char readfrom[reply_length];
     for(int i=0;i<reply_length;i++)
     readfrom[i]=in_reply[i];
     string inStr(readfrom);
     cout<<"received:"<<inStr<<endl;
     return inStr;
  }
  else
    return "";
}

void recvr_response(ros::NodeHandle nh)
{
  string inStr = recvr_txt();
  //std::cout << "msg received:  "+inStr<<endl;
  //check connection status
  if(inStr.find("echo_test") != std::string::npos) //confirm connection established.
    setConnected();
  if (packageValid(inStr))//
  {
    try{
      int endIndex=inStr.find(endTag);
      string strTemp=inStr.substr(0,endIndex);
      //split string to doubles
      //cout<<strTemp<<endl;
      cout<<"strTemp:"<<strTemp<<endl;
      std::vector<string> strArray =string_convertor::split(strTemp, ':');
      string cmd_name = strArray[0];
      cout<<"cmd_name:"<<cmd_name<<endl;
      pubP = nh.advertise<std_msgs::String>("/rr/response/"+cmd_name, 1, true);//task will be only published once
      std_msgs::String msg;
      msg.data = strTemp;
      pubP.publish(msg);
    }
    catch(exception &e) {
      std::cout << "Catch an exception: " << e.what() << std::endl;
    }
  }

}

void checkConnection()
{
  double diffSeconds = difftime(time(0), lastConnected);
  if(diffSeconds>=reConnDetectCycle)
    Connect();
}

void setConnected()
{
  connected = true;
  lastConnected = time(0);
}

void Connect()
{
  connected = false;
  iterator4 = resolver.resolve(query);
  cout << "UDP client is trying to connect to ...."<< UDP_SERVER_IP<<":"<<PORT<< endl;
  //waiting for remote connection

  while(!connected)
  {
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    sendStr("test_conn:1:0");
    if(recvr_txt().find("echo_test") != std::string::npos) //confirm connection established.
      setConnected();
  }
  cout << "Connection established! ...."<< UDP_SERVER_IP<<":"<<PORT<< endl;
}

//===========================MAIN FUNCTION START===========================

int main(int argc, char* argv[]){
  string UDP_SERVER_IP="172.31.1.147";//"172.31.1.147"//define the udp server ip address  //"127.0.0.1" "172.31.1.147"/
  string PORT="30002"; //The port on which to listen for incoming data
  if (argc >2) {
        UDP_SERVER_IP=argv[1];
        PORT=argv[2];
  }
  udp::resolver::query query(udp::v4(), UDP_SERVER_IP , PORT);

  Connect();

  ros::init(argc, argv, "wrapper");
  ros::NodeHandle nh;

  ros::Subscriber subP = nh.subscribe("/rr/commands", 1, commandsCallback);
  //ros::Subscriber subP = nh.subscribe("/target_bbox", 1, commandsCallback);
  //pubP = nh.advertise<std_msgs::String>("/rr/response", 1, true);//task will be only published once
  pubCommand = nh.advertise<std_msgs::String>("/rr/commands", 1, true);
  ros::Rate loop_rate(3);//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //bool done=false;
  while(ros::ok())
  {
    ros::spinOnce();
    checkConnection();
    recvr_response(nh);
   }

  ros::shutdown();
  return 0;
}
