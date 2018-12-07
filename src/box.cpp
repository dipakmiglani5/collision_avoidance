#include<geometry_msgs/Point.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Odometry.h>
#include<ros/ros.h>
#include<math.h>

geometry_msgs::Point pH, pF, vH, vR, vF;

float r0 = 1.4, k1 = 1;
float b = 0, c = 27, k2;
float modvH;
float modvR;
float lW , l0 = 2;
float e = 1;
void getHummingbird(const geometry_msgs::Pose point)
{
	vH.x = 100*(-pH.x + point.position.x);
	vH.y = 100*(-pH.y + point.position.y);
	vH.z = 100*(-pH.z + point.position.z);
	pH = point.position;
}

void getFirefly(const geometry_msgs::Pose point)
{
	vF.x = 100*(-pF.x + point.position.x);
	vF.y = 100*(-pF.y + point.position.y);
	vF.z = 100*(-pF.z + point.position.z);
    pF = point.position;
}

float dot(geometry_msgs::Point diff3, geometry_msgs::Point vR1){
	return (vR1.x * diff3.x +  vR1.y * diff3.y + vR1.z * diff3.z);
}
float cosy(){
	return (k1 * vH.x / modvH + k2 * vR.x / modvR) / (k1 + k2);
}
float sosy(){
	return (k1 * vH.y / modvH + k2 * vR.y / modvR) / (k1 + k2);
}
// void wosy(float lW[3][2]){
// 	for(int i = 0; i < 3; i++){
// 		for(int j = 0; j < 2; j++){
// 			l[i][j] = ();
// 		}
// 	}
// }
int main(int argc, char **argv)
{
  ros::init(argc,argv,"box");
  ros::NodeHandle nh;

  ros::Subscriber s1=nh.subscribe("/firefly/ground_truth/pose",3,getFirefly );
  ros::Subscriber s2=nh.subscribe("/hummingbird/ground_truth/pose",3,getHummingbird);
  ros::Publisher p = nh.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 1);
  geometry_msgs::Point diff2;
	diff2.x = pF.x - pH.x;
	diff2.y = pF.y - pH.y;
	diff2.z = pF.z - pH.z;
  ros::Rate loopRate(100);
  loopRate.sleep();

  while(ros::ok())
  {
		geometry_msgs::Point diff;
		diff.x = pF.x - pH.x;
		diff.y = pF.y - pH.y;
		diff.z = pF.z - pH.z;

		vR.x = -100*(diff.x-diff2.x);
		vR.y = -100*(diff.y-diff2.y);
		vR.z = -100*(diff.z-diff2.z);

		diff2 = diff;


	    float distancesq =diff.x*diff.x + diff.y*diff.y + diff.z*diff.z + 0.01;
	    float dFH = pow(distancesq, 0.5);
	    float r;


	    k2 = b / (c * dot(diff, vH) + 1);
	    float rH = r0 + k1 * dot(diff, vH) / sqrt(distancesq);
		r = rH + k2 * dot(diff, vR) / sqrt(distancesq);
		modvH = sqrt(vH.x * vH.x + vH.y * vH.y) + 0.00001;
		modvR = sqrt(vR.x * vR.x + vR.y * vR.y) + 0.00001;
		float modDiff = pow (pow(diff.x, 2) + pow(diff.y, 2), 0.5);
		float dOH = sqrt(pH.x*pH.x + pH.y*pH.y + (pH.z-3)*(pH.z-3));

		// lW = l0 + m*dot(vH,)


	    geometry_msgs::PoseStamped coords;
	    coords.header.stamp = ros::Time::now();
	    coords.pose.position.z=3;

	 
		if (dFH < r)
		{  

			if (modvH < 0.001 && abs(vH.z) > 0.001)					//For attacks done along z axis only
			{
				coords.pose.position.x=pF.x + 15*r;
		    	coords.pose.position.y=pF.y;
		    	ROS_INFO_STREAM("move along x axis");
		    	ROS_INFO_STREAM(coords.pose.position);
			}
			else if(modvH  < 0.01 && abs(vH.z) < 0.001) 			//Manual Stationary
			{
				float t1 = pH.x-(pH.x*(rH+0.3))/sqrt(pH.x*pH.x + pH.y*pH.y);
				float t2 = pH.y-(pH.y*(rH+0.3))/sqrt(pH.x*pH.x + pH.y*pH.y);

				if(abs(pF.x - t1) > 0.01 || abs(pF.y - t2) > 0.01){
				                            						//if not diametrically opposite, throw perpendicular to diff
				
				coords.pose.position.x=pF.x-(diff.y*(r0))/modDiff;
		    	coords.pose.position.y=pF.y+(diff.x*(r0))/modDiff;	
		    	}
		    	else if(dOH < r0){										//Moving the automatic diametrically opposite to the manual
		    		
					coords.pose.position.x= t1;  
					coords.pose.position.y= t2;
		    	}
			}
			else													//Manual attacks in plane
			 {  
				

		    	if (-diff.y * vH.x + diff.x * vH.y < 0)
		    	{
		    		coords.pose.position.x = pF.x - (r + 0.3) * diff.y/modDiff;
		    	 	coords.pose.position.y = pF.y + (r + 0.3) * diff.x/modDiff;

		    	 	ROS_INFO_STREAM("first");
		    	}
		    	else 
		    	{		
		    	 	coords.pose.position.x = pF.x + (r + 0.3) * diff.y/modDiff;
		    		coords.pose.position.y = pF.y - (r + 0.3) * diff.x/modDiff;

		    		 	ROS_INFO_STREAM("second");
				}   
			}
			if(abs(pF.x) > l0 - e * abs(vF.x)){
				if(pF.x > 0){
					coords.pose.position.x = l0 - e * abs(vF.x);
				}
				else{
					coords.pose.position.x = -l0 + e * abs(vF.x);
				}
				if(vF.y > 0){
					coords.pose.position.y = l0 - e * vF.y;
				}
				else{
					coords.pose.position.y = -l0 + e * vF.y;	
				}
			}

			if(abs(pF.y) > l0 - e * abs(vF.y)){
				if(pF.y > 0){
					coords.pose.position.y = l0 - e * abs(vF.y);
				}
				else{
					coords.pose.position.y = -l0 + e * abs(vF.y);
				}
				if(vF.x > 0){
					coords.pose.position.x = l0 - e * vF.x;
				}
				else{
					coords.pose.position.x = -l0 + e * vF.x;	
				}
			}

			// if(abs(pF.x) > l0 - e * abs(vF.x) && abs(pF.y) > l0 - e * abs(vF.y)){

			// }

		}
	    else														//Moving the automatic to (0,0,3)
	    {
	    	if(modvH  < 0.01 && abs(vH.z) < 0.01 && dOH < r0){
				coords.pose.position.x=pH.x-(pH.x*(rH+0.3))/sqrt(pH.x*pH.x + pH.y*pH.y);
				coords.pose.position.y=pH.y-(pH.y*(rH+0.3))/sqrt(pH.x*pH.x + pH.y*pH.y);
		    }
			else{
	    	ROS_INFO_STREAM("Go To Origin");
	    	coords.pose.position.x=0;
	    	coords.pose.position.y=0;
			}
	    }
	    



	    coords.pose.orientation.z=0;								//Orientation
	    coords.pose.orientation.x=0;
	    coords.pose.orientation.y=0;
	    coords.pose.orientation.w=0;
	    
	    p.publish(coords);
	    ros::spinOnce();
	    loopRate.sleep();
	}

	return 0;
}