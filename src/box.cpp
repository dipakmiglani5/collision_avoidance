/* 
Collision Avoidance,
Team Aerial Robotics, IIT Kanpur

Contributors
Apoorv Goyal
Dipak Miglani
Aman Parekh
Atharv Tripathi
Kshitij Kabeer
*/

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
float l0 = 2;
float e = 1;
float rH;
float modDiff;
float dOH;

/*
pH      - Position of Hummingbird(manual drone)
pF      - Position of Firefly(autonomous drone)
vH      - Velocity of Hummingbird
vF      - Velocity of Firefly
vR      - Velocity of Hummingbird w.r.t Firefly
r0      - Sphere of influence parameter, constant
k1      - Sphere of influence parameter, constant
b       - Sphere of influence parameter, constant
c       - Sphere of influence parameter, constant
k2      - Sphere of influence parameter, dependant on diff and vH
modvH   - Speed of Hummingbird in z=3 plane
modvF   - Speed of Firefly in z=3 plane
l0      - Minimum distance from the wall parameter
e       - Minimum distance from the wall parameter
rH      - Sphere of influence parameter, dependant on diff and vH
modDiff - Modulus of diff (defined below), excluding z component
dOH     - Distance of Hummingbird from fixed point 
*/

void getHummingbird(const geometry_msgs::Pose point)  //Callback function for Subscribing the position of Humminbird
{													  //Also calculates velocity of Hummingbird
	vH.x = 100*(-pH.x + point.position.x);
	vH.y = 100*(-pH.y + point.position.y);
	vH.z = 100*(-pH.z + point.position.z);
	pH = point.position;
}

void getFirefly(const geometry_msgs::Pose point)      //Callback function for subscribing position of Firefly
{													  //Also calculates velocity of Firefly
	vF.x = 100*(-pF.x + point.position.x);
	vF.y = 100*(-pF.y + point.position.y);
	vF.z = 100*(-pF.z + point.position.z);
    pF = point.position;
}

float dot(geometry_msgs::Point diff3, geometry_msgs::Point vR1)
{
	return (vR1.x * diff3.x +  vR1.y * diff3.y + vR1.z * diff3.z); //Function for calculating dot product of two vectors
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"box");
    ros::NodeHandle nh;
    ros::Subscriber s1=nh.subscribe("/firefly/ground_truth/pose",3,getFirefly ); //Subscriber object for firefly's position
    ros::Subscriber s2=nh.subscribe("/hummingbird/ground_truth/pose",3,getHummingbird); //Subscriber object for hummingbird's position
    ros::Publisher p = nh.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 1);//Publisher object for firefly's commands
    geometry_msgs::Point diff2; // Relative coordinates of Hummingbird w.r.t Firefly, 0.01 seconds before
	diff2.x = pH.x - pF.x;
	diff2.y = pH.y - pF.y;
	diff2.z = pH.z - pF.z;
    ros::Rate loopRate(100);
    loopRate.sleep();

	while(ros::ok())
	{
		geometry_msgs::Point diff;
		diff.x = pH.x - pF.x; //Relative coordinates of Hummingbird w.r.t. Firefly, just recieved
		diff.y = pH.y - pF.y;
		diff.z = pH.z - pF.z;

		vR.x = 100*(diff.x-diff2.x);
		vR.y = 100*(diff.y-diff2.y);
		vR.z = 100*(diff.z-diff2.z);

		diff2 = diff;


	    float distancesq =diff.x*diff.x + diff.y*diff.y + diff.z*diff.z + 0.01;		//Distance squared between Firefly and Hummingbird
	    float dFH = pow(distancesq, 0.5);
	    float r;	//Radius of Sphere of Influence


	    k2 = b / (c * dot(diff, vH) + 1);	//Calculation of k2
	    rH = r0 + k1 * dot(diff, vH) / sqrt(distancesq);  	//Calculation of rH
		r = rH + k2 * dot(diff, vR) / sqrt(distancesq);		//Calculation of radius of sphere of influence
		modvH = sqrt(vH.x * vH.x + vH.y * vH.y) + 0.00001; 		//Calculation of vH
		modvR = sqrt(vR.x * vR.x + vR.y * vR.y) + 0.00001;		//Calculation of vR
		modDiff = pow (pow(diff.x, 2) + pow(diff.y, 2), 0.5);		//Calculation of modDiff
		dOH = sqrt(pH.x*pH.x + pH.y*pH.y + (pH.z-3)*(pH.z-3));		//Calculation of dOH


	    geometry_msgs::PoseStamped coords;		//pose commands object that will given to the Firefly
	    coords.header.stamp = ros::Time::now();	
	    coords.pose.position.z=3;

	 
		if (dFH < r)
		{  

			if (modvH < 0.001 && abs(vH.z) > 0.001)					//For attacks done along z axis only
			{
				coords.pose.position.x=pF.x + 15*r;
		    	coords.pose.position.y=pF.y;
			}
			else if(modvH  < 0.01 && abs(vH.z) < 0.001) 			//When Hummingbird is stationary
			{
				float t1 = pH.x-(pH.x*(rH+0.3))/sqrt(pH.x*pH.x + pH.y*pH.y);	 //x-coordinate of point diameterically opposite point of Hummingbird
				float t2 = pH.y-(pH.y*(rH+0.3))/sqrt(pH.x*pH.x + pH.y*pH.y);	 //y-coordinate of point diameterically opposite point of Hummingbird

				if(abs(pF.x - t1) > 0.01 || abs(pF.y - t2) > 0.01)		//If Firefly is not diameterically opposite to stationary Hummingbird which is in the sphere of influence of the Firefly, commands firefly to go perpendicular to diff
				{                            						
					coords.pose.position.x=pF.x-(diff.y*(r0))/modDiff;
		    		coords.pose.position.y=pF.y+(diff.x*(r0))/modDiff;	
		    	}

		    	else if(dOH < r0)										//If Hummingbird is not in our sphere of influence, but near the fixed point, then move Firefly diametrically opposite to the Hummingbird
		    	{	
					coords.pose.position.x= t1;  
					coords.pose.position.y= t2;
		    	}

			}
			else													//When Hummingbird attacks in plane z=3
			{  														//The if else statements are to determine which perpendicular direction, firefly should move.
				
		    	if (-diff.y * vH.x + diff.x * vH.y < 0)
		    	{
		    		coords.pose.position.x = pF.x - (r + 0.3) * diff.y/modDiff;
		    	 	coords.pose.position.y = pF.y + (r + 0.3) * diff.x/modDiff;
		    	}

		    	else 
		    	{		
		    	 	coords.pose.position.x = pF.x + (r + 0.3) * diff.y/modDiff;
		    		coords.pose.position.y = pF.y - (r + 0.3) * diff.x/modDiff;
				}

			}
			if(abs(pF.x) > l0 - e * abs(vF.x))		//If the Firefly is going out of the box in x direction
			{	
				
				if(pF.x > 0)	//If Firefly is going out of the square in positive x direction
				{
					coords.pose.position.x = l0 - e * abs(vF.x);
				}

				else
				{
					coords.pose.position.x = -l0 + e * abs(vF.x);	//If Firefly is going out of the square in negative x direction 
				}

				if(vF.y > 0)
				{
					coords.pose.position.y = l0 - e * vF.y;		//If Firefly is going in positive y direction when it encounters square edge in x direction, go in this direction
				}

				else
				{
					coords.pose.position.y = -l0 + e * vF.y;	//If Firefly is going in negative y direction when it encounters square edge in x direction, go in this direction
				}

			}

			if(abs(pF.y) > l0 - e * abs(vF.y))		//If the Firefly is going out of the box in y direction
			{
				
				if(pF.y > 0)	//If Firefly is going out of the square in positive y direction
				{
					coords.pose.position.y = l0 - e * abs(vF.y);
				}

				else		//If Firefly is going out of the square in negative y direction
				{
					coords.pose.position.y = -l0 + e * abs(vF.y);
				}
				
				if(vF.x > 0)		//If Firefly is going in positive x direction when it encounters square edge in y direction, go in this direction
				{
					coords.pose.position.x = l0 - e * vF.x;
				}
				
				else		//If Firefly is going in negative x direction when it encounters square edge in y direction, go in this direction
				{
					coords.pose.position.x = -l0 + e * vF.x;	
				}
			
			}

		}
	    
	    else														//If none of the above conditions match
	    {
	    	
	    	if(modvH  < 0.01 && abs(vH.z) < 0.01 && dOH < r0)		//This block is to prevent oscillation of Firefly when Hummingbird is stationary near the fixed point 
	    	{
				coords.pose.position.x=pH.x-(pH.x*(rH+0.3))/sqrt(pH.x*pH.x + pH.y*pH.y);
				coords.pose.position.y=pH.y-(pH.y*(rH+0.3))/sqrt(pH.x*pH.x + pH.y*pH.y);
		    }

			else		//Go back to the fixed point
			{
	    	coords.pose.position.x=0;
	    	coords.pose.position.y=0;
			}

	    }
	    
	    coords.pose.orientation.z=0;								//Orientation of Firefly
	    coords.pose.orientation.x=0;
	    coords.pose.orientation.y=0;
	    coords.pose.orientation.w=0;
	    
	    p.publish(coords);		//Publishing coords, the variable containing the pose data of the command we wish to give to Firefly
	    ros::spinOnce();
	    loopRate.sleep();
	}

	return 0;
}
