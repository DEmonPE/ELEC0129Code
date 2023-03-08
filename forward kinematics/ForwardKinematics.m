% Joint Lengths

L = [0 9.5 15.5]./100; % measured in m

% Link Lengths a_(n-1) 
a = [0 0 9.5/100]; % measured in m

% Link Twists alpha_(n-1) 

alpha = [0 -90 180]; % measured in degrees

% Link Offsets (d_n) 

d = [0 0 0]; % measured in m

% Joint Angles (theta_n) measured in degrees

theta1 = 0; % variable
theta2 = 0; % variable
theta3 = 0; % variable

% Matrix Transformations

% Transformation from frame {0} to frame {1}

T01=[cosd(theta1) , -sind(theta1) , 0 , a(1);
    cosd(alpha(1))*sind(theta1) , cosd(alpha(1))*cosd(theta1) , -sind(alpha(1)) , -sind(alpha(1))*d(1);
    sind(alpha(1))*sind(theta1) , sind(alpha(1))*cosd(theta1) , cosd(alpha(1)) , cosd(alpha(1))*d(1);
    0 0 0 1];

% Transformation from frame {1} to frame {2}

T12=[cosd(theta2) , -sind(theta2) , 0 , a(2);
    cosd(alpha(2))*sind(theta2) , cosd(alpha(2))*cosd(theta2) , -sind(alpha(2)) , -sind(alpha(2))*d(2);
    sind(alpha(2))*sind(theta2) , sind(alpha(2))*cosd(theta2) , cosd(alpha(2)) , cosd(alpha(2))*d(2);
    0 0 0 1];

% Transformation from frame {2} to frame {3}

T23 =[cosd(theta3) , -sind(theta3) , 0 , a(3);
    cosd(alpha(3))*sind(theta3) , cosd(alpha(3))*cosd(theta3) , -sind(alpha(3)) , -sind(alpha(3))*d(3);
    sind(alpha(3))*sind(theta3) , sind(alpha(3))*cosd(theta3) , cosd(alpha(3)) , cosd(alpha(3))*d(3);
    0 0 0 1];

% Final Transformation from frame {0} to frame {3}

T03= T01*T12*T23;

% Position of end-effector from frame {3}

Pwrt3= [15.5/100 0 0 1]'; % transpose 

% Position and Orientation of end-effector from frame {0}

P0=T03*Pwrt3;

    
    