clear

% Define position of end-effector (coordinates of 0P)

x = 0; % x-coordinate
y = 0; % y-coordinate
z = 0; % z-coordinate

% Define joint lengths 

L1 = 0; % units of m
L2 = 0.095; % units of m
L3 = 0.155; % units of m

% Define expressions for cos(theta3) and sin(theta3)
c3 = (x^2 + y^2 + z^2 - L3^2 - L2^2) / (2 * L2 * L3);
s3 = sqrt(1 - c3^2);

% Calculate joint angle 3
theta3 = atan2(round(s3, 12), round(c3, 12));

% Construct a triangle with known parameters
K1 = L2 + L3 * c3; % adjacent of triangle
K2 = L3 * s3; % opposite of triangle

r = sqrt(K1^2 + K2^2); % length of hypotenuse
gamma = atan2(K2,K1); % angle between K1 and K2

% Calculate joint angle 2
theta2 = gamma - atan2(round(z/r, 12) ,sqrt(1 - round((z / r)^2, 12) ));

% Calculate joint angle 1
theta1 = atan2(y,x);

% Convert joint angles in degrees (FINAL ANSWERS)
t1 = theta1 * 180 / pi;
t2 = theta2 * 180 / pi;
t3 = theta3 * 180 / pi;

% The joint angles above were compared to results obtained using forward kinematics and proved to be correct 