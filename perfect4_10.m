clc;
clear;
close;
%% Robot Construction
endEffector = "body7";
base='base';
radians=pi/180;
degrees=180/pi;
%DH model table
%            a     alpha    d          theta   
dhparams = [0   	-pi/2	0.14787   	0;
            0.42241	 0      0.1367     0;
            0.39335	 0     -0.1312      0;
            0   	 pi/2	0.1267	    0;
            0       -pi/2	0           0;
            0        0      0           0];
%Create a rigid body tree. This tree is initialized with a base coordinate frame to attach bodies to.
BaseTransform=trvec2tform([0, 0, 0]);
ToolTransform=trvec2tform([0, -0.0997, 0.0996]);

myrobot = buildRobot(dhparams,BaseTransform,ToolTransform);
%% Show robot (fake robot using collision)

myrobot=addRobotBody(myrobot);
axis([-0.5,1.5,-0.5,1.5,-0.5,1.5])
view([230 20])
%%
% show(myrobot);
% axis([-0.5,1.5,-0.5,1.5,-0.5,0.5])
% view([230 20])
%% Joint space

jnt=homeConfiguration(myrobot)
showRobot(myrobot, [0 0 0 0 0 0])
axis([-0.5,1.5,-0.5,1.5,-0.5,1.5])
view([230 20])
%% Kinematics

%Get home configuration of robot
JointHome=homeConfiguration(myrobot);
PosiHome=getTransform(myrobot,JointHome ,'body7', 'body0')


joints1=[0 0 0 0 0 0]*radians;
%joints2=[-20 30 -30 40 20 60]*pi/180;
Posi1=fkine(myrobot, joints1, 'body7', 'base')
% Posi1=fkine(myrobot, joints1, 'body7', 'body0')
% Posi1=fkine(myrobot, joints1, 'body6', 'base')
% Posi1=fkine(myrobot, joints1, 'body6', 'body0')

%Posi2=fkine(myrobot,joints2 , 'body7', 'base')
showRobot(myrobot,joints1);
axis([-0.5,1.5,-0.5,1.5,-0.5,1.5])
view([230 20])
%% Robot Inverse Kinematics
%% Analytical IK, only for rrrsss robot

ikJoints=ikine(myrobot, Posi1,'body7', 'base');
ikAngle=ikJoints*180/pi
figure(1)
showRobot(myrobot,ikJoints(1,:));
axis([-0.5,1.5,-0.5,1.5,-0.5,1.5])
% figure;
% numSolutions = size(ikJoints,1);
% for i = 1:size(ikJoints,1)
%     subplot(1,numSolutions,i)
%     showRobot(myrobot,ikJoints(i,:));
% end
%% Numerial IK, for all robot, but slow and wrong in many situations

weights = [0.25 0.25 0.25 1 1 1];
Posi2=Posi1;
[joints, info]=ikine2(myrobot,'body7', weights, Posi2,joints1);
joints*180/pi



fkine(myrobot, joints, 'body7', 'base')


% set up workcells
figure(2)
showRobot(myrobot, [0 0 0 0 0 0])
hold on;
ax = gca;
plane = collisionBox(2,2,0.025);
plane.Pose = trvec2tform([0 0 -0.0125]);
show(plane,'Parent', ax);

topPlate = collisionBox(0.26,0.02,0.5);
topPlate.Pose = trvec2tform([-0.33 -0.01 0.25]);
[~, patchObj] = show(topPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

bottomPlate =  collisionBox(0.26,0.02,0.5);
bottomPlate.Pose = trvec2tform([-0.13 -0.101 0.25]);
[~, patchObj] = show(bottomPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

leftShelf = collisionBox(0.1,0.05,0.05);
leftShelf.Pose = trvec2tform([0.4 0.025 0.025]);
[~, patchObj] = show(leftShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

targetShelf = collisionBox(0.26,0.5,0.02);
targetShelf.Pose = trvec2tform([0 0.45 0.05]);
[~, patchObj] = show(targetShelf,'Parent',ax);
patchObj.FaceColor = [0 1 0];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel1 = collisionCylinder(0.03,0.003);
Wheel1.Pose = trvec2tform([0.3515 0.025 0.0515]);
[~, patchObj] = show(Wheel1,'Parent',ax);
patchObj.FaceColor = [0 0 1];

rightShelf = collisionBox(0.1,0.05,0.05);
rightShelf.Pose = trvec2tform([0.4 0.425 0.025]);
[~, patchObj] = show(rightShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel2 = collisionCylinder(0.03,0.003);
Wheel2.Pose = trvec2tform([0.3515 0.425 0.0515]);
[~, patchObj] = show(Wheel2,'Parent',ax);
patchObj.FaceColor = [0 0 1];

n=7;
rh=(2*pi)/(2*(n-1));
k=9;
hold on;
i=1;
t=linspace(0,2*pi,n);
x=0.42+0.02*cos(t);
y=0.425+0.02*((sqrt(3)/2)^(i-1))*sin(t);
z=[0.05,0.05,0.05,0.05,0.05,0.05,0.05]
plot3(x,y,z,'r','LineWidth',2);axis equal;

%% Robot target
HOME=[1 0 0 0.2;
      0 -1 0 0.2;
      0 0 -1 0.6;
      0 0 0 1];
T11=[1 0 0 -0.330;
    0 -1 0 -0.01;
    0 0 -1 0.5;
    0 0 0 1];
T110=[1 0 0 -0.330;
      0 -1 0 -0.01;
      0 0 -1 0.6;
      0 0 0 1];

T12=[1 0 0 0;
    0 0 -1 0.7;
    0 1 0 0.05;
    0 0 0 1];
T120=[1 0 0 0;
      0 0 -1 0.7;
      0 1 0 0.2;
      0 0 0 1];
T31=[1 0 0 0.42;
     0 -1 0 0.025;
     0 0 -1 0.065;
     0 0 0 1];
T310=[1 0 0 0.42;
      0 -1 0 0.025;
      0 0 -1 0.2;
      0 0 0 1];
T32=[1 0 0 0.08;
     0 -1 0 0.25;
     0 0 -1 0.08;
     0 0 0 1];
T320=[1 0 0 0.08;
      0 -1 0 0.25;
      0 0 -1 0.20;
      0 0 0 1];

% ikJoints2=ikine(myrobot, T12,'body7', 'base');
% ikAngle2=ikJoints2*degrees

%inverse kinematics
% pick 1 of 8 configurations

ikJoints=ikine(myrobot, HOME,endEffector, base);
ikJoints*degrees;
config=2;
ikJointsHome=ikJoints(config,:);
% ikJointsHome(1,1)=ikJointsHome(1,1)-pi/2;
% ikJointsHome(1,2)=ikJointsHome(1,2)+pi/2;
% ikJointsHome(1,4)=ikJointsHome(1,4)-pi/2;

ikJoints=ikine(myrobot, T11,endEffector, base);
ikJoints*degrees;
ikJoints1=ikJoints(config,:);
% ikJoints1(1,1)=ikJoints1(1,1)-pi/2;
% ikJoints1(1,2)=ikJoints1(1,2)+pi/2;
% ikJoints1(1,4)=ikJoints1(1,4)-pi/2;

ikJoints=ikine(myrobot, T110,endEffector, base);
ikJoints*degrees;
ikJoints2=ikJoints(config,:);
% ikJoints2(1,1)=ikJoints2(1,1)-pi/2;
% ikJoints2(1,2)=ikJoints2(1,2)+pi/2;
% ikJoints2(1,4)=ikJoints2(1,4)-pi/2;

ikJoints=ikine(myrobot, T12,endEffector, base);
ikJoints*degrees;
ikJoints3=ikJoints(config,:);
% ikJoints3(1,1)=ikJoints3(1,1)-pi/2;
% ikJoints3(1,2)=ikJoints3(1,2)+pi/2;
% ikJoints3(1,4)=ikJoints3(1,4)-pi/2;

ikJoints=ikine(myrobot, T120,endEffector, base);
ikJoints*degrees;
ikJoints4=ikJoints(config,:);

ikJoints=ikine(myrobot, T31,endEffector, base);
ikJoints*degrees;
ikJoints5=ikJoints(config,:);

ikJoints=ikine(myrobot, T310,endEffector, base);
ikJoints*degrees;
ikJoints6=ikJoints(config,:);

ikJoints=ikine(myrobot, T32,endEffector, base);
ikJoints*degrees;
ikJoints7=ikJoints(config,:);

ikJoints=ikine(myrobot, T320,endEffector, base);
ikJoints*degrees;
ikJoints8=ikJoints(config,:);

%display
figure(2);
showRobot(myrobot,ikJoints1);
hold on;
plot3(T11(1,4),T11(2,4),T11(3,4),'r.','MarkerSize',20);
plot3(T110(1,4),T110(2,4),T110(3,4),'r.','MarkerSize',20);
plot3(T12(1,4),T12(2,4),T12(3,4),'r.','MarkerSize',20);
plot3(HOME(1,4),HOME(2,4),HOME(3,4),'r.','MarkerSize',20);
plot3(T120(1,4),T120(2,4),T120(3,4),'r.','MarkerSize',20);
plot3(T31(1,4),T31(2,4),T31(3,4),'r.','MarkerSize',20);
plot3(T310(1,4),T310(2,4),T310(3,4),'r.','MarkerSize',20);
plot3(T32(1,4),T32(2,4),T32(3,4),'r.','MarkerSize',20);
plot3(T320(1,4),T320(2,4),T320(3,4),'r.','MarkerSize',20);
%% Path visualize task1
%define x y z postion array
A=[];
B=[];
C=[];
%Set current robot joint configuration.
currentRobotJConfig = ikJointsHome;
%Specify the trajectory time step and approximate desired tool speed.
timeStep = 0.1; % seconds
toolSpeed = 0.1; % m/s
%Set the initial and final end-effector pose.



%HOMEto110
jointInit = ikJointsHome;
jointFinal = ikJoints2;

taskInit = HOME;
taskFinal = T110;
%First, compute tool traveling distance.
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
%Next, define trajectory times based on traveling distance and desired tool speed.
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
numPoints=length(trajTimes);
timeInterval = [trajTimes(1); trajTimes(end)];

%Joint-Space Trajectory
ctrlpoints = [jointInit',jointFinal'];
jointArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
poseArray=zeros(4,4,numPoints);
for i=1:numPoints
    jointNow=jointArray(:,i);
    poseNow=fkine(myrobot, jointNow, endEffector, base);
    poseArray(:,:,i)=poseNow;
end
% plot
figure(4);
title("Joint Move")
hold on
axis([-1 1 -1 1 -0.1 1.5]);
for i=1:numPoints
    jointNow=jointArray(:,i);
    poseNow=poseArray(:,:,i);
    %position storage
    A=[A poseNow(1,4)];
    B=[B poseNow(2,4)];
    C=[C poseNow(3,4)];
    %show robot posture
    showRobot(myrobot,jointNow);
    hold on;
    
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
    scatter3(A,B,C,5,'r','filled');
    ax = gca;
plane = collisionBox(2,2,0.025);
plane.Pose = trvec2tform([0 0 -0.0125]);
show(plane,'Parent', ax);

topPlate = collisionBox(0.26,0.02,0.5);
topPlate.Pose = trvec2tform([-0.33 -0.01 0.25]);
[~, patchObj] = show(topPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

bottomPlate =  collisionBox(0.26,0.02,0.5);
bottomPlate.Pose = trvec2tform([-0.13 -0.101 0.25]);
[~, patchObj] = show(bottomPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

leftShelf = collisionBox(0.1,0.05,0.05);
leftShelf.Pose = trvec2tform([0.4 0.025 0.025]);
[~, patchObj] = show(leftShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

targetShelf = collisionBox(0.26,0.5,0.02);
targetShelf.Pose = trvec2tform([0 0.45 0.05]);
[~, patchObj] = show(targetShelf,'Parent',ax);
patchObj.FaceColor = [0 1 0];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel1 = collisionCylinder(0.03,0.003);
Wheel1.Pose = trvec2tform([0.3515 0.025 0.0515]);
[~, patchObj] = show(Wheel1,'Parent',ax);
patchObj.FaceColor = [0 0 1];

rightShelf = collisionBox(0.1,0.05,0.05);
rightShelf.Pose = trvec2tform([0.4 0.425 0.025]);
[~, patchObj] = show(rightShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel2 = collisionCylinder(0.03,0.003);
Wheel2.Pose = trvec2tform([0.3515 0.425 0.0515]);
[~, patchObj] = show(Wheel2,'Parent',ax);
patchObj.FaceColor = [0 0 1];

n=7;
rh=(2*pi)/(2*(n-1));
k=9;
hold on;
i=1;
t=linspace(0,2*pi,n);
x=0.42+0.02*cos(t);
y=0.425+0.02*((sqrt(3)/2)^(i-1))*sin(t);
z=[0.05,0.05,0.05,0.05,0.05,0.05,0.05]
plot3(x,y,z,'r','LineWidth',2);axis equal;
    hold off;
    drawnow;
end




%110to11
jointInit = ikJoints2;
jointFinal = ikJoints1;

taskInit = T110;
taskFinal = T11;
%First, compute tool traveling distance.
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
%Next, define trajectory times based on traveling distance and desired tool speed.
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
numPoints=length(trajTimes);
timeInterval = [trajTimes(1); trajTimes(end)];


currentRobotJConfig=ikJoints2;
%task space
[taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
numJoints=6;
joints=zeros(numJoints,numPoints);
for i=1:length(trajTimes)
    poseNow=taskWaypoints(:,:,i);
    jointNow=ikine2(myrobot,endEffector, weights,poseNow ,currentRobotJConfig);
    currentRobotJConfig=jointNow;
    joints(:,i)=jointNow;
end
% plot
figure(4);
title("Joint Move")
hold on
axis([-1 1 -1 1 -0.1 1.5]);
for i=1:length(trajTimes)
    poseNow=taskWaypoints(:,:,i);
    jointNow =joints(:,i);
    %show robot posture
    showRobot(myrobot,jointNow);
    %position storage
    A=[A poseNow(1,4)];
    B=[B poseNow(2,4)];
    C=[C poseNow(3,4)];

    hold on;
    
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
    scatter3(A,B,C,5,'r','filled');
    ax = gca;
plane = collisionBox(2,2,0.025);
plane.Pose = trvec2tform([0 0 -0.0125]);
show(plane,'Parent', ax);

topPlate = collisionBox(0.26,0.02,0.5);
topPlate.Pose = trvec2tform([-0.33 -0.01 0.25]);
[~, patchObj] = show(topPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

bottomPlate =  collisionBox(0.26,0.02,0.5);
bottomPlate.Pose = trvec2tform([-0.13 -0.101 0.25]);
[~, patchObj] = show(bottomPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

leftShelf = collisionBox(0.1,0.05,0.05);
leftShelf.Pose = trvec2tform([0.4 0.025 0.025]);
[~, patchObj] = show(leftShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

targetShelf = collisionBox(0.26,0.5,0.02);
targetShelf.Pose = trvec2tform([0 0.45 0.05]);
[~, patchObj] = show(targetShelf,'Parent',ax);
patchObj.FaceColor = [0 1 0];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel1 = collisionCylinder(0.03,0.003);
Wheel1.Pose = trvec2tform([0.3515 0.025 0.0515]);
[~, patchObj] = show(Wheel1,'Parent',ax);
patchObj.FaceColor = [0 0 1];

rightShelf = collisionBox(0.1,0.05,0.05);
rightShelf.Pose = trvec2tform([0.4 0.425 0.025]);
[~, patchObj] = show(rightShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel2 = collisionCylinder(0.03,0.003);
Wheel2.Pose = trvec2tform([0.3515 0.425 0.0515]);
[~, patchObj] = show(Wheel2,'Parent',ax);
patchObj.FaceColor = [0 0 1];

n=7;
rh=(2*pi)/(2*(n-1));
k=9;
hold on;
i=1;
t=linspace(0,2*pi,n);
x=0.42+0.02*cos(t);
y=0.425+0.02*((sqrt(3)/2)^(i-1))*sin(t);
z=[0.05,0.05,0.05,0.05,0.05,0.05,0.05]
plot3(x,y,z,'r','LineWidth',2);axis equal;
    hold off;
    drawnow;
end




%11to110
jointInit = ikJoints1;
jointFinal = ikJoints2;

taskInit = T11;
taskFinal = T110;
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
numPoints=length(trajTimes);
timeInterval = [trajTimes(1); trajTimes(end)];
ctrlpoints = [jointInit',jointFinal'];
jointArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
poseArray=zeros(4,4,numPoints);
for i=1:numPoints
    jointNow=jointArray(:,i);
    poseNow=fkine(myrobot, jointNow, endEffector, base);
    poseArray(:,:,i)=poseNow;
end
% plot
figure(4);
for i=1:numPoints
    jointNow=jointArray(:,i);
    poseNow=poseArray(:,:,i);
    A=[A poseNow(1,4)];
    B=[B poseNow(2,4)];
    C=[C poseNow(3,4)];
    %show robot posture
    showRobot(myrobot,jointNow);
    hold on;
    
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
    scatter3(A,B,C,5,'red','filled');
    ax = gca;
plane = collisionBox(2,2,0.025);
plane.Pose = trvec2tform([0 0 -0.0125]);
show(plane,'Parent', ax);

topPlate = collisionBox(0.26,0.02,0.5);
topPlate.Pose = trvec2tform([-0.33 -0.01 0.25]);
[~, patchObj] = show(topPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

bottomPlate =  collisionBox(0.26,0.02,0.5);
bottomPlate.Pose = trvec2tform([-0.13 -0.101 0.25]);
[~, patchObj] = show(bottomPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

leftShelf = collisionBox(0.1,0.05,0.05);
leftShelf.Pose = trvec2tform([0.4 0.025 0.025]);
[~, patchObj] = show(leftShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

targetShelf = collisionBox(0.26,0.5,0.02);
targetShelf.Pose = trvec2tform([0 0.45 0.05]);
[~, patchObj] = show(targetShelf,'Parent',ax);
patchObj.FaceColor = [0 1 0];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel1 = collisionCylinder(0.03,0.003);
Wheel1.Pose = trvec2tform([0.3515 0.025 0.0515]);
[~, patchObj] = show(Wheel1,'Parent',ax);
patchObj.FaceColor = [0 0 1];

rightShelf = collisionBox(0.1,0.05,0.05);
rightShelf.Pose = trvec2tform([0.4 0.425 0.025]);
[~, patchObj] = show(rightShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel2 = collisionCylinder(0.03,0.003);
Wheel2.Pose = trvec2tform([0.3515 0.425 0.0515]);
[~, patchObj] = show(Wheel2,'Parent',ax);
patchObj.FaceColor = [0 0 1];

n=7;
rh=(2*pi)/(2*(n-1));
k=9;
hold on;
i=1;
t=linspace(0,2*pi,n);
x=0.42+0.02*cos(t);
y=0.425+0.02*((sqrt(3)/2)^(i-1))*sin(t);
z=[0.05,0.05,0.05,0.05,0.05,0.05,0.05]
plot3(x,y,z,'r','LineWidth',2);axis equal;
    
    hold off;
    drawnow;
end




%110to120
jointInit = ikJoints2;
jointFinal = ikJoints4;

taskInit = T110;
taskFinal = T120;
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
numPoints=length(trajTimes);
timeInterval = [trajTimes(1); trajTimes(end)];
ctrlpoints = [jointInit',jointFinal'];
jointArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
poseArray=zeros(4,4,numPoints);
for i=1:numPoints
    jointNow=jointArray(:,i);
    poseNow=fkine(myrobot, jointNow, endEffector, base);
    poseArray(:,:,i)=poseNow;
end
% plot
figure(4);
for i=1:numPoints
    jointNow=jointArray(:,i);
    poseNow=poseArray(:,:,i);
    A=[A poseNow(1,4)];
    B=[B poseNow(2,4)];
    C=[C poseNow(3,4)];
    %show robot posture
    showRobot(myrobot,jointNow);
    hold on;
    
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
    scatter3(A,B,C,5,'red','filled');
    ax = gca;
plane = collisionBox(2,2,0.025);
plane.Pose = trvec2tform([0 0 -0.0125]);
show(plane,'Parent', ax);

topPlate = collisionBox(0.26,0.02,0.5);
topPlate.Pose = trvec2tform([-0.33 -0.01 0.25]);
[~, patchObj] = show(topPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

bottomPlate =  collisionBox(0.26,0.02,0.5);
bottomPlate.Pose = trvec2tform([-0.13 -0.101 0.25]);
[~, patchObj] = show(bottomPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

leftShelf = collisionBox(0.1,0.05,0.05);
leftShelf.Pose = trvec2tform([0.4 0.025 0.025]);
[~, patchObj] = show(leftShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

targetShelf = collisionBox(0.26,0.5,0.02);
targetShelf.Pose = trvec2tform([0 0.45 0.05]);
[~, patchObj] = show(targetShelf,'Parent',ax);
patchObj.FaceColor = [0 1 0];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel1 = collisionCylinder(0.03,0.003);
Wheel1.Pose = trvec2tform([0.3515 0.025 0.0515]);
[~, patchObj] = show(Wheel1,'Parent',ax);
patchObj.FaceColor = [0 0 1];

rightShelf = collisionBox(0.1,0.05,0.05);
rightShelf.Pose = trvec2tform([0.4 0.425 0.025]);
[~, patchObj] = show(rightShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel2 = collisionCylinder(0.03,0.003);
Wheel2.Pose = trvec2tform([0.3515 0.425 0.0515]);
[~, patchObj] = show(Wheel2,'Parent',ax);
patchObj.FaceColor = [0 0 1];

n=7;
rh=(2*pi)/(2*(n-1));
k=9;
hold on;
i=1;
t=linspace(0,2*pi,n);
x=0.42+0.02*cos(t);
y=0.425+0.02*((sqrt(3)/2)^(i-1))*sin(t);
z=[0.05,0.05,0.05,0.05,0.05,0.05,0.05]
plot3(x,y,z,'r','LineWidth',2);axis equal;

    hold off;
    drawnow;
end





%120to12
jointInit = ikJoints4;
jointFinal = ikJoints3;

taskInit = T120;
taskFinal = T12;

%First, compute tool traveling distance.
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
%Next, define trajectory times based on traveling distance and desired tool speed.
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
numPoints=length(trajTimes);
timeInterval = [trajTimes(1); trajTimes(end)];


currentRobotJConfig=ikJoints4;
%task space
[taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
numJoints=6;
joints=zeros(numJoints,numPoints);
for i=1:length(trajTimes)
    poseNow=taskWaypoints(:,:,i);
    jointNow=ikine2(myrobot,endEffector, weights,poseNow ,currentRobotJConfig);
    currentRobotJConfig=jointNow;
    joints(:,i)=jointNow;
end
% plot
figure(4);
title("Joint Move")
hold on
axis([-1 1 -1 1 -0.1 1.5]);
for i=1:length(trajTimes)
    poseNow=taskWaypoints(:,:,i);
    jointNow =joints(:,i);
    %show robot posture
    showRobot(myrobot,jointNow);
    %position storage
    A=[A poseNow(1,4)];
    B=[B poseNow(2,4)];
    C=[C poseNow(3,4)];

    hold on;
    
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
    scatter3(A,B,C,5,'r','filled');
    ax = gca;
plane = collisionBox(2,2,0.025);
plane.Pose = trvec2tform([0 0 -0.0125]);
show(plane,'Parent', ax);

topPlate = collisionBox(0.26,0.02,0.5);
topPlate.Pose = trvec2tform([-0.33 -0.01 0.25]);
[~, patchObj] = show(topPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

bottomPlate =  collisionBox(0.26,0.02,0.5);
bottomPlate.Pose = trvec2tform([-0.13 -0.101 0.25]);
[~, patchObj] = show(bottomPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

leftShelf = collisionBox(0.1,0.05,0.05);
leftShelf.Pose = trvec2tform([0.4 0.025 0.025]);
[~, patchObj] = show(leftShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

targetShelf = collisionBox(0.26,0.5,0.02);
targetShelf.Pose = trvec2tform([0 0.45 0.05]);
[~, patchObj] = show(targetShelf,'Parent',ax);
patchObj.FaceColor = [0 1 0];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel1 = collisionCylinder(0.03,0.003);
Wheel1.Pose = trvec2tform([0.3515 0.025 0.0515]);
[~, patchObj] = show(Wheel1,'Parent',ax);
patchObj.FaceColor = [0 0 1];

rightShelf = collisionBox(0.1,0.05,0.05);
rightShelf.Pose = trvec2tform([0.4 0.425 0.025]);
[~, patchObj] = show(rightShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel2 = collisionCylinder(0.03,0.003);
Wheel2.Pose = trvec2tform([0.3515 0.425 0.0515]);
[~, patchObj] = show(Wheel2,'Parent',ax);
patchObj.FaceColor = [0 0 1];

n=7;
rh=(2*pi)/(2*(n-1));
k=9;
hold on;
i=1;
t=linspace(0,2*pi,n);
x=0.42+0.02*cos(t);
y=0.425+0.02*((sqrt(3)/2)^(i-1))*sin(t);
z=[0.05,0.05,0.05,0.05,0.05,0.05,0.05]
plot3(x,y,z,'r','LineWidth',2);axis equal;
    hold off;
    drawnow;
end

%12to120
jointInit = ikJoints3;
jointFinal = ikJoints4;

taskInit = T12;
taskFinal = T120;

%First, compute tool traveling distance.
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
%Next, define trajectory times based on traveling distance and desired tool speed.
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
numPoints=length(trajTimes);
timeInterval = [trajTimes(1); trajTimes(end)];


currentRobotJConfig=ikJoints3;
%task space
[taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
numJoints=6;
joints=zeros(numJoints,numPoints);
for i=1:length(trajTimes)
    poseNow=taskWaypoints(:,:,i);
    jointNow=ikine2(myrobot,endEffector, weights,poseNow ,currentRobotJConfig);
    currentRobotJConfig=jointNow;
    joints(:,i)=jointNow;
end
% plot
figure(4);
title("Joint Move")
hold on
axis([-1 1 -1 1 -0.1 1.5]);
for i=1:length(trajTimes)
    poseNow=taskWaypoints(:,:,i);
    jointNow =joints(:,i);
    %show robot posture
    showRobot(myrobot,jointNow);
    %position storage
    A=[A poseNow(1,4)];
    B=[B poseNow(2,4)];
    C=[C poseNow(3,4)];

    hold on;
    
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
    scatter3(A,B,C,5,'r','filled');
    ax = gca;
plane = collisionBox(2,2,0.025);
plane.Pose = trvec2tform([0 0 -0.0125]);
show(plane,'Parent', ax);

topPlate = collisionBox(0.26,0.02,0.5);
topPlate.Pose = trvec2tform([-0.33 -0.01 0.25]);
[~, patchObj] = show(topPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

bottomPlate =  collisionBox(0.26,0.02,0.5);
bottomPlate.Pose = trvec2tform([-0.13 -0.101 0.25]);
[~, patchObj] = show(bottomPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

leftShelf = collisionBox(0.1,0.05,0.05);
leftShelf.Pose = trvec2tform([0.4 0.025 0.025]);
[~, patchObj] = show(leftShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

targetShelf = collisionBox(0.26,0.5,0.02);
targetShelf.Pose = trvec2tform([0 0.45 0.05]);
[~, patchObj] = show(targetShelf,'Parent',ax);
patchObj.FaceColor = [0 1 0];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel1 = collisionCylinder(0.03,0.003);
Wheel1.Pose = trvec2tform([0.3515 0.025 0.0515]);
[~, patchObj] = show(Wheel1,'Parent',ax);
patchObj.FaceColor = [0 0 1];

rightShelf = collisionBox(0.1,0.05,0.05);
rightShelf.Pose = trvec2tform([0.4 0.425 0.025]);
[~, patchObj] = show(rightShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel2 = collisionCylinder(0.03,0.003);
Wheel2.Pose = trvec2tform([0.3515 0.425 0.0515]);
[~, patchObj] = show(Wheel2,'Parent',ax);
patchObj.FaceColor = [0 0 1];

n=7;
rh=(2*pi)/(2*(n-1));
k=9;
hold on;
i=1;
t=linspace(0,2*pi,n);
x=0.42+0.02*cos(t);
y=0.425+0.02*((sqrt(3)/2)^(i-1))*sin(t);
z=[0.05,0.05,0.05,0.05,0.05,0.05,0.05]
plot3(x,y,z,'r','LineWidth',2);axis equal;
    hold off;
    drawnow;
end

%120toHOME
jointInit = ikJoints4;
jointFinal = ikJointsHome;

taskInit = T120;
taskFinal = HOME;
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
numPoints=length(trajTimes);
timeInterval = [trajTimes(1); trajTimes(end)];
ctrlpoints = [jointInit',jointFinal'];
jointArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
poseArray=zeros(4,4,numPoints);
for i=1:numPoints
    jointNow=jointArray(:,i);
    poseNow=fkine(myrobot, jointNow, endEffector, base);
    poseArray(:,:,i)=poseNow;
end
% plot
figure(4);
for i=1:numPoints
    jointNow=jointArray(:,i);
    poseNow=poseArray(:,:,i);
    A=[A poseNow(1,4)];
    B=[B poseNow(2,4)];
    C=[C poseNow(3,4)];
    %show robot posture
    showRobot(myrobot,jointNow);
    hold on;
    
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
    scatter3(A,B,C,5,'red','filled');
    ax = gca;
plane = collisionBox(2,2,0.025);
plane.Pose = trvec2tform([0 0 -0.0125]);
show(plane,'Parent', ax);

topPlate = collisionBox(0.26,0.02,0.5);
topPlate.Pose = trvec2tform([-0.33 -0.01 0.25]);
[~, patchObj] = show(topPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

bottomPlate =  collisionBox(0.26,0.02,0.5);
bottomPlate.Pose = trvec2tform([-0.13 -0.101 0.25]);
[~, patchObj] = show(bottomPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

leftShelf = collisionBox(0.1,0.05,0.05);
leftShelf.Pose = trvec2tform([0.4 0.025 0.025]);
[~, patchObj] = show(leftShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

targetShelf = collisionBox(0.26,0.5,0.02);
targetShelf.Pose = trvec2tform([0 0.45 0.05]);
[~, patchObj] = show(targetShelf,'Parent',ax);
patchObj.FaceColor = [0 1 0];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel1 = collisionCylinder(0.03,0.003);
Wheel1.Pose = trvec2tform([0.3515 0.025 0.0515]);
[~, patchObj] = show(Wheel1,'Parent',ax);
patchObj.FaceColor = [0 0 1];

rightShelf = collisionBox(0.1,0.05,0.05);
rightShelf.Pose = trvec2tform([0.4 0.425 0.025]);
[~, patchObj] = show(rightShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel2 = collisionCylinder(0.03,0.003);
Wheel2.Pose = trvec2tform([0.3515 0.425 0.0515]);
[~, patchObj] = show(Wheel2,'Parent',ax);
patchObj.FaceColor = [0 0 1];

n=7;
rh=(2*pi)/(2*(n-1));
k=9;
hold on;
i=1;
t=linspace(0,2*pi,n);
x=0.42+0.02*cos(t);
y=0.425+0.02*((sqrt(3)/2)^(i-1))*sin(t);
z=[0.05,0.05,0.05,0.05,0.05,0.05,0.05]
plot3(x,y,z,'r','LineWidth',2);axis equal;

    hold off;
    drawnow;
end


%% Path visualize task 2
%define x y z array
E=[];
F=[];
G=[];
%Set current robot joint configuration.
currentRobotJConfig = ikJoints1;
%Specify the trajectory time step and approximate desired tool speed.
timeStep = 0.1; % seconds
toolSpeed = 0.1; % m/s
%Set the initial and final end-effector pose.



%HOMEto310
jointInit = ikJointsHome;
jointFinal = ikJoints6;

taskInit = HOME;
taskFinal = T310;
%First, compute tool traveling distance.
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
%Next, define trajectory times based on traveling distance and desired tool speed.
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
numPoints=length(trajTimes);
timeInterval = [trajTimes(1); trajTimes(end)];

%Joint-Space Trajectory
ctrlpoints = [jointInit',jointFinal'];
jointArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
poseArray=zeros(4,4,numPoints);
for i=1:numPoints
    jointNow=jointArray(:,i);
    poseNow=fkine(myrobot, jointNow, endEffector, base);
    poseArray(:,:,i)=poseNow;
end
% plot
figure(5);
title("Joint Move")
axis([-1 1 -1 1 -0.1 1.5]);
for i=1:numPoints
    jointNow=jointArray(:,i);
    poseNow=poseArray(:,:,i);
    %position storage
    E=[E poseNow(1,4)];
    F=[F poseNow(2,4)];
    G=[G poseNow(3,4)];
    %show robot posture
    showRobot(myrobot,jointNow);
    hold on;
    
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
    scatter3(E,F,G,5,'r','filled');
    ax = gca;
plane = collisionBox(2,2,0.025);
plane.Pose = trvec2tform([0 0 -0.0125]);
show(plane,'Parent', ax);

topPlate = collisionBox(0.26,0.02,0.5);
topPlate.Pose = trvec2tform([-0.33 -0.01 0.25]);
[~, patchObj] = show(topPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

bottomPlate =  collisionBox(0.26,0.02,0.5);
bottomPlate.Pose = trvec2tform([-0.13 -0.101 0.25]);
[~, patchObj] = show(bottomPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

leftShelf = collisionBox(0.1,0.05,0.05);
leftShelf.Pose = trvec2tform([0.4 0.025 0.025]);
[~, patchObj] = show(leftShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

targetShelf = collisionBox(0.26,0.5,0.02);
targetShelf.Pose = trvec2tform([0 0.45 0.05]);
[~, patchObj] = show(targetShelf,'Parent',ax);
patchObj.FaceColor = [0 1 0];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel1 = collisionCylinder(0.03,0.003);
Wheel1.Pose = trvec2tform([0.3515 0.025 0.0515]);
[~, patchObj] = show(Wheel1,'Parent',ax);
patchObj.FaceColor = [0 0 1];

rightShelf = collisionBox(0.1,0.05,0.05);
rightShelf.Pose = trvec2tform([0.4 0.425 0.025]);
[~, patchObj] = show(rightShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel2 = collisionCylinder(0.03,0.003);
Wheel2.Pose = trvec2tform([0.3515 0.425 0.0515]);
[~, patchObj] = show(Wheel2,'Parent',ax);
patchObj.FaceColor = [0 0 1];

n=7;
rh=(2*pi)/(2*(n-1));
k=9;
hold on;
i=1;
t=linspace(0,2*pi,n);
x=0.42+0.02*cos(t);
y=0.425+0.02*((sqrt(3)/2)^(i-1))*sin(t);
z=[0.05,0.05,0.05,0.05,0.05,0.05,0.05]
plot3(x,y,z,'r','LineWidth',2);axis equal;
    hold off;
    drawnow;
end



%310to31
jointInit = ikJoints6;
jointFinal = ikJoints5;

taskInit = T310;
taskFinal = T31;

%First, compute tool traveling distance.
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
%Next, define trajectory times based on traveling distance and desired tool speed.
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
numPoints=length(trajTimes);
timeInterval = [trajTimes(1); trajTimes(end)];


currentRobotJConfig=ikJoints6;
%task space
[taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
numJoints=6;
joints=zeros(numJoints,numPoints);
for i=1:length(trajTimes)
    poseNow=taskWaypoints(:,:,i);
    jointNow=ikine2(myrobot,endEffector, weights,poseNow ,currentRobotJConfig);
    currentRobotJConfig=jointNow;
    joints(:,i)=jointNow;
end
% plot
figure(5);
title("Joint Move")
hold on
axis([-1 1 -1 1 -0.1 1.5]);
for i=1:length(trajTimes)
    poseNow=taskWaypoints(:,:,i);
    jointNow =joints(:,i);
    %show robot posture
    showRobot(myrobot,jointNow);
    %position storage
    E=[E poseNow(1,4)];
    F=[F poseNow(2,4)];
    G=[G poseNow(3,4)];

    hold on;
    
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
    scatter3(E,F,G,5,'r','filled');
    ax = gca;
plane = collisionBox(2,2,0.025);
plane.Pose = trvec2tform([0 0 -0.0125]);
show(plane,'Parent', ax);

topPlate = collisionBox(0.26,0.02,0.5);
topPlate.Pose = trvec2tform([-0.33 -0.01 0.25]);
[~, patchObj] = show(topPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

bottomPlate =  collisionBox(0.26,0.02,0.5);
bottomPlate.Pose = trvec2tform([-0.13 -0.101 0.25]);
[~, patchObj] = show(bottomPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

leftShelf = collisionBox(0.1,0.05,0.05);
leftShelf.Pose = trvec2tform([0.4 0.025 0.025]);
[~, patchObj] = show(leftShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

targetShelf = collisionBox(0.26,0.5,0.02);
targetShelf.Pose = trvec2tform([0 0.45 0.05]);
[~, patchObj] = show(targetShelf,'Parent',ax);
patchObj.FaceColor = [0 1 0];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel1 = collisionCylinder(0.03,0.003);
Wheel1.Pose = trvec2tform([0.3515 0.025 0.0515]);
[~, patchObj] = show(Wheel1,'Parent',ax);
patchObj.FaceColor = [0 0 1];

rightShelf = collisionBox(0.1,0.05,0.05);
rightShelf.Pose = trvec2tform([0.4 0.425 0.025]);
[~, patchObj] = show(rightShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel2 = collisionCylinder(0.03,0.003);
Wheel2.Pose = trvec2tform([0.3515 0.425 0.0515]);
[~, patchObj] = show(Wheel2,'Parent',ax);
patchObj.FaceColor = [0 0 1];

n=7;
rh=(2*pi)/(2*(n-1));
k=9;
hold on;
i=1;
t=linspace(0,2*pi,n);
x=0.42+0.02*cos(t);
y=0.425+0.02*((sqrt(3)/2)^(i-1))*sin(t);
z=[0.05,0.05,0.05,0.05,0.05,0.05,0.05]
plot3(x,y,z,'r','LineWidth',2);axis equal;
    hold off;
    drawnow;
end



%31to310
jointInit = ikJoints5;
jointFinal = ikJoints6;

taskInit = T31;
taskFinal = T310;

%First, compute tool traveling distance.
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
%Next, define trajectory times based on traveling distance and desired tool speed.
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
numPoints=length(trajTimes);
timeInterval = [trajTimes(1); trajTimes(end)];


currentRobotJConfig=ikJoints5;
%task space
[taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
numJoints=6;
joints=zeros(numJoints,numPoints);
for i=1:length(trajTimes)
    poseNow=taskWaypoints(:,:,i);
    jointNow=ikine2(myrobot,endEffector, weights,poseNow ,currentRobotJConfig);
    currentRobotJConfig=jointNow;
    joints(:,i)=jointNow;
end
% plot
figure(5);
title("Joint Move")
hold on
axis([-1 1 -1 1 -0.1 1.5]);
for i=1:length(trajTimes)
    poseNow=taskWaypoints(:,:,i);
    jointNow =joints(:,i);
    %show robot posture
    showRobot(myrobot,jointNow);
    %position storage
    E=[E poseNow(1,4)];
    F=[F poseNow(2,4)];
    G=[G poseNow(3,4)];

    hold on;
    
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
    scatter3(E,F,G,5,'r','filled');
    ax = gca;
plane = collisionBox(2,2,0.025);
plane.Pose = trvec2tform([0 0 -0.0125]);
show(plane,'Parent', ax);

topPlate = collisionBox(0.26,0.02,0.5);
topPlate.Pose = trvec2tform([-0.33 -0.01 0.25]);
[~, patchObj] = show(topPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

bottomPlate =  collisionBox(0.26,0.02,0.5);
bottomPlate.Pose = trvec2tform([-0.13 -0.101 0.25]);
[~, patchObj] = show(bottomPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

leftShelf = collisionBox(0.1,0.05,0.05);
leftShelf.Pose = trvec2tform([0.4 0.025 0.025]);
[~, patchObj] = show(leftShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

targetShelf = collisionBox(0.26,0.5,0.02);
targetShelf.Pose = trvec2tform([0 0.45 0.05]);
[~, patchObj] = show(targetShelf,'Parent',ax);
patchObj.FaceColor = [0 1 0];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel1 = collisionCylinder(0.03,0.003);
Wheel1.Pose = trvec2tform([0.3515 0.025 0.0515]);
[~, patchObj] = show(Wheel1,'Parent',ax);
patchObj.FaceColor = [0 0 1];

rightShelf = collisionBox(0.1,0.05,0.05);
rightShelf.Pose = trvec2tform([0.4 0.425 0.025]);
[~, patchObj] = show(rightShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel2 = collisionCylinder(0.03,0.003);
Wheel2.Pose = trvec2tform([0.3515 0.425 0.0515]);
[~, patchObj] = show(Wheel2,'Parent',ax);
patchObj.FaceColor = [0 0 1];

n=7;
rh=(2*pi)/(2*(n-1));
k=9;
hold on;
i=1;
t=linspace(0,2*pi,n);
x=0.42+0.02*cos(t);
y=0.425+0.02*((sqrt(3)/2)^(i-1))*sin(t);
z=[0.05,0.05,0.05,0.05,0.05,0.05,0.05]
plot3(x,y,z,'r','LineWidth',2);axis equal;
    hold off;
    drawnow;
end



%310to320
jointInit = ikJoints6;
jointFinal = ikJoints8;

taskInit = T310;
taskFinal = T320;
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
numPoints=length(trajTimes);
timeInterval = [trajTimes(1); trajTimes(end)];
ctrlpoints = [jointInit',jointFinal'];
jointArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
poseArray=zeros(4,4,numPoints);
for i=1:numPoints
    jointNow=jointArray(:,i);
    poseNow=fkine(myrobot, jointNow, endEffector, base);
    poseArray(:,:,i)=poseNow;
end
% plot
for i=1:numPoints
    jointNow=jointArray(:,i);
    poseNow=poseArray(:,:,i);
    %position storage
    E=[E poseNow(1,4)];
    F=[F poseNow(2,4)];
    G=[G poseNow(3,4)];
    %show robot posture
    showRobot(myrobot,jointNow);
    hold on;
    
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
    scatter3(E,F,G,5,'r','filled');
    ax = gca;
plane = collisionBox(2,2,0.025);
plane.Pose = trvec2tform([0 0 -0.0125]);
show(plane,'Parent', ax);

topPlate = collisionBox(0.26,0.02,0.5);
topPlate.Pose = trvec2tform([-0.33 -0.01 0.25]);
[~, patchObj] = show(topPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

bottomPlate =  collisionBox(0.26,0.02,0.5);
bottomPlate.Pose = trvec2tform([-0.13 -0.101 0.25]);
[~, patchObj] = show(bottomPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

leftShelf = collisionBox(0.1,0.05,0.05);
leftShelf.Pose = trvec2tform([0.4 0.025 0.025]);
[~, patchObj] = show(leftShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

targetShelf = collisionBox(0.26,0.5,0.02);
targetShelf.Pose = trvec2tform([0 0.45 0.05]);
[~, patchObj] = show(targetShelf,'Parent',ax);
patchObj.FaceColor = [0 1 0];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel1 = collisionCylinder(0.03,0.003);
Wheel1.Pose = trvec2tform([0.3515 0.025 0.0515]);
[~, patchObj] = show(Wheel1,'Parent',ax);
patchObj.FaceColor = [0 0 1];

rightShelf = collisionBox(0.1,0.05,0.05);
rightShelf.Pose = trvec2tform([0.4 0.425 0.025]);
[~, patchObj] = show(rightShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel2 = collisionCylinder(0.03,0.003);
Wheel2.Pose = trvec2tform([0.3515 0.425 0.0515]);
[~, patchObj] = show(Wheel2,'Parent',ax);
patchObj.FaceColor = [0 0 1];

n=7;
rh=(2*pi)/(2*(n-1));
k=9;
hold on;
i=1;
t=linspace(0,2*pi,n);
x=0.42+0.02*cos(t);
y=0.425+0.02*((sqrt(3)/2)^(i-1))*sin(t);
z=[0.05,0.05,0.05,0.05,0.05,0.05,0.05]
plot3(x,y,z,'r','LineWidth',2);axis equal;
    hold off;
    drawnow;
end




%320to32
jointInit = ikJoints8;
jointFinal = ikJoints7;

taskInit = T320;
taskFinal = T32;


%First, compute tool traveling distance.
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
%Next, define trajectory times based on traveling distance and desired tool speed.
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
numPoints=length(trajTimes);
timeInterval = [trajTimes(1); trajTimes(end)];


currentRobotJConfig=ikJoints8;
%task space
[taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
numJoints=6;
joints=zeros(numJoints,numPoints);
for i=1:length(trajTimes)
    poseNow=taskWaypoints(:,:,i);
    jointNow=ikine2(myrobot,endEffector, weights,poseNow ,currentRobotJConfig);
    currentRobotJConfig=jointNow;
    joints(:,i)=jointNow;
end
% plot
figure(5);
title("Joint Move")
hold on
axis([-1 1 -1 1 -0.1 1.5]);
for i=1:length(trajTimes)
    poseNow=taskWaypoints(:,:,i);
    jointNow =joints(:,i);
    %show robot posture
    showRobot(myrobot,jointNow);
    %position storage
    E=[E poseNow(1,4)];
    F=[F poseNow(2,4)];
    G=[G poseNow(3,4)];

    hold on;
    
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
    scatter3(E,F,G,5,'r','filled');
    ax = gca;
plane = collisionBox(2,2,0.025);
plane.Pose = trvec2tform([0 0 -0.0125]);
show(plane,'Parent', ax);

topPlate = collisionBox(0.26,0.02,0.5);
topPlate.Pose = trvec2tform([-0.33 -0.01 0.25]);
[~, patchObj] = show(topPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

bottomPlate =  collisionBox(0.26,0.02,0.5);
bottomPlate.Pose = trvec2tform([-0.13 -0.101 0.25]);
[~, patchObj] = show(bottomPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

leftShelf = collisionBox(0.1,0.05,0.05);
leftShelf.Pose = trvec2tform([0.4 0.025 0.025]);
[~, patchObj] = show(leftShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

targetShelf = collisionBox(0.26,0.5,0.02);
targetShelf.Pose = trvec2tform([0 0.45 0.05]);
[~, patchObj] = show(targetShelf,'Parent',ax);
patchObj.FaceColor = [0 1 0];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel1 = collisionCylinder(0.03,0.003);
Wheel1.Pose = trvec2tform([0.3515 0.025 0.0515]);
[~, patchObj] = show(Wheel1,'Parent',ax);
patchObj.FaceColor = [0 0 1];

rightShelf = collisionBox(0.1,0.05,0.05);
rightShelf.Pose = trvec2tform([0.4 0.425 0.025]);
[~, patchObj] = show(rightShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel2 = collisionCylinder(0.03,0.003);
Wheel2.Pose = trvec2tform([0.3515 0.425 0.0515]);
[~, patchObj] = show(Wheel2,'Parent',ax);
patchObj.FaceColor = [0 0 1];

n=7;
rh=(2*pi)/(2*(n-1));
k=9;
hold on;
i=1;
t=linspace(0,2*pi,n);
x=0.42+0.02*cos(t);
y=0.425+0.02*((sqrt(3)/2)^(i-1))*sin(t);
z=[0.05,0.05,0.05,0.05,0.05,0.05,0.05]
plot3(x,y,z,'r','LineWidth',2);axis equal;
    hold off;
    drawnow;
end

%32to320
jointInit = ikJoints7;
jointFinal = ikJoints8;

taskInit = T32;
taskFinal = T320;

%First, compute tool traveling distance.
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
%Next, define trajectory times based on traveling distance and desired tool speed.
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
numPoints=length(trajTimes);
timeInterval = [trajTimes(1); trajTimes(end)];


currentRobotJConfig=ikJoints7;
%task space
[taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
numJoints=6;
joints=zeros(numJoints,numPoints);
for i=1:length(trajTimes)
    poseNow=taskWaypoints(:,:,i);
    jointNow=ikine2(myrobot,endEffector, weights,poseNow ,currentRobotJConfig);
    currentRobotJConfig=jointNow;
    joints(:,i)=jointNow;
end
% plot
figure(5);
title("Joint Move")
hold on
axis([-1 1 -1 1 -0.1 1.5]);
for i=1:length(trajTimes)
    poseNow=taskWaypoints(:,:,i);
    jointNow =joints(:,i);
    %show robot posture
    showRobot(myrobot,jointNow);
    %position storage
    E=[E poseNow(1,4)];
    F=[F poseNow(2,4)];
    G=[G poseNow(3,4)];

    hold on;
    
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
    scatter3(E,F,G,5,'r','filled');
    ax = gca;
plane = collisionBox(2,2,0.025);
plane.Pose = trvec2tform([0 0 -0.0125]);
show(plane,'Parent', ax);

topPlate = collisionBox(0.26,0.02,0.5);
topPlate.Pose = trvec2tform([-0.33 -0.01 0.25]);
[~, patchObj] = show(topPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

bottomPlate =  collisionBox(0.26,0.02,0.5);
bottomPlate.Pose = trvec2tform([-0.13 -0.101 0.25]);
[~, patchObj] = show(bottomPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

leftShelf = collisionBox(0.1,0.05,0.05);
leftShelf.Pose = trvec2tform([0.4 0.025 0.025]);
[~, patchObj] = show(leftShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

targetShelf = collisionBox(0.26,0.5,0.02);
targetShelf.Pose = trvec2tform([0 0.45 0.05]);
[~, patchObj] = show(targetShelf,'Parent',ax);
patchObj.FaceColor = [0 1 0];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel1 = collisionCylinder(0.03,0.003);
Wheel1.Pose = trvec2tform([0.3515 0.025 0.0515]);
[~, patchObj] = show(Wheel1,'Parent',ax);
patchObj.FaceColor = [0 0 1];

rightShelf = collisionBox(0.1,0.05,0.05);
rightShelf.Pose = trvec2tform([0.4 0.425 0.025]);
[~, patchObj] = show(rightShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel2 = collisionCylinder(0.03,0.003);
Wheel2.Pose = trvec2tform([0.3515 0.425 0.0515]);
[~, patchObj] = show(Wheel2,'Parent',ax);
patchObj.FaceColor = [0 0 1];

n=7;
rh=(2*pi)/(2*(n-1));
k=9;
hold on;
i=1;
t=linspace(0,2*pi,n);
x=0.42+0.02*cos(t);
y=0.425+0.02*((sqrt(3)/2)^(i-1))*sin(t);
z=[0.05,0.05,0.05,0.05,0.05,0.05,0.05]
plot3(x,y,z,'r','LineWidth',2);axis equal;
    hold off;
    drawnow;
end



%320toHOME
jointInit = ikJoints8;
jointFinal = ikJointsHome;

taskInit = T320;
taskFinal = HOME;
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
numPoints=length(trajTimes);
timeInterval = [trajTimes(1); trajTimes(end)];
ctrlpoints = [jointInit',jointFinal'];
jointArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
poseArray=zeros(4,4,numPoints);
for i=1:numPoints
    jointNow=jointArray(:,i);
    poseNow=fkine(myrobot, jointNow, endEffector, base);
    poseArray(:,:,i)=poseNow;
end
% plot
for i=1:numPoints
    jointNow=jointArray(:,i);
    poseNow=poseArray(:,:,i);
    %position storage
    E=[E poseNow(1,4)];
    F=[F poseNow(2,4)];
    G=[G poseNow(3,4)];
    %show robot posture
    showRobot(myrobot,jointNow);
    hold on;
    
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
    scatter3(E,F,G,5,'r','filled');
    ax = gca;
plane = collisionBox(2,2,0.025);
plane.Pose = trvec2tform([0 0 -0.0125]);
show(plane,'Parent', ax);

topPlate = collisionBox(0.26,0.02,0.5);
topPlate.Pose = trvec2tform([-0.33 -0.01 0.25]);
[~, patchObj] = show(topPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

bottomPlate =  collisionBox(0.26,0.02,0.5);
bottomPlate.Pose = trvec2tform([-0.13 -0.101 0.25]);
[~, patchObj] = show(bottomPlate,'Parent',ax);
patchObj.FaceColor = [0 1 0];

leftShelf = collisionBox(0.1,0.05,0.05);
leftShelf.Pose = trvec2tform([0.4 0.025 0.025]);
[~, patchObj] = show(leftShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

targetShelf = collisionBox(0.26,0.5,0.02);
targetShelf.Pose = trvec2tform([0 0.45 0.05]);
[~, patchObj] = show(targetShelf,'Parent',ax);
patchObj.FaceColor = [0 1 0];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Spacer = collisionCylinder(0.003,0.03);
Spacer.Pose = trvec2tform([0.42 0.025 0.065]);
[~, patchObj] = show(Spacer,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel1 = collisionCylinder(0.03,0.003);
Wheel1.Pose = trvec2tform([0.3515 0.025 0.0515]);
[~, patchObj] = show(Wheel1,'Parent',ax);
patchObj.FaceColor = [0 0 1];

rightShelf = collisionBox(0.1,0.05,0.05);
rightShelf.Pose = trvec2tform([0.4 0.425 0.025]);
[~, patchObj] = show(rightShelf,'Parent',ax);
patchObj.FaceColor = [0 0 1];

Wheel2 = collisionCylinder(0.03,0.003);
Wheel2.Pose = trvec2tform([0.3515 0.425 0.0515]);
[~, patchObj] = show(Wheel2,'Parent',ax);
patchObj.FaceColor = [0 0 1];

n=7;
rh=(2*pi)/(2*(n-1));
k=9;
hold on;
i=1;
t=linspace(0,2*pi,n);
x=0.42+0.02*cos(t);
y=0.425+0.02*((sqrt(3)/2)^(i-1))*sin(t);
z=[0.05,0.05,0.05,0.05,0.05,0.05,0.05]
plot3(x,y,z,'r','LineWidth',2);axis equal;
    hold off;
    drawnow;
end



% %% Plan trajectory plot2 task1
% %Set current robot joint configuration.
% currentRobotJConfig = ikJointsHome;
% %Specify the trajectory time step and approximate desired tool speed.
% timeStep = 0.1; % seconds
% toolSpeed = 0.1; % m/s
% %Set the initial and final end-effector pose.
% %HOMEto11
% jointInit = ikJointsHome;
% jointFinal = ikJoints1;
% 
% taskInit = HOME;
% taskFinal = T11;
% %First, compute tool traveling distance.
% distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
% %Next, define trajectory times based on traveling distance and desired tool speed.
% initTime = 0;
% finalTime = (distance/toolSpeed) - initTime;
% trajTimes = initTime:timeStep:finalTime;
% numPoints=length(trajTimes);
% timeInterval = [trajTimes(1); trajTimes(end)];
% 
% %Joint-Space Trajectory
% ctrlpoints = [jointInit',jointFinal'];
% jointArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
% poseArray=zeros(4,4,numPoints);
% for i=1:numPoints
%     jointNow=jointArray(:,i);
%     poseNow=fkine(myrobot, jointNow, endEffector, base);
%     poseArray(:,:,i)=poseNow;
% end
% 
% 
% %task space
% [taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
% numJoints=6;
% joints=zeros(numJoints,numPoints);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jointNow=ikine2(myrobot,endEffector, weights,poseNow ,currentRobotJConfig);
%     joints(:,i)=jointNow;
% end
% 
% %plot
% figure(7);
% 
% hold on
% axis([-1 1 -1 1 -0.1 1.5]);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jiontNow =joints(:,i);
%     %showRobot(myrobot,jointNow);
%     taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
%     drawnow;
% end
% showRobot(myrobot,currentRobotJConfig);
% hold on;
% 
% %11to110
% jointInit = ikJoints1;
% jointFinal = ikJoints2;
% 
% taskInit = T11;
% taskFinal = T110;
% distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
% initTime = 0;
% finalTime = (distance/toolSpeed) - initTime;
% trajTimes = initTime:timeStep:finalTime;
% numPoints=length(trajTimes);
% timeInterval = [trajTimes(1); trajTimes(end)];
% ctrlpoints = [jointInit',jointFinal'];
% jointArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
% poseArray=zeros(4,4,numPoints);
% for i=1:numPoints
%     jointNow=jointArray(:,i);
%     poseNow=fkine(myrobot, jointNow, endEffector, base);
%     poseArray(:,:,i)=poseNow;
% end
% 
% 
% %task space
% [taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
% joints=zeros(numJoints,numPoints);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jointNow=ikine2(myrobot,endEffector, weights,poseNow ,currentRobotJConfig);
%     joints(:,i)=jointNow;
% end
% 
% %plot
% figure(7);
% 
% hold on
% axis([-1 1 -1 1 -0.1 1.5]);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jiontNow =joints(:,i);
%     %showRobot(myrobot,jointNow);
%     taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
%     drawnow;
% end
% showRobot(myrobot,currentRobotJConfig);
% hold on;
% 
% %110to120
% jointInit = ikJoints2;
% jointFinal = ikJoints4;
% 
% taskInit = T110;
% taskFinal = T120;
% distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
% initTime = 0;
% finalTime = (distance/toolSpeed) - initTime;
% trajTimes = initTime:timeStep:finalTime;
% numPoints=length(trajTimes);
% timeInterval = [trajTimes(1); trajTimes(end)];
% ctrlpoints = [jointInit',jointFinal'];
% jointArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
% poseArray=zeros(4,4,numPoints);
% for i=1:numPoints
%     jointNow=jointArray(:,i);
%     poseNow=fkine(myrobot, jointNow, endEffector, base);
%     poseArray(:,:,i)=poseNow;
% end
% 
% %task space
% [taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
% joints=zeros(numJoints,numPoints);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jointNow=ikine2(myrobot,endEffector, weights,poseNow ,currentRobotJConfig);
%     joints(:,i)=jointNow;
% end
% 
% %plot
% figure(7);
% 
% hold on
% axis([-1 1 -1 1 -0.1 1.5]);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jiontNow =joints(:,i);
%     %showRobot(myrobot,jointNow);
%     taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
%     drawnow;
% end
% showRobot(myrobot,currentRobotJConfig);
% hold on;
% 
% %120to12
% jointInit = ikJoints4;
% jointFinal = ikJoints3;
% 
% taskInit = T120;
% taskFinal = T12;
% distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
% initTime = 0;
% finalTime = (distance/toolSpeed) - initTime;
% trajTimes = initTime:timeStep:finalTime;
% numPoints=length(trajTimes);
% timeInterval = [trajTimes(1); trajTimes(end)];
% ctrlpoints = [jointInit',jointFinal'];
% jointArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
% poseArray=zeros(4,4,numPoints);
% for i=1:numPoints
%     jointNow=jointArray(:,i);
%     poseNow=fkine(myrobot, jointNow, endEffector, base);
%     poseArray(:,:,i)=poseNow;
% end
% 
% %task space
% [taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
% joints=zeros(numJoints,numPoints);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jointNow=ikine2(myrobot,endEffector, weights,poseNow ,currentRobotJConfig);
%     joints(:,i)=jointNow;
% end
% 
% %plot
% figure(7);
% 
% hold on
% axis([-1 1 -1 1 -0.1 1.5]);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jiontNow =joints(:,i);
%     %showRobot(myrobot,jointNow);
%     taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
%     drawnow;
% end
% showRobot(myrobot,currentRobotJConfig);
% hold on;
% 
% 
% %12toHOME
% jointInit = ikJoints3;
% jointFinal = ikJointsHome;
% 
% taskInit = T12;
% taskFinal = HOME;
% distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
% initTime = 0;
% finalTime = (distance/toolSpeed) - initTime;
% trajTimes = initTime:timeStep:finalTime;
% numPoints=length(trajTimes);
% timeInterval = [trajTimes(1); trajTimes(end)];
% ctrlpoints = [jointInit',jointFinal'];
% jointArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
% poseArray=zeros(4,4,numPoints);
% for i=1:numPoints
%     jointNow=jointArray(:,i);
%     poseNow=fkine(myrobot, jointNow, endEffector, base);
%     poseArray(:,:,i)=poseNow;
% end
% 
% 
% %task space
% [taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
% joints=zeros(numJoints,numPoints);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jointNow=ikine2(myrobot,endEffector, weights,poseNow ,currentRobotJConfig);
%     joints(:,i)=jointNow;
% end
% 
% %plot
% figure(7);
% 
% hold on
% axis([-1 1 -1 1 -0.1 1.5]);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jiontNow =joints(:,i);
%     %showRobot(myrobot,jointNow);
%     taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
%     drawnow;
% end
% showRobot(myrobot,currentRobotJConfig);
% hold on;
% 
% 
% %% Plan trajectory between 2 points
% %Set current robot joint configuration.
% currentRobotJConfig = ikJoints1;
% %Specify the trajectory time step and approximate desired tool speed.
% timeStep = 0.1; % seconds
% toolSpeed = 0.1; % m/s
% %Set the initial and final end-effector pose.
% %HOMEto31
% jointInit = ikJointsHome;
% jointFinal = ikJoints5;
% 
% taskInit = HOME;
% taskFinal = T31;
% %First, compute tool traveling distance.
% distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
% %Next, define trajectory times based on traveling distance and desired tool speed.
% initTime = 0;
% finalTime = (distance/toolSpeed) - initTime;
% trajTimes = initTime:timeStep:finalTime;
% numPoints=length(trajTimes);
% timeInterval = [trajTimes(1); trajTimes(end)];
% 
% %Joint-Space Trajectory
% ctrlpoints = [jointInit',jointFinal'];
% jointArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
% poseArray=zeros(4,4,numPoints);
% for i=1:numPoints
%     jointNow=jointArray(:,i);
%     poseNow=fkine(myrobot, jointNow, endEffector, base);
%     poseArray(:,:,i)=poseNow;
% end
% 
% 
% %task space
% [taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
% joints=zeros(numJoints,numPoints);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jointNow=ikine2(myrobot,endEffector, weights,poseNow ,currentRobotJConfig);
%     joints(:,i)=jointNow;
% end
% 
% %plot
% figure(8);
% 
% hold on
% axis([-1 1 -1 1 -0.1 1.5]);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jiontNow =joints(:,i);
%     %showRobot(myrobot,jointNow);
%     taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
%     drawnow;
% end
% showRobot(myrobot,currentRobotJConfig);
% hold on;
% 
% %31to310
% jointInit = ikJoints5;
% jointFinal = ikJoints6;
% 
% taskInit = T31;
% taskFinal = T310;
% distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
% initTime = 0;
% finalTime = (distance/toolSpeed) - initTime;
% trajTimes = initTime:timeStep:finalTime;
% numPoints=length(trajTimes);
% timeInterval = [trajTimes(1); trajTimes(end)];
% ctrlpoints = [jointInit',jointFinal'];
% jointArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
% poseArray=zeros(4,4,numPoints);
% for i=1:numPoints
%     jointNow=jointArray(:,i);
%     poseNow=fkine(myrobot, jointNow, endEffector, base);
%     poseArray(:,:,i)=poseNow;
% end
% 
% 
% %task space
% [taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
% joints=zeros(numJoints,numPoints);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jointNow=ikine2(myrobot,endEffector, weights,poseNow ,currentRobotJConfig);
%     joints(:,i)=jointNow;
% end
% 
% %plot
% figure(8);
% 
% hold on
% axis([-1 1 -1 1 -0.1 1.5]);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jiontNow =joints(:,i);
%     %showRobot(myrobot,jointNow);
%     taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
%     drawnow;
% end
% showRobot(myrobot,currentRobotJConfig);
% hold on;
% 
% %310to320
% jointInit = ikJoints6;
% jointFinal = ikJoints8;
% 
% taskInit = T310;
% taskFinal = T320;
% distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
% initTime = 0;
% finalTime = (distance/toolSpeed) - initTime;
% trajTimes = initTime:timeStep:finalTime;
% numPoints=length(trajTimes);
% timeInterval = [trajTimes(1); trajTimes(end)];
% ctrlpoints = [jointInit',jointFinal'];
% jointArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
% poseArray=zeros(4,4,numPoints);
% for i=1:numPoints
%     jointNow=jointArray(:,i);
%     poseNow=fkine(myrobot, jointNow, endEffector, base);
%     poseArray(:,:,i)=poseNow;
% end
% 
% %task space
% [taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
% joints=zeros(numJoints,numPoints);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jointNow=ikine2(myrobot,endEffector, weights,poseNow ,currentRobotJConfig);
%     joints(:,i)=jointNow;
% end
% 
% %plot
% figure(8);
% 
% hold on
% axis([-1 1 -1 1 -0.1 1.5]);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jiontNow =joints(:,i);
%     %showRobot(myrobot,jointNow);
%     taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
%     drawnow;
% end
% showRobot(myrobot,currentRobotJConfig);
% hold on;
% 
% %320to32
% jointInit = ikJoints8;
% jointFinal = ikJoints7;
% 
% taskInit = T320;
% taskFinal = T32;
% distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
% initTime = 0;
% finalTime = (distance/toolSpeed) - initTime;
% trajTimes = initTime:timeStep:finalTime;
% numPoints=length(trajTimes);
% timeInterval = [trajTimes(1); trajTimes(end)];
% ctrlpoints = [jointInit',jointFinal'];
% jointArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
% poseArray=zeros(4,4,numPoints);
% for i=1:numPoints
%     jointNow=jointArray(:,i);
%     poseNow=fkine(myrobot, jointNow, endEffector, base);
%     poseArray(:,:,i)=poseNow;
% end
% 
% %task space
% [taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
% joints=zeros(numJoints,numPoints);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jointNow=ikine2(myrobot,endEffector, weights,poseNow ,currentRobotJConfig);
%     joints(:,i)=jointNow;
% end
% 
% %plot
% figure(8);
% 
% hold on
% axis([-1 1 -1 1 -0.1 1.5]);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jiontNow =joints(:,i);
%     %showRobot(myrobot,jointNow);
%     taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
%     drawnow;
% end
% showRobot(myrobot,currentRobotJConfig);
% hold on;
% 
% 
% %32toHOME
% jointInit = ikJoints7;
% jointFinal = ikJointsHome;
% 
% taskInit = T32;
% taskFinal = HOME;
% distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
% initTime = 0;
% finalTime = (distance/toolSpeed) - initTime;
% trajTimes = initTime:timeStep:finalTime;
% numPoints=length(trajTimes);
% timeInterval = [trajTimes(1); trajTimes(end)];
% ctrlpoints = [jointInit',jointFinal'];
% jointArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
% poseArray=zeros(4,4,numPoints);
% for i=1:numPoints
%     jointNow=jointArray(:,i);
%     poseNow=fkine(myrobot, jointNow, endEffector, base);
%     poseArray(:,:,i)=poseNow;
% end
% 
% 
% %task space
% [taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
% joints=zeros(numJoints,numPoints);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jointNow=ikine2(myrobot,endEffector, weights,poseNow ,currentRobotJConfig);
%     joints(:,i)=jointNow;
% end
% 
% %plot
% figure(8);
% 
% hold on
% axis([-1 1 -1 1 -0.1 1.5]);
% for i=1:length(trajTimes)
%     poseNow=taskWaypoints(:,:,i);
%     jiontNow =joints(:,i);
%     %showRobot(myrobot,jointNow);
%     taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
%     drawnow;
% end
% showRobot(myrobot,currentRobotJConfig);
% hold on;
% 
