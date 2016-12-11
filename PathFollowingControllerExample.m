%% Path Following for a Differential Drive Robot
%% Introduction
% This example demonstrates how to control a robot to follow a desired path
% using a robot motion model. The example uses the Pure Pursuit path following
% controller to drive a simulated robot along a predetermined path. A desired path is a
% set of waypoints defined explicitly or computed using a path planner (refer to
% <PathPlanningExample.html Path Planning example>). The Pure Pursuit
% path following controller for a simulated differential drive robot is created and
% computes the control commands to follow a given path. The computed control commands are
% used to drive the simulated robot along the desired trajectory to
% follow the desired path based on the Pure Pursuit controller.

%% Define waypoints
% Define a set of waypoints for the desired path for the robot
path = [2.00    1.00;
    1.25    1.75;
    5.25    8.25;
    7.25    8.75;
    11.75   10.75;
    12.00   10.00];
close all
% path = [2.00    1.00;
%     1.25    1.75;
%     5.25    3.25;
%     7.25    9.75;
%     11.75   10.75;
%     12.00   1.00];

%%
% Visualize the desired path
plot(path(:,1), path(:,2),'k--d')
xlim([0 13])
ylim([0 13])

%% Define robot motion model
% A robot motion model is needed when an external simulator or a physical
% robot is not available. The simulated motion model used in this example
% updates and returns the pose of the robot for given inputs. An external
% simulator or a physical robot will require a localization mechanism to
% provide an updated pose of the robot.
%
% Set the current location and the goal location of the robot as defined by the path
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);

%%
% Assume an initial robot orientation (the robot orientation is the angle
% between the robot heading and the positive X-axis, measured
% counterclockwise).
initialOrientation = 0;

%%
% Define the current pose for the robot [x y theta]
robotCurrentPose = [robotCurrentLocation initialOrientation];

%%
% Initialize a simulated robot object with the current pose. The simulated
% robot has kinematic equations for the motion of a two-wheeled robot.
% This simulated robot object can drive based on these equations of motion
% and the linear and angular velocity inputs. It also has plotting
% capabilities to display the robot's current location and draw the
% trajectory of the robot.
robot = ExampleHelperDifferentialDriveRobot(robotCurrentPose);

%% Define the path following controller
% Based on the path defined above and a robot motion model, you need a path
% following controller to drive the robot along the path. Create the path
% following controller using the  |<docid:robotics_ref.buoofp1-1 robotics.PurePursuit>|  object.
controller = robotics.PurePursuit

%%
% Use the path defined above to set the desired waypoints for the
% controller
controller.Waypoints = path;

%%
% Set the path following controller parameters. The desired linear
% velocity is set to 0.3 meters/second for this example.
controller.DesiredLinearVelocity = 0.3;

%%
% The maximum angular velocity acts as a saturation limit for rotational velocity, which is
% set at 2 radians/second for this example.
controller.MaxAngularVelocity = 2;

%%
% As a general rule, the lookahead distance should be larger than the desired
% linear velocity for a smooth path. The robot might cut corners when the
% lookahead distance is large. In contrast, a small lookahead distance can
% result in an unstable path following behavior. A value of 0.5 m was chosen
% for this example.
controller.LookaheadDistance = 0.5;

%% Using the path following controller, drive the robot over the desired waypoints
% The path following controller provides input control signals for the
% robot, which the robot uses to drive itself along the desired path.
%
% Define a goal radius, which is the desired distance threshold
% between the robot's final location and the goal location. Once the robot is
% within this distance from the goal, it will stop.
%
% Also, compute the current distance between the robot location and
% the goal location. This distance is continuously checked against the goal
% radius and the robot stops when this distance is less than the goal radius.
%
% Note that too small value of the goal radius may cause the robot to miss
% the goal, which may result in an unexpected behavior near the goal.
goalRadius = 0.1;
distanceToGoal = norm(robotCurrentLocation - robotGoal);

%%
% The |<docid:robotics_ref.buopp2z-1 step>| function computes control commands for the robot.
% Drive the robot using these control commands until it reaches within the
% goal radius. If you are using an external simulator or a physical robot,
% then the controller outputs should be applied to the robot and a localization
% system may be required to update the pose of the robot.
while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = step(controller, robot.CurrentPose);
    
    % Simulate the robot using the controller outputs.
    drive(robot, v, omega)
    
    % Extract current location information ([X,Y]) from the current pose of the
    % robot
    robotCurrentLocation = robot.CurrentPose(1:2);
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentLocation - robotGoal);
end

%%
% The simulated robot has reached the goal location using the path following
% controller along the desired path.

%% Using the path following controller along with PRM
% If the desired set of waypoints are computed by a path planner, the path
% following controller can be used in the same fashion.
%
% Import a simple map for which we want to compute a path
filePath = fullfile(fileparts(which('PathFollowingControllerExample')), 'data', 'exampleMaps.mat');
load(filePath)
map = robotics.BinaryOccupancyGrid(simpleMap, 2)

%%
% Display the map
show(map)

%%
% You can compute the |path| using the PRM path planning algorithm. See
% <PathPlanningExample.html Path Planning example> for details.
robotRadius = 0.2;
mapInflated = copy(map);
inflate(mapInflated,robotRadius);
prm = robotics.PRM(mapInflated);
prm.NumNodes = 100;
prm.ConnectionDistance = 10;

%%
% Find a path between the start and end location. Note that the |path| will
% be different due to the probabilistic nature of the PRM algorithm.
startLocation = [2.0 1.0];
endLocation = [12.0 10.0];
path = findpath(prm, startLocation, endLocation)

%%
% Display the path
hold on
show(prm, 'Map', 'off', 'Roadmap', 'off');
hold off

%%
% You defined a path following controller above which you can re-use
% for computing the control commands of a robot on this map. To
% re-use the controller and redefine the waypoints while keeping the other
% information the same, use the |<docid:robotics_ref.buopqdc-1 release>| function.
release(controller)
controller.Waypoints = path;

%%
% Set current location and the goal of the robot as defined by the path
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);

%%
% Assume an initial robot orientation
initialOrientation = 0;

%%
% Define the current pose for robot motion [x y theta]
robotCurrentPose = [robotCurrentLocation initialOrientation];

%%
% Re-create the simulated robot object with the current pose.
robot = ExampleHelperDifferentialDriveRobot(robotCurrentPose);

%%
% Compute distance to the goal location
distanceToGoal = norm(robotCurrentLocation - robotGoal);

%%
% Define a goal radius
goalRadius = 0.1;

%%
% Drive the robot using the controller output on the given map until it
% reaches the goal
while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = step(controller, robot.CurrentPose);
    
    % Simulate the robot using the controller outputs
    drive(robot, v, omega)
    
    % Extract current location information from the current pose
    robotCurrentLocation = robot.CurrentPose(1:2);
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentLocation - robotGoal);
end

%% Next Steps
%
% * Refer to the Path Planning example: <PathPlanningExample.html Path Planning in Cluttered Environment>
% * Refer to the Mapping example:
% <MapUpdateUsingSensorDataExample.html Updating an Occupancy Grid From Range Sensor Data>

displayEndOfDemoMessage(mfilename)
