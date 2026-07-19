%% ========================================================================
% ROBOTICS AND AUTOMATION
%
% CAD-Based Robot Structure Identification
% and Autonomous Cutting Task Demonstration
%
% Robot:
% pkg_4_dof_robotic_arm_10_snapshot_2
%
% End Effector:
% Rotary Cutting Tool
%
% Student:
% _______________________
%
% ========================================================================

clear
clc
close all

%% ========================================================================
% SIMULATION PARAMETERS
% ========================================================================

modelName = 'pkg_4_dof_robotic_arm_10_snapshot_2';

simulationTime = 20;          % seconds

Ts = 0.01;                    % sample time

time = (0:Ts:simulationTime)';

%% ========================================================================
% VERIFY MODEL EXISTS
% ========================================================================

if ~bdIsLoaded(modelName)

    open_system(modelName);

end

set_param(modelName,'StopTime',num2str(simulationTime));

disp('Robot model loaded successfully.')

%% ========================================================================
% ROBOT INFORMATION
% ========================================================================

Robot.Name = '4-DOF Articulated Cutting Robot';
Robot.Type = 'Articulated Manipulator';
Robot.DOF  = 4;

Robot.EndEffector = 'Rotary Cutter';

Robot.Task = 'Autonomous Cutting Demonstration';

%% ========================================================================
% CUTTING TASK DESCRIPTION
%
% Phase 1  : Home
% Phase 2  : Approach Workpiece
% Phase 3  : Align Cutter
% Phase 4  : Contact Material
% Phase 5  : Cut Start
% Phase 6  : Cut Mid 1
% Phase 7  : Cut Mid 2
% Phase 8  : Cut End
% Phase 9  : Retract Cutter
% Phase 10 : Safe Position
% Phase 11 : Return Home
%
% ========================================================================

motionTime = [

     0
     2
     5
     8
     9.5
    11
    12.5
    14
    16
    18
    20

];

motionName = {

'Home'
'Approach'
'Align Cutter'
'Contact Material'
'Cut Start'
'Cut Mid 1'
'Cut Mid 2'
'Cut End'
'Retract Cutter'
'Safe Position'
'Return Home'

};

%% ========================================================================
% CUTTER OPERATION
%
% 0 = Cutter OFF
% 1 = Cutter ON
%
% ========================================================================

ToolState = [

0
0
0
1
1
1
1
1
0
0
0

];

%% ========================================================================
% FEED RATE
%
% Relative cutting feed
%
% ========================================================================

FeedRate = [

0
25
15
5
10
10
10
8
20
25
0

];

%% ========================================================================
% CUTTER ANGLE
%
% Desired cutter orientation (degrees)
%
% ========================================================================

CutterAngle = [

0
-10
-25
-40
-40
-40
-40
-40
-20
-10
0

];

%% ========================================================================
% JOINT TARGETS (Degrees)
%
% Joint 1 = Base Rotation
% Joint 2 = Shoulder
% Joint 3 = Elbow
% Joint 4 = Wrist / Cutter Orientation
%
% ========================================================================

Joint1_deg = [

     0
    20
    30
    32
    34
    36
    39
    42
    25
    12
     0

];

Joint2_deg = [

     0
   -10
   -28
   -42
   -41
   -40
   -39
   -38
   -20
   -10
     0

];

Joint3_deg = [

     0
    15
    38
    55
    58
    62
    66
    70
    25
    12
     0

];

Joint4_deg = [

     0
   -10
   -25
   -40
   -40
   -40
   -40
   -40
   -18
   -10
     0

];

%% ========================================================================
% DISPLAY TASK SUMMARY
% ========================================================================

disp(' ')
disp('========================================================')
disp('AUTONOMOUS CUTTING TASK')
disp('========================================================')

fprintf('Robot                : %s\n',Robot.Name);
fprintf('End Effector         : %s\n',Robot.EndEffector);
fprintf('Degrees of Freedom   : %d\n',Robot.DOF);
fprintf('Simulation Time      : %.1f seconds\n',simulationTime);
fprintf('Motion Phases        : %d\n',length(motionName));

disp(' ')
disp('Cutting Sequence')

for k = 1:length(motionName)

    if ToolState(k) == 1
        CutterStatus = 'ON';
    else
        CutterStatus = 'OFF';
    end

    fprintf('%2d. %-20s   Cutter: %s\n', ...
        k,...
        motionName{k},...
        CutterStatus);

end

disp('========================================================')

%% ========================================================================
% PART 2
% CUTTING TRAJECTORY GENERATION
%
% This section generates smooth trajectories for:
%
%   • Robot joints
%   • Cutter orientation
%   • Cutter activation
%   • Feed rate
%
% Piecewise Cubic Hermite Interpolation (PCHIP)
% is used to eliminate abrupt robot motion.
%
% ========================================================================

disp(' ')
disp('========================================================')
disp('GENERATING CUTTING TRAJECTORIES')
disp('========================================================')

%% ========================================================================
% GENERATE SMOOTH JOINT TRAJECTORIES
% ========================================================================

Joint1_profile_deg = interp1( ...
    motionTime,...
    Joint1_deg,...
    time,...
    'pchip');

Joint2_profile_deg = interp1( ...
    motionTime,...
    Joint2_deg,...
    time,...
    'pchip');

Joint3_profile_deg = interp1( ...
    motionTime,...
    Joint3_deg,...
    time,...
    'pchip');

Joint4_profile_deg = interp1( ...
    motionTime,...
    Joint4_deg,...
    time,...
    'pchip');

disp('Joint trajectories generated successfully.')

%% ========================================================================
% GENERATE TOOL ORIENTATION TRAJECTORY
% ========================================================================

CutterAngle_profile = interp1( ...
    motionTime,...
    CutterAngle,...
    time,...
    'pchip');

disp('Cutter angle trajectory generated.')

%% ========================================================================
% GENERATE CUTTER STATE
%
% Zero-Order Hold
%
% OFF remains OFF
% ON remains ON
%
% ========================================================================

CutterState = interp1( ...
    motionTime,...
    ToolState,...
    time,...
    'previous');

CutterState(isnan(CutterState)) = 0;

disp('Cutter activation profile generated.')

%% ========================================================================
% GENERATE FEED RATE
% ========================================================================

FeedRate_profile = interp1( ...
    motionTime,...
    FeedRate,...
    time,...
    'pchip');

disp('Feed-rate profile generated.')

%% ========================================================================
% CONVERT JOINTS TO RADIANS
%
% Simscape Revolute Joints require radians.
% ========================================================================

Joint1 = deg2rad(Joint1_profile_deg);
Joint2 = deg2rad(Joint2_profile_deg);
Joint3 = deg2rad(Joint3_profile_deg);
Joint4 = deg2rad(Joint4_profile_deg);

disp('Joint angles converted to radians.')

%% ========================================================================
% VERIFY GENERATED TRAJECTORIES
% ========================================================================

disp(' ')
disp('Verifying generated trajectories...')

assert(length(Joint1)==length(time));
assert(length(Joint2)==length(time));
assert(length(Joint3)==length(time));
assert(length(Joint4)==length(time));

assert(length(CutterState)==length(time));

assert(length(FeedRate_profile)==length(time));

assert(length(CutterAngle_profile)==length(time));

disp('Trajectory verification successful.')

%% ========================================================================
% DISPLAY JOINT LIMITS
% ========================================================================

disp(' ')
disp('Joint Motion Limits')

fprintf('Joint 1 : %.1f°  -> %.1f°\n', ...
    min(Joint1_profile_deg), ...
    max(Joint1_profile_deg));

fprintf('Joint 2 : %.1f°  -> %.1f°\n', ...
    min(Joint2_profile_deg), ...
    max(Joint2_profile_deg));

fprintf('Joint 3 : %.1f°  -> %.1f°\n', ...
    min(Joint3_profile_deg), ...
    max(Joint3_profile_deg));

fprintf('Joint 4 : %.1f°  -> %.1f°\n', ...
    min(Joint4_profile_deg), ...
    max(Joint4_profile_deg));

%% ========================================================================
% TRAJECTORY SUMMARY
% ========================================================================

disp(' ')
disp('========================================================')
disp('CUTTING TRAJECTORY SUMMARY')
disp('========================================================')

fprintf('Simulation Time      : %.2f s\n',simulationTime);
fprintf('Sample Time          : %.3f s\n',Ts);
fprintf('Trajectory Samples   : %d\n',length(time));
fprintf('Motion Phases        : %d\n',length(motionName));
fprintf('Interpolation        : PCHIP\n');

disp('========================================================')

%% ========================================================================
% CREATE REFERENCE TABLE
% ========================================================================

TrajectoryTable = table( ...
    motionTime(:), ...
    Joint1_deg(:), ...
    Joint2_deg(:), ...
    Joint3_deg(:), ...
    Joint4_deg(:), ...
    ToolState(:), ...
    FeedRate(:), ...
    CutterAngle(:), ...
    'VariableNames', ...
    {'Time',...
    'Joint1',...
    'Joint2',...
    'Joint3',...
    'Joint4',...
    'Cutter',...
    'FeedRate',...
    'CutterAngle'});

disp(' ')
disp('REFERENCE CUTTING TRAJECTORY')

disp(TrajectoryTable)

disp(' ')
disp('Part 2 completed successfully.')

%% ========================================================================
% PART 3
% PREPARE SIMULINK INPUT SIGNALS
%
% This section converts the generated trajectories into the format
% required by Simulink "From Workspace" blocks.
%
% Generated Signals
%
%   Joint 1
%   Joint 2
%   Joint 3
%   Joint 4
%   Cutter State
%   Feed Rate
%   Cutter Angle
%
% ========================================================================

disp(' ')
disp('========================================================')
disp('PREPARING SIMULINK INPUT SIGNALS')
disp('========================================================')

%% ========================================================================
% CREATE FROM WORKSPACE MATRICES
% ========================================================================

joint1_input = [time Joint1];

joint2_input = [time Joint2];

joint3_input = [time Joint3];

joint4_input = [time Joint4];

cutter_input = [time CutterState];

feedrate_input = [time FeedRate_profile];

cutterangle_input = [time CutterAngle_profile];

disp('Workspace input matrices created successfully.')

%% ========================================================================
% VALIDATE MATRIX FORMAT
% ========================================================================

disp(' ')
disp('Validating workspace matrices...')

WorkspaceSignals = {

joint1_input
joint2_input
joint3_input
joint4_input
cutter_input
feedrate_input
cutterangle_input

};

SignalNames = {

'Joint1'
'Joint2'
'Joint3'
'Joint4'
'Cutter'
'Feed Rate'
'Cutter Angle'

};

for k = 1:length(WorkspaceSignals)

    Signal = WorkspaceSignals{k};

    assert(size(Signal,2)==2,...
        sprintf('%s must contain exactly two columns.',...
        SignalNames{k}));

    assert(isreal(Signal),...
        sprintf('%s contains invalid values.',...
        SignalNames{k}));

end

disp('All workspace matrices are valid.')

%% ========================================================================
% DISPLAY MATRIX INFORMATION
% ========================================================================

disp(' ')
disp('Workspace Matrix Sizes')

fprintf('Joint1        : %d x %d\n',size(joint1_input));

fprintf('Joint2        : %d x %d\n',size(joint2_input));

fprintf('Joint3        : %d x %d\n',size(joint3_input));

fprintf('Joint4        : %d x %d\n',size(joint4_input));

fprintf('Cutter State  : %d x %d\n',size(cutter_input));

fprintf('Feed Rate     : %d x %d\n',size(feedrate_input));

fprintf('Cutter Angle  : %d x %d\n',size(cutterangle_input));

%% ========================================================================
% SAVE TRAJECTORY FILE
% ========================================================================

save( ...
'RobotTrajectory.mat',...
'joint1_input',...
'joint2_input',...
'joint3_input',...
'joint4_input',...
'cutter_input',...
'feedrate_input',...
'cutterangle_input');

disp('RobotTrajectory.mat saved successfully.')

%% ========================================================================
% VERIFY VARIABLES EXIST
% ========================================================================

disp(' ')
disp('Checking MATLAB workspace...')

RequiredVariables = {

'joint1_input'
'joint2_input'
'joint3_input'
'joint4_input'
'cutter_input'
'feedrate_input'
'cutterangle_input'

};

for k = 1:length(RequiredVariables)

    if evalin('base',...
        sprintf('exist(''%s'',''var'')',RequiredVariables{k}))

        fprintf('[OK] %s found.\n',RequiredVariables{k});

    else

        error('%s is missing.',RequiredVariables{k});

    end

end

%% ========================================================================
% DISPLAY SAMPLE DATA
% ========================================================================

disp(' ')
disp('Preview of Joint1 Input')

disp(joint1_input(1:10,:))

disp(' ')

disp('Preview of Cutter State')

disp(cutter_input(1:20,:))

disp(' ')

disp('Preview of Feed Rate')

disp(feedrate_input(1:20,:))

%% ========================================================================
% PART SUMMARY
% ========================================================================

disp(' ')
disp('========================================================')
disp('SIMULINK INPUT PREPARATION COMPLETE')
disp('========================================================')

fprintf('Signals Prepared : %d\n',7);

fprintf('Simulation Ready : YES\n');

fprintf('Trajectory File  : RobotTrajectory.mat\n');

disp('========================================================')

%% ========================================================================
% PART 4
% CUTTING TRAJECTORY ANALYSIS
%
% This section analyses:
%
%   1. Joint Position
%   2. Joint Velocity
%   3. Joint Acceleration
%   4. Cutter Activity
%   5. Feed Rate
%
% ========================================================================

disp(' ')
disp('========================================================')
disp('CUTTING TRAJECTORY ANALYSIS')
disp('========================================================')

%% ========================================================================
% COMPUTE JOINT VELOCITIES
% ========================================================================

Joint1_velocity = gradient(Joint1,Ts);

Joint2_velocity = gradient(Joint2,Ts);

Joint3_velocity = gradient(Joint3,Ts);

Joint4_velocity = gradient(Joint4,Ts);

%% ========================================================================
% COMPUTE JOINT ACCELERATIONS
% ========================================================================

Joint1_acceleration = gradient(Joint1_velocity,Ts);

Joint2_acceleration = gradient(Joint2_velocity,Ts);

Joint3_acceleration = gradient(Joint3_velocity,Ts);

Joint4_acceleration = gradient(Joint4_velocity,Ts);

disp('Velocity and acceleration calculated.')

%% ========================================================================
% FIGURE 1
% JOINT POSITION
% ========================================================================

figure( ...
'Name','Joint Position', ...
'Color','white', ...
'NumberTitle','off');

plot(time,rad2deg(Joint1),'LineWidth',2)
hold on

plot(time,rad2deg(Joint2),'LineWidth',2)

plot(time,rad2deg(Joint3),'LineWidth',2)

plot(time,rad2deg(Joint4),'LineWidth',2)

grid on
box on

xlabel('Time (Seconds)')

ylabel('Joint Angle (Degrees)')

title('Joint Position During Cutting Task')

legend( ...
'Base',...
'Shoulder',...
'Elbow',...
'Wrist',...
'Location','best')

for k = 1:length(motionTime)

    xline(motionTime(k),'k--');

end

xline(8,'r','Cut Start');

xline(14,'r','Cut End');

saveas(gcf,'Figure_01_Joint_Position.png');

%% ========================================================================
% FIGURE 2
% JOINT VELOCITY
% ========================================================================

figure( ...
'Name','Joint Velocity', ...
'Color','white', ...
'NumberTitle','off');

plot(time,Joint1_velocity,'LineWidth',2)
hold on

plot(time,Joint2_velocity,'LineWidth',2)

plot(time,Joint3_velocity,'LineWidth',2)

plot(time,Joint4_velocity,'LineWidth',2)

grid on
box on

xlabel('Time (Seconds)')

ylabel('Angular Velocity (rad/s)')

title('Joint Velocity During Cutting')

legend( ...
'Base',...
'Shoulder',...
'Elbow',...
'Wrist',...
'Location','best')

for k = 1:length(motionTime)

    xline(motionTime(k),'k--');

end

xline(8,'r','Cut Start');

xline(14,'r','Cut End');

saveas(gcf,'Figure_02_Joint_Velocity.png');

%% ========================================================================
% FIGURE 3
% JOINT ACCELERATION
% ========================================================================

figure( ...
'Name','Joint Acceleration', ...
'Color','white', ...
'NumberTitle','off');

plot(time,Joint1_acceleration,'LineWidth',2)
hold on

plot(time,Joint2_acceleration,'LineWidth',2)

plot(time,Joint3_acceleration,'LineWidth',2)

plot(time,Joint4_acceleration,'LineWidth',2)

grid on
box on

xlabel('Time (Seconds)')

ylabel('Angular Acceleration (rad/s^2)')

title('Joint Acceleration During Cutting')

legend( ...
'Base',...
'Shoulder',...
'Elbow',...
'Wrist',...
'Location','best')

for k = 1:length(motionTime)

    xline(motionTime(k),'k--');

end

xline(8,'r','Cut Start');

xline(14,'r','Cut End');

saveas(gcf,'Figure_03_Joint_Acceleration.png');

%% ========================================================================
% FIGURE 4
% CUTTER ACTIVITY
% ========================================================================

figure( ...
'Name','Cutter Activity', ...
'Color','white', ...
'NumberTitle','off');

stairs(time,CutterState,'LineWidth',2)

grid on
box on

ylim([-0.2 1.2])

xlabel('Time (Seconds)')

ylabel('Cutter State')

yticks([0 1])

yticklabels({'OFF','ON'})

title('Cutter Operation Timeline')

saveas(gcf,'Figure_04_Cutter_Activity.png');

%% ========================================================================
% FIGURE 5
% FEED RATE
% ========================================================================

figure( ...
'Name','Feed Rate', ...
'Color','white', ...
'NumberTitle','off');

plot(time,FeedRate_profile,'LineWidth',2)

grid on
box on

xlabel('Time (Seconds)')

ylabel('Relative Feed Rate')

title('Cutting Feed Rate')

saveas(gcf,'Figure_05_Feed_Rate.png');

%% ========================================================================
% TRAJECTORY STATISTICS
% ========================================================================

disp(' ')
disp('========================================================')
disp('CUTTING TRAJECTORY STATISTICS')
disp('========================================================')

fprintf('Maximum Base Angle        : %.2f deg\n', ...
max(rad2deg(Joint1)));

fprintf('Maximum Shoulder Angle    : %.2f deg\n', ...
max(rad2deg(Joint2)));

fprintf('Maximum Elbow Angle       : %.2f deg\n', ...
max(rad2deg(Joint3)));

fprintf('Maximum Wrist Angle       : %.2f deg\n', ...
max(rad2deg(Joint4)));

fprintf('\n');

fprintf('Maximum Base Velocity     : %.3f rad/s\n', ...
max(abs(Joint1_velocity)));

fprintf('Maximum Shoulder Velocity : %.3f rad/s\n', ...
max(abs(Joint2_velocity)));

fprintf('Maximum Elbow Velocity    : %.3f rad/s\n', ...
max(abs(Joint3_velocity)));

fprintf('Maximum Wrist Velocity    : %.3f rad/s\n', ...
max(abs(Joint4_velocity)));

fprintf('\n');

fprintf('Maximum Base Acceleration     : %.3f rad/s^2\n', ...
max(abs(Joint1_acceleration)));

fprintf('Maximum Shoulder Acceleration : %.3f rad/s^2\n', ...
max(abs(Joint2_acceleration)));

fprintf('Maximum Elbow Acceleration    : %.3f rad/s^2\n', ...
max(abs(Joint3_acceleration)));

fprintf('Maximum Wrist Acceleration    : %.3f rad/s^2\n', ...
max(abs(Joint4_acceleration)));

fprintf('\n');

fprintf('Cutting Duration : %.1f seconds\n',14-8);

fprintf('Motion Phases    : %d\n',length(motionName));

disp('========================================================')

disp(' ')
disp('Part 4 completed successfully.')

%% ========================================================================
% PART 5
% ROBOT CUTTING SIMULATION
%
% This section:
%
%   1. Verifies required variables
%   2. Loads the Simulink model
%   3. Executes the cutting simulation
%   4. Displays simulation statistics
%   5. Saves all generated results
%
% ========================================================================

disp(' ')
disp('========================================================')
disp('STARTING AUTONOMOUS CUTTING SIMULATION')
disp('========================================================')

%% ========================================================================
% VERIFY REQUIRED VARIABLES
% ========================================================================

RequiredVariables = {

'joint1_input'
'joint2_input'
'joint3_input'
'joint4_input'
'cutter_input'
'feedrate_input'
'cutterangle_input'

};

disp('Checking required workspace variables...')

for k = 1:length(RequiredVariables)

    if evalin('base', ...
            sprintf('exist(''%s'',''var'')',RequiredVariables{k}))

        fprintf('[OK] %s\n',RequiredVariables{k});

    else

        error('Missing required variable: %s', ...
            RequiredVariables{k});

    end

end

disp('All required variables are available.')

%% ========================================================================
% LOAD MODEL
% ========================================================================

disp(' ')
disp('Opening Simulink model...')

if ~bdIsLoaded(modelName)

    open_system(modelName);

end

disp('Model loaded successfully.')

%% ========================================================================
% CONFIGURE SIMULATION
% ========================================================================

set_param(modelName,...
    'StopTime',num2str(simulationTime));

disp('Simulation parameters configured.')

%% ========================================================================
% RUN SIMULATION
% ========================================================================

disp(' ')
disp('Executing robotic cutting task...')

tic

SimulationOutput = sim(modelName);

ElapsedTime = toc;

disp('Simulation completed successfully.')

%% ========================================================================
% SAVE SIMULATION OUTPUT
% ========================================================================

save( ...
'RobotSimulationOutput.mat',...
'SimulationOutput');

disp('Simulation output saved.')

%% ========================================================================
% CUTTING TASK EXECUTION SUMMARY
% ========================================================================

disp(' ')
disp('========================================================')
disp('CUTTING TASK EXECUTION')
disp('========================================================')

for k = 1:length(motionName)

    if ToolState(k)==1
        CutterStatus='ON';
    else
        CutterStatus='OFF';
    end

    fprintf('%2d. %-20s  Cutter : %s\n',...
        k,...
        motionName{k},...
        CutterStatus);

end

disp('========================================================')

%% ========================================================================
% CUTTING OPERATION REPORT
% ========================================================================

disp(' ')
disp('CUTTING OPERATION')

CutStart = motionTime(find(ToolState==1,1,'first'));

CutEnd = motionTime(find(ToolState==1,1,'last'));

fprintf('Cut Start Time     : %.2f s\n',CutStart);

fprintf('Cut End Time       : %.2f s\n',CutEnd);

fprintf('Cutting Duration   : %.2f s\n', ...
    CutEnd-CutStart);

fprintf('Maximum Feed Rate  : %.2f\n', ...
    max(FeedRate));

fprintf('Maximum Cutter Angle : %.2f deg\n', ...
    max(abs(CutterAngle)));

%% ========================================================================
% SIMULATION SUMMARY
% ========================================================================

disp(' ')
disp('========================================================')
disp('SIMULATION SUMMARY')
disp('========================================================')

fprintf('Robot Name           : %s\n', ...
    Robot.Name);

fprintf('Robot Type           : %s\n', ...
    Robot.Type);

fprintf('Degrees of Freedom   : %d\n', ...
    Robot.DOF);

fprintf('End Effector         : %s\n', ...
    Robot.EndEffector);

fprintf('Task                 : %s\n', ...
    Robot.Task);

fprintf('Simulation Time      : %.2f seconds\n', ...
    simulationTime);

fprintf('Sampling Time        : %.3f seconds\n', ...
    Ts);

fprintf('Trajectory Samples   : %d\n', ...
    length(time));

fprintf('Motion Phases        : %d\n', ...
    length(motionName));

fprintf('Execution Time       : %.2f seconds\n', ...
    ElapsedTime);

disp('========================================================')

%% ========================================================================
% SAVE MATLAB WORKSPACE
% ========================================================================

save('RobotProjectWorkspace.mat');

disp('Workspace saved successfully.')

%% ========================================================================
% GENERATED FILES
% ========================================================================

disp(' ')
disp('Generated Files')

GeneratedFiles = {

'RobotTrajectory.mat'
'RobotSimulationOutput.mat'
'RobotProjectWorkspace.mat'
'Figure_01_Joint_Position.png'
'Figure_02_Joint_Velocity.png'
'Figure_03_Joint_Acceleration.png'
'Figure_04_Cutter_Activity.png'
'Figure_05_Feed_Rate.png'

};

for k = 1:length(GeneratedFiles)

    fprintf('%s\n',GeneratedFiles{k});

end

%% ========================================================================
% FINAL MESSAGE
% ========================================================================

disp(' ')
disp('========================================================')
disp('PROJECT COMPLETED SUCCESSFULLY')
disp('========================================================')

disp('The robotic cutting trajectory')
disp('has been successfully executed.')

disp('The cutter approached the workpiece,')

disp('performed the programmed cutting task,')

disp('retracted safely,')

disp('and returned to the home position.')

disp('========================================================')