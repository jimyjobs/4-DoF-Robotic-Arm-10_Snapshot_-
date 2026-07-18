%% ========================================================================
%  4-DOF ARTICULATED ROBOT - PICK AND PLACE DEMONSTRATION (WITH GRIPPER)
%
%  Matched to the actual Simscape Multibody model structure:
%      revolute_1 <- joint1_input    (Base rotation)
%      revolute_2 <- joint2_input    (Shoulder)
%      revolute_3 <- joint3_input    (Elbow)
%      revolute_4 <- joint4_input    (Tool orientation -> x0_8_cutter)
%      finger_1   <- gripper_input   (NEW: Prismatic joint, base-mounted
%                                     to x0_8_cutter)
%      finger_2   <- gripper_input * -1  (NEW: mirrored via a Gain block
%                                     in Simulink, so fingers close
%                                     symmetrically)
%
%  BLOCK DIAGRAM CHANGES REQUIRED (do this in Simulink before running):
%    1. Add two Prismatic Joint blocks near x0_8_cutter (Base port ->
%       x0_8_cutter frame, Follower port -> a small new Solid block
%       for each finger).
%    2. Set both joints' primitive axis to the same direction.
%    3. Add a Gain block set to -1 on finger_2's input so it moves
%       opposite to finger_1 (symmetric closing).
%    4. Add a "From Workspace" block named gripper_input, wired
%       directly into finger_1 and through the Gain block into finger_2.
%
%  Once that's done, this script drives all 5 inputs (joint1-4_input
%  plus gripper_input) for a real pick-and-place: the arm reaches to
%  the object, the gripper actually closes (gripper_input -> 0 m),
%  the object is held through the transfer, then the gripper opens
%  (gripper_input -> 0.01 m) to release it at the place location.
%
%  Task: pick an object from one location and place it at another.
% ========================================================================

clear;
clc;
close all;

%% ------------------------------------------------------------------
% SECTION 0: SETTINGS
% ------------------------------------------------------------------

model_name      = 'pkg_4_dof_robotic_arm_10_snapshot_2';
simulation_time = 26;
Ts              = 0.01;
time            = (0:Ts:simulation_time)';

%% ------------------------------------------------------------------
% SECTION 1: MOTION PHASES (11 waypoints)
% ------------------------------------------------------------------
% ADJUST these angles to match where your object and place location
% actually sit in your model / workspace (check against your Task 8
% reachability results before finalizing).

phase_time = [0; 3; 6; 9; 12; 15; 18; 21; 23; 24; 26];

phase_names = {
    'Home'
    'Wake Up'
    'Rotate Toward Object'
    'Reach / Descend to Object'
    'Grip Object (close gripper)'
    'Lift Object'
    'Transfer to Place'
    'Lower to Place'
    'Release Object (open gripper)'
    'Retract'
    'Return Home'
};

%             Home  WakeUp Rotate  Descend  Grip   Lift  Transfer Lower  Release Retract Home
Joint1_deg = [  0  ;   0  ;  40  ;   40   ;  40  ;  40 ;  -35   ; -35  ;  -35  ;  -35  ;  0  ];
Joint2_deg = [  0  ;  -5  ;  -5  ;  -45   ; -60  ; -30 ;  -30   ; -55  ;  -55  ;  -30  ;  0  ];
Joint3_deg = [  0  ;   5  ;   5  ;   40   ;  55  ;  25 ;   25   ;  45  ;   45  ;   20  ;  0  ];
Joint4_deg = [  0  ;   0  ;   0  ;  -25   ; -35  ; -35 ;  -35   ; -35  ;    0  ;    0  ;  0  ];

% Gripper opening, in METERS (matches a Prismatic Joint's SI units).
% 0.010 m = fingers fully open. 0.000 m = fingers fully closed on object.
%              Home   WakeUp  Rotate  Descend  Grip   Lift   Transfer  Lower  Release  Retract  Home
Gripper_m   = [0.010; 0.010;  0.010;  0.010;  0.000; 0.000; 0.000;    0.000; 0.010;   0.010;   0.010];

% Reading the table above as a story:
%   Home->WakeUp->Rotate   : arm lifts slightly, base turns 40 deg toward object, gripper stays open
%   Reach/Descend          : shoulder/elbow extend down to the object, tool orients, gripper still open
%   Grip (close gripper)   : deepest reach, gripper_input closes from 0.010 m to 0.000 m on the object
%   Lift                   : arm raises WHILE gripper stays closed (object is actually held)
%   Transfer               : base rotates from +40 to -35 deg toward the place location, still gripping
%   Lower to Place         : arm descends to the place surface, gripper STILL closed
%   Release (open gripper) : gripper_input opens back to 0.010 m = object physically released
%   Retract                : arm lifts away from the now-placed object
%   Return Home            : all joints back to 0, gripper open

%% ------------------------------------------------------------------
% SECTION 2: PRINT THE TRAJECTORY TABLE (for the report)
% ------------------------------------------------------------------

fprintf('=======================================================================\n');
fprintf(' PICK-AND-PLACE JOINT TRAJECTORY TABLE\n');
fprintf('=======================================================================\n');
fprintf('%-6s %-30s %8s %8s %8s %8s %8s\n', ...
    'Time', 'Phase', 'J1(deg)', 'J2(deg)', 'J3(deg)', 'J4(deg)', 'Grip(m)');
for k = 1:numel(phase_time)
    fprintf('%-6.0f %-30s %8.1f %8.1f %8.1f %8.1f %8.3f\n', ...
        phase_time(k), phase_names{k}, ...
        Joint1_deg(k), Joint2_deg(k), Joint3_deg(k), Joint4_deg(k), Gripper_m(k));
end
fprintf('=======================================================================\n\n');

%% ------------------------------------------------------------------
% SECTION 3: SMOOTH TRAJECTORY GENERATION (PCHIP)
% ------------------------------------------------------------------

Joint1 = deg2rad(interp1(phase_time, Joint1_deg, time, 'pchip'));
Joint2 = deg2rad(interp1(phase_time, Joint2_deg, time, 'pchip'));
Joint3 = deg2rad(interp1(phase_time, Joint3_deg, time, 'pchip'));
Joint4 = deg2rad(interp1(phase_time, Joint4_deg, time, 'pchip'));

% Gripper: use 'pchip' too, but clamp to [0, 0.010] since overshoot past
% fully-open or fully-closed would be physically meaningless.
Gripper = interp1(phase_time, Gripper_m, time, 'pchip');
Gripper = min(max(Gripper, 0), 0.010);

%% ------------------------------------------------------------------
% SECTION 4: FROM WORKSPACE VARIABLES (names match the block diagram)
% ------------------------------------------------------------------

joint1_input  = [time Joint1];
joint2_input  = [time Joint2];
joint3_input  = [time Joint3];
joint4_input  = [time Joint4];
gripper_input = [time Gripper];

% Since this is a top-level script (not a function), these variables
% land directly in the base workspace, so revolute_1..4's and the new
% gripper Prismatic Joints' "From Workspace" blocks will list
% joint1_input..joint4_input and gripper_input automatically.

%% ------------------------------------------------------------------
% SECTION 5: VELOCITY AND ACCELERATION
% ------------------------------------------------------------------

vel1 = gradient(Joint1, Ts);
vel2 = gradient(Joint2, Ts);
vel3 = gradient(Joint3, Ts);
vel4 = gradient(Joint4, Ts);

acc1 = gradient(vel1, Ts);
acc2 = gradient(vel2, Ts);
acc3 = gradient(vel3, Ts);
acc4 = gradient(vel4, Ts);

%% ------------------------------------------------------------------
% SECTION 6: PLOT JOINT TRAJECTORIES
% ------------------------------------------------------------------

figure('Color', 'white');
plot(time, rad2deg(Joint1), 'LineWidth', 2); hold on;
plot(time, rad2deg(Joint2), 'LineWidth', 2);
plot(time, rad2deg(Joint3), 'LineWidth', 2);
plot(time, rad2deg(Joint4), 'LineWidth', 2);
grid on;
xlabel('Time (Seconds)');
ylabel('Joint Angle (Degrees)');
title('Pick-and-Place: Joint Trajectories');
legend('Joint 1 (Base)','Joint 2 (Shoulder)','Joint 3 (Elbow)','Joint 4 (Tool)', ...
    'Location','best');

for k = 1:numel(phase_time)
    xline(phase_time(k), 'k--');
    text(phase_time(k), 80, phase_names{k}, 'Rotation', 90, 'FontSize', 7);
end

saveas(gcf, 'PickPlace_Joint_Angles.png');

%% ------------------------------------------------------------------
% SECTION 6b: PLOT GRIPPER OPENING
% ------------------------------------------------------------------

figure('Color', 'white');
plot(time, Gripper*1000, 'LineWidth', 2, 'Color', [0.85 0.33 0.10]);
grid on;
xlabel('Time (Seconds)'); ylabel('Gripper Opening (mm)');
title('Pick-and-Place: Gripper Opening vs Time');
ylim([-1 11]);
for k = 1:numel(phase_time)
    xline(phase_time(k), 'k--');
end
saveas(gcf, 'PickPlace_Gripper_Opening.png');

%% ------------------------------------------------------------------
% SECTION 7: PLOT VELOCITY
% ------------------------------------------------------------------

figure('Color', 'white');
plot(time, vel1, 'LineWidth', 2); hold on;
plot(time, vel2, 'LineWidth', 2);
plot(time, vel3, 'LineWidth', 2);
plot(time, vel4, 'LineWidth', 2);
grid on;
xlabel('Time (Seconds)'); ylabel('Angular Velocity (rad/s)');
title('Pick-and-Place: Joint Velocities');
legend('J1','J2','J3','J4');
saveas(gcf, 'PickPlace_Joint_Velocities.png');

%% ------------------------------------------------------------------
% SECTION 8: PLOT ACCELERATION
% ------------------------------------------------------------------

figure('Color', 'white');
plot(time, acc1, 'LineWidth', 2); hold on;
plot(time, acc2, 'LineWidth', 2);
plot(time, acc3, 'LineWidth', 2);
plot(time, acc4, 'LineWidth', 2);
grid on;
xlabel('Time (Seconds)'); ylabel('Angular Acceleration (rad/s^2)');
title('Pick-and-Place: Joint Accelerations');
legend('J1','J2','J3','J4');
saveas(gcf, 'PickPlace_Joint_Accelerations.png');

%% ------------------------------------------------------------------
% SECTION 9: MOTION SUMMARY
% ------------------------------------------------------------------

disp('===================================================')
disp('   4-DOF ROBOT - PICK AND PLACE DEMONSTRATION')
disp('===================================================')
disp('')
for k = 1:numel(phase_names)
    fprintf('%2d. %s (t = %.0f s)\n', k, phase_names{k}, phase_time(k));
end
disp('')
fprintf('Simulation Time : %d seconds\n', simulation_time);
disp('Interpolation   : PCHIP')
disp('Robot Type      : 4-DOF Articulated Arm')
disp('===================================================')

%% ------------------------------------------------------------------
% SECTION 10: OPEN MODEL, SET TIME, RUN
% ------------------------------------------------------------------
if ~bdIsLoaded(model_name)
    open_system(model_name);
end

set_param(model_name, 'StopTime', num2str(simulation_time));

fprintf('\nRunning Pick-and-Place Simulation...\n');
try
    sim(model_name);
    fprintf('Simulation finished successfully.\n');
catch ME
    fprintf('Simulation failed: %s\n', ME.message);
end

save('PickPlace_Trajectory.mat', ...
    'joint1_input','joint2_input','joint3_input','joint4_input','gripper_input');

%% ------------------------------------------------------------------
% SCREENSHOT CHECKLIST FOR THE REPORT
% ------------------------------------------------------------------
% Capture the 3D animation at these times, matching the phase table:
%   t = 0 s   -> Home, gripper open
%   t = 6 s   -> Rotated toward object, gripper open
%   t = 12 s  -> Gripper CLOSED on the object (deepest reach)
%   t = 18 s  -> Mid-transfer, base rotated toward place, gripper still closed
%   t = 21 s  -> Lowered to place position, gripper still closed
%   t = 23 s  -> Gripper OPEN again, object released
%   t = 26 s  -> Returned home