function cross_BOD()

% Set up syms and global constants
beta = []; % Our independent variable
t = []; % Time variable
syms beta t;

% Equation of the bridge
R = 4*[0.396*cos(2.65*(beta*t+1.4));...
       -0.99*sin(beta*t+1.4);...
       0];
   
T = diff(R, t);
T_hat = T / norm(T); % Tangent unit vector

dT_dt = diff(T_hat, t); 
omega = cross(T_hat, dT_dt); % Angular velocity vector [0; 0; w]

v = norm(T); % Speed
d = .235; % Wheel-base of NEATO

% Left and right wheel speeds
V_L = v - omega(3)*d/2;
V_R = v + omega(3)*d/2;

% Final value for beta
beta_num = .2;

% Final position on the bridge (used for telling the NEATO to stop)
final_r = double(subs(R, {beta, t}, {3.2, 1}));

% Set up wheel velocity and position data streams
pubvel = rospublisher('raw_vel');
sub_states = rossubscriber('/gazebo/model_states', 'gazebo_msgs/ModelStates');

% stop the robot if it's going right now
stopMsg = rosmessage(pubvel);
stopMsg.Data = [0 0];
send(pubvel, stopMsg);


% The next block places the NEATO at the beginning of the bridge
message = rosmessage(pubvel);
send(pubvel, message);
bridgeStart = double(subs(R,{beta, t},{beta_num, 0}));
startingThat = double(subs(T_hat,{beta, t},{beta_num, 0}));
placeNeato(bridgeStart(1),  bridgeStart(2), startingThat(1), startingThat(2));

% Start collecting data
% collectDataset_sim("bod.mat");

% Wait for the NEATO to fall
pause(2);

% Set start time to be now
start = rostime('now');

while 1
    current = rostime('now');
    elapsed = current - start; % Figure out how much time has passed since we started
    
    % Calculated real values of left and right wheel velocities
    V_L_num = double(subs(V_L, {beta, t}, {beta_num, elapsed.seconds}));
    V_R_num = double(subs(V_R, {beta, t}, {beta_num, elapsed.seconds}));

    % Send the left and right wheel velocities
    message.Data = [V_L_num, V_R_num];
    send(pubvel, message);
    
    % Chill for a sec
    pause(.1);
    
    % Check to see if we are at the end of the bridge
    
    % Get current position
    [pos_x, pos_y] = getNeatoPosition(receive(sub_states));
    real_r = [pos_x, pos_y, 0]';
    
    % Get distance from the end of the bridge
    delta_r = real_r - final_r;
    disp(norm(delta_r))
    
    % If we're close to the end, stop, and do_a_dance()
    if norm(delta_r) < .5
        message.Data = [0.0, 0.0];
        send(pubvel, message);
        
        pause(1);
        do_a_dance();
        return
    end
end

    % For simulated Neatos only:
    % Place the Neato in the specified x, y position and specified heading vector.
    function placeNeato(posX, posY, headingX, headingY)
        svc = rossvcclient('gazebo/set_model_state');
        msg = rosmessage(svc);

        msg.ModelState.ModelName = 'neato_standalone';
        startYaw = atan2(headingY, headingX);
        quat = eul2quat([startYaw 0 0]);

        msg.ModelState.Pose.Position.X = posX;
        msg.ModelState.Pose.Position.Y = posY;
        msg.ModelState.Pose.Position.Z = 1.0;
        msg.ModelState.Pose.Orientation.W = quat(1);
        msg.ModelState.Pose.Orientation.X = quat(2);
        msg.ModelState.Pose.Orientation.Y = quat(3);
        msg.ModelState.Pose.Orientation.Z = quat(4);

        % put the robot in the appropriate place
        ret = call(svc, msg);
    end

    % Gets the NEATO's current position
    function [posX, posY] = getNeatoPosition(msg)
        for i = 1 : length(msg.Name)
            if strcmp(msg.Name{i}, 'neato_standalone')
                posX = msg.Pose(i).Position.X;
                posY = msg.Pose(i).Position.Y;
                return
            end
        end
    end

    % Makes the NEATO do a little dance
    function do_a_dance()
        start = rostime('now');
        t = 0;
        
        % For five seconds
        while 1
            % Make the NEATO shake back and forth, doin' a lil' happy dance
            % :)
            
            message.Data = [.4, -.4];
            send(pubvel, message);
            pause(.3);
            
            message.Data = [-.4, .4];
            send(pubvel, message);
            pause(.3);
            
            t = rostime('now').seconds - start.seconds;
            
            % After 5 seconds, stop and exit
            if t > 5
                message.Data = [0, 0];
                send(pubvel, message);
                return
            end
        end
    end

end