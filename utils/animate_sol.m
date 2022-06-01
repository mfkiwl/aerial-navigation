function animate_sol(time_vector, state, params)
% animate(...) returns an animated GIF of the numerical solution to the
% quadrotor dynamics.
%
% This is accomplished in two major steps:
% 1. Evenly sample the state vector at regular intervals
% 2. Take snapshot images of a plot at each sample point

assert(size(time_vector, 1) == size(state, 1))

%% Sample the 'state' at regular intervals
min_sampling_time = 0.025; % sample period in [seconds]
time_step = min_sampling_time; % the current distance between samples in [sec]
index = 1; % index number
sampled_state_index = []; % the subset of the 'state' variable

while index < size(time_vector, 1) - 1
    % keep going till we've gotten through the whole 'time_vector'
    time_step = time_step - (time_vector(index + 1) - time_vector(index));
    
    if time_step < 0
        % if the time between samples is now negative, that means we have
        % met the minimum sampling period
        sampled_state_index = [sampled_state_index; index];
        time_step = min_sampling_time;
    end
    
    index = index + 1;
end

%% Take snapshot images of a plot at each sample point
% create the same number of frames as there are samples
sampled_state = state(sampled_state_index,:);
sampled_time = time_vector(sampled_state_index);
animate_sol_helper(sampled_time, sampled_state(:,1), sampled_state(:,2), sampled_state(:,3), sampled_state(:,7), sampled_state(:,8), sampled_state(:,9), params);

end