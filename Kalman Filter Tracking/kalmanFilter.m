function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
    %UNTITLED Summary of this function goes here
    %Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    dt = t- previous_t;
    A=[ 1,0,dt,0;
        0,1,0,dt;
        0,0,1,0;
        0,0,0,1];
    C=[ 1,0,0,0;
        0,1,0,0];
    sig_m=diag([1e1,1e1,1e1,1e1]);
    sig_o=diag([0.001,0.001]);
    
    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    P=A*param.P*A' + sig_m;
    R=sig_o;
    %K=P*C'* inv(R+C*P*C');
    K=P*C'/(R+C*P*C');
    
    % v_x = (x - state(1)) / (t - previous_t);
    % v_y = (y - state(2)) / (t - previous_t);
    % Predict 330ms into the future
    % predictx = x + v_x * 0.330;
    % predicty = y + v_y * 0.330;
    state=A*state'+K*([x,y]'-C*A*state');
    state=state';
    param.P=P-K*C*P;
    
    v_x=state(3);
    v_y=state(4);
    predictx=state(1)+v_x*10*dt;
    predicty=state(2)+v_y*10*dt;
end