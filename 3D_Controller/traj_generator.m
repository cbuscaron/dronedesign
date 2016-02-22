function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%

persistent waypoints0 traj_time d0
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
else
        
    if(t >= traj_time(end))
        t = traj_time(end) - 0.0001;
    end

    t_index = find(traj_time > t,1) - 1;

    if (t_index == 0)
        t_index = 1;
    end
   
    desired_state.pos = zeros(3,1);
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;

    coff = getCoff(waypoints0);
    scale = (t-traj_time(t_index))/(d0(t_index));
    t0 = polyT(8,0,scale)';
    t1 = polyT(8,1,scale)';
    t2 = polyT(8,2,scale)';
    index = (t_index-1)*8+1:t_index*8;
    desired_state.pos = coff(index,:)'*t0;
    desired_state.vel = coff(index,:)'*t1.*(1/d0(t_index));
    desired_state.acc = coff(index,:)'*t2.*(1/d0(t_index)^2);
end

end


%% UTILITY FUNCTION
function[T] = polyT(n,k,t)
    T = zeros(n,1);
    D = zeros(n,1);
    %Initial
    for i=1:n
        D(i)=i-1;
        T(i)=1;
    end
    %Derivative
    for j=1:k
        for i=1:n
            T(i)=T(i)*D(i);
            if D(i)>0
                D(i)=D(i)-1;
            end
        end
    end
    %Put t value
    for i=1:n
        T(i)=T(i)*t^D(i);
    end
    T=T';
end

%% COEFFICIENT 
    function[coff] = getCoff(waypoints)
        n = size(waypoints,2)-1;
        A = zeros(8*n,8*n);
        b = zeros(3,8*n);
       % coff = zeros(3,8*n);
        for i=1:n
        b(:,i)=waypoints(:,i);
        b(:,i+n)=waypoints(:,i+1);
        end        %Constraint1 Pi(0) = Wi for all i=1..n
        row = 1;
        for i=1:n
            A(row,((i-1)*8)+1:i*8)=polyT(8,0,0);
            row=row+1;
        end
        % Constraint2 Pi(1)=Wi+1 for all i=1..n
        for i=1:n
            A(row,((i-1)*8)+1:i*8)=polyT(8,0,1);
            row=row+1;
        end
        % Constraint3 P1(k)(0)=0 for all 1<=k<=3
        for k=1:3
            A(row,(1:8))=polyT(8,k,0);
            row=row+1;
        end
        % Constraint4 Pn(k)(1)=0 for all 1<=k<=3
            for k=1:3
            A(row,25:32)=polyT(8,k,1);
            row=row+1;
            end
         % Constraint5 Pi-1(k)(1)=Pi(k)(0) for all 1<=k<=3
       for i=2:n
           for k=1:6
              A(row,((i-2)*8)+1:i*8)= [polyT(8,k,1) -polyT(8,k,0)];
               row=row+1;
           end
       end
       coff = A\b'; 
    end
