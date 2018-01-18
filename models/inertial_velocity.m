function inertial_velocity(block)
% inertial_velocity.m     e.anderlini@ucl.ac.uk     18/01/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is a Level-2 Matlab S-function for the computation of the velocity
% vector in the inertial reference frame.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    setup(block);
end

%% Set up the block:
function setup(block)
    % Register number of input and output ports:
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 1;

    % Setup functional port properties to dynamically inherited:
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
    
    % Size the input port correctly:
    block.InputPort(1).Dimensions        = 12;
    % Specify whether there is direct feedthrough:
    block.InputPort(1).DirectFeedthrough = true;
    
    % Size the output port correctly:
    block.OutputPort(1).Dimensions   = 6;
    % Set the output ports' sampling mode:
    block.OutputPort(1).SamplingMode = 'Sample';
    
    % Define the number of parameters:
    block.NumDialogPrms = 0;

    % Set block sample time to inherited:
    block.SampleTimes = [0,0];

    % Set the block simStateCompliance to default:
    block.SimStateCompliance = 'DefaultSimState';

    % Register methods:
    block.RegBlockMethod('Outputs',              @Output);    
end

%% Compute the transformation matrix:
function J = transformation_matrix(phi,theta,psi)
    % Translational transformation matrix:
    R = [cos(psi)*cos(theta),cos(psi)*sin(phi)*sin(theta)-cos(phi)*sin(psi),...
    sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta);...
    cos(theta)*sin(psi),cos(phi)*cos(psi)+sin(phi)*sin(psi)*sin(theta),...
    cos(phi)*sin(psi)*sin(theta)-cos(psi)*sin(phi);
    -sin(theta),cos(theta)*sin(phi),cos(theta)*cos(phi)];

    % Rotational transformation matrix:
    T = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);...
    0, cos(phi), -sin(phi);...
    0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

    % 6 degrees of freedom transformation matrix:
    J = [R,zeros(3,3);zeros(3,3),T];
end

%% Output the desired position, velocity and acceleration:
function Output(block)
    % Extract the current states:
    states = block.InputPort(1).Data;

    % Compute the transformation matrix:
    J = transformation_matrix(states(4),states(5),states(6));

    % Output the velocity vector in the inertial frame:
    block.OutputPort(1).Data = J*states(7:12);
end