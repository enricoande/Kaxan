function t_transmat_4dof(block)
% t_transmat_4dof.m     e.anderlini@ucl.ac.uk     19/01/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is a Level-2 Matlab S-function for the computation of the transpose
% of the generalised transformation matrix between inertial and body-fixed
% reference frames in 4 DOF.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    setup(block);
end

%% Set up the block:
function setup(block)
    % Register number of input and output ports:
    block.NumInputPorts  = 2;
    block.NumOutputPorts = 1;

    % Setup functional port properties to dynamically inherited:
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
    
    % Size the input ports correctly:
    block.InputPort(1).Dimensions        = 12;
    block.InputPort(2).Dimensions        = 4;
    % Specify whether there is direct feedthrough:
    block.InputPort(1).DirectFeedthrough = true;
    block.InputPort(2).DirectFeedthrough = true;
    
    % Size the output port correctly:
    block.OutputPort(1).Dimensions   = 4;
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
function Jt = transpose_transformation_matrix(phi,theta,psi)
    % Translational transformation matrix:
    R = [cos(psi)*cos(theta),cos(theta)*sin(psi),-sin(theta);...
        cos(psi)*sin(phi)*sin(theta)-cos(phi)*sin(psi),...
        cos(phi)*cos(psi)+sin(phi)*sin(psi)*sin(theta),...
        cos(theta)*sin(phi);...
        sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta),...
        cos(phi)*sin(psi)*sin(theta)-cos(psi)*sin(phi),cos(theta)*cos(phi)];

    % Rotational transformation matrix:
    T = cos(phi)/cos(theta);

    % Transpose of the 6 degrees of freedom transformation matrix:
    Jt = [R,zeros(3,1);zeros(1,3),T];
end

%% Output the desired position, velocity and acceleration:
function Output(block)
    % Extract the current states & the thrust vector in the inertial frame:
    states = block.InputPort(1).Data;
    inertial_thrust = block.InputPort(2).Data;

    % Compute the transpose of the transformation matrix:
    Jt = transpose_transformation_matrix(states(4),states(5),states(6));
    
    % Output the thrust vector in 4 DOF in the body-fixed frame:
    block.OutputPort(1).Data = Jt*inertial_thrust;
end