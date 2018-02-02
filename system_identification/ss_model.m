function ss_preparation(block)
% ss_preparation.m     e.anderlini@ucl.ac.uk     01/02/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is a Level-2 Matlab S-function for the computation of the velocity
% vector in the inertial reference frame.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    setup(block);
end

%% Set up the block:
function setup(block)
    % Register number of input and output ports:
    block.NumInputPorts  = 8;
    block.NumOutputPorts = 2;

    % Setup functional port properties to dynamically inherited:
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
    
    % Size the input ports correctly and specify whether there is direct feedthrough:
    for i=1:8
        block.InputPort(i).Dimensions        = 12;
        block.InputPort(i).DirectFeedthrough = true;
        block.InputPort(i).SamplingMode = 0;
    end
    
    % Size the output ports correctly:
    block.OutputPort(1).Dimensions   = [8,8];
    block.OutputPort(2).Dimensions   = [8,4];
    % Set the output ports' sampling mode:
    for i=1:2
        block.OutputPort(i).SamplingMode = 'Sample';
    end
    
    % Define the number of parameters:
    block.NumDialogPrms = 0;

    % Set block sample time to continuous:
    block.SampleTimes = [0.1,0];

    % Set the block simStateCompliance to default:
    block.SimStateCompliance = 'DefaultSimState';

    % Register methods:
    block.RegBlockMethod('Outputs',              @Output);    
end

%% Output the desired position, velocity and acceleration:
function Output(block)
    % Build the state-space system matrices Ad and Bd:
    Ad = [];
    Bd = [];
    for i=1:8
        Ad = [Ad;block.InputPort(i).Data(5:12)'];
        Bd = [Bd;block.InputPort(i).Data(1:4)'];
    end
    
    % Otherwise output the matrices Ad and Bd estimated on-line:
    block.OutputPort(1).Data = Ad;
    block.OutputPort(2).Data = Bd;
end