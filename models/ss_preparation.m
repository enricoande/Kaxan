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
    block.NumOutputPorts = 4;

    % Setup functional port properties to dynamically inherited:
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
    
    % Size the input ports correctly and specify whether there is direct feedthrough:
%     block.InputPort(1).Dimensions        = 12;
%     block.InputPort(1).DirectFeedthrough = true;
%     block.InputPort(2).Dimensions        = 12;
%     block.InputPort(2).DirectFeedthrough = true;
%     block.InputPort(3).Dimensions        = 12;
%     block.InputPort(3).DirectFeedthrough = true;
%     block.InputPort(4).Dimensions        = 12;
%     block.InputPort(4).DirectFeedthrough = true;
%     block.InputPort(5).Dimensions        = 12;
%     block.InputPort(5).DirectFeedthrough = true;
%     block.InputPort(6).Dimensions        = 12;
%     block.InputPort(6).DirectFeedthrough = true;
%     block.InputPort(7).Dimensions        = 12;
%     block.InputPort(7).DirectFeedthrough = true;
%     block.InputPort(8).Dimensions        = 12;
%     block.InputPort(8).DirectFeedthrough = true;
    for i=1:8
        block.InputPort(i).Dimensions        = 12;
        block.InputPort(i).DirectFeedthrough = true;
        block.InputPort(i).SamplingMode = 0;
    end
    
    % Size the output ports correctly:
    block.OutputPort(1).Dimensions   = [8,8];
    block.OutputPort(2).Dimensions   = [8,4];
    block.OutputPort(3).Dimensions   = [8,8];
    block.OutputPort(4).Dimensions   = [8,4];
    % Set the output ports' sampling mode:
%     block.OutputPort(1).SamplingMode = 'Sample';
%     block.OutputPort(2).SamplingMode = 'Sample';
%     block.OutputPort(3).SamplingMode = 'Sample';
%     block.OutputPort(4).SamplingMode = 'Sample';
    for i=1:4
        block.OutputPort(i).SamplingMode = 'Sample';
    end
    
    % Define the number of parameters:
    block.NumDialogPrms = 4;

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
        Ad = [Ad;block.InputPort(i).Data(1:8)'];
        Bd = [Bd;block.InputPort(i).Data(9:12)'];
    end
    
    % Output the initial matrices Ad and Bd during the initialization:
    if block.CurrentTime<10
        for i=1:2
            block.OutputPort(i).Data = block.DialogPrm(i).Data;
        end
    % Otherwise output the matrices Ad and Bd estimated on-line:
    else
        block.OutputPort(1).Data = Ad;
        block.OutputPort(2).Data = Bd;
    end

    % Output the matrices Cd and Dd:
    for i=3:4
        block.OutputPort(i).Data = block.DialogPrm(i).Data;
    end
end