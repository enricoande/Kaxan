function accumulator(block)
% accumulator.m     e.anderlini@ucl.ac.uk     22/01/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is a Level-2 Matlab S-function for the accumulation of a signal.
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
    block.InputPort(1).Dimensions        = 4;
    % Specify whether there is direct feedthrough:
    block.InputPort(1).DirectFeedthrough = true;
    
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
    block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Outputs',              @Output);    
end

%% Set-up the dynamic work vector:
function DoPostPropSetup(block)
    block.NumDworks = 1;
    block.Dwork(1).Name = 'signal'; 
    block.Dwork(1).Dimensions      = 4;
    block.Dwork(1).DatatypeID      = 0;
    block.Dwork(1).Complexity      = 'Real';
    block.Dwork(1).UsedAsDiscState = true;
end

%% Initialize the dynamic work vector:
function InitConditions(block)
    % Initialize Dwork
    block.Dwork(1).Data = zeros(4,1);
end

%% Output the accumulated signal:
function Output(block)
    % Extract the current signal:
    signal = block.InputPort(1).Data;
    
    % Accumulate the signal (basically, integrate from t=0 s):
    block.Dwork(1).Data = block.Dwork(1).Data + signal;
    % Avoid wind-up:
    for i=1:4
        block.Dwork(1).Data(i) = min(block.Dwork(1).Data(i),1);
        block.Dwork(1).Data(i) = max(block.Dwork(1).Data(i),-1);
    end

    % Return the accumulated signal:
    block.OutputPort(1).Data = block.Dwork(1).Data;
end