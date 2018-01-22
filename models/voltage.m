function voltage(block)
% voltage.m     e.anderlini@ucl.ac.uk     22/01/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is a Level-2 Matlab S-function for the computation of the voltage
% in each thruster.
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
    block.NumDialogPrms = 1;

    % Set block sample time to inherited:
    block.SampleTimes = [0,0];

    % Set the block simStateCompliance to default:
    block.SimStateCompliance = 'DefaultSimState';

    % Register methods:
    block.RegBlockMethod('Outputs',              @Output);    
end

%% Output the desired position, velocity and acceleration:
function Output(block)
    % Get the coefficients:
    coeffs = block.DialogPrm(1).Data;

    % Get the desired thrust in each thruster:
    thrust = block.InputPort(1).Data;
    % Initialize the corresponding voltage:
    volt = zeros(4,1);
    
    % Get the voltage in each thruster:
    for i=1:2
        % 520 thrusters:
        if thrust(i)<0
            volt(i) = coeffs(1,1)*(-thrust(i))^coeffs(1,2)+coeffs(1,3);
        else
            volt(i) = coeffs(2,1)*thrust(i)^coeffs(2,2)+coeffs(2,3);
        end
        
        % 540 thrusters:
        j = i +2;
        if thrust(j)<0
            volt(j) = coeffs(1,1)*(-thrust(j))^coeffs(1,2)+coeffs(1,3);
        else
            volt(j) = coeffs(2,1)*thrust(j)^coeffs(2,2)+coeffs(2,3);
        end
    end

    % Output the voltage in each thruster:
    block.OutputPort(1).Data = volt;
end