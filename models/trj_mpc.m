function trj_mpc(block)
% trjgen_mpc.m     e.anderlini@ucl.ac.uk     12/02/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is a Level-2 Matlab S-function for the specification of the desired
% trajectory to the MPC controller.
% NB: At the moment, the time horizon comprises of 25 time steps.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    setup(block);
end

%% Set up the block:
function setup(block)
    % Register number of input and output ports:
    block.NumInputPorts  = 0;
    block.NumOutputPorts = 1;

    % Setup functional port properties to dynamically inherited:
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
    
    % Size the output ports correctly:
    block.OutputPort(1).Dimensions = [25,8];  % can be changed 
    
    % Set the output ports' sampling mode:
    block.OutputPort(1).SamplingMode = 'Sample';
    
    % Define the number of parameters:
    block.NumDialogPrms = 3;

    % Set block sample time to inherited:
    block.SampleTimes = [0,0];

    % Set the block simStateCompliance to default:
    block.SimStateCompliance = 'DefaultSimState';

    % Register methods:
    block.RegBlockMethod('InitializeConditions', @InitConditions); 
    block.RegBlockMethod('Outputs',              @Output);    
end

%% Store and modify the trajectory object:
function InitConditions(block)
    % Get the trajectory data points:
    simin = block.DialogPrm(1).Data;
    trj = [simin(:,1:4),simin(:,7:10),simin(:,13)];
    % Store the object in the block's user's data:
    set_param(block.BlockHandle, 'UserData', trj);
end

%% Output the desired position, velocity and acceleration:
function Output(block)
    % Access the trajectory object stored in the user's data:
    trj = get_param(block.BlockHandle, 'UserData');
    % Get the prediction horizon data:
    dt  = block.DialogPrm(2).Data;
    p   = block.DialogPrm(3).Data;
    
    % Get the desired trajectory for the next time horizon:
    t = block.CurrentTime+(0:dt:dt*(p-1));
    out = interp1(trj(:,1),trj(:,2:end),t);

    % Output the desired trajectory:
    block.OutputPort(1).Data = out;
end