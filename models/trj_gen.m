function trj_gen(block)
% trj_gen.m     e.anderlini@ucl.ac.uk     27/11/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is a Level-2 Matlab S-function for the generation of the desired
% trajectory.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    setup(block);
end

%% Set up the block:
function setup(block)
    % Register number of input and output ports:
    block.NumInputPorts  = 0;
    block.NumOutputPorts = 3;

    % Setup functional port properties to dynamically inherited:
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
    
    % Size the output ports correctly:
    block.OutputPort(1).Dimensions = 6;
    block.OutputPort(2).Dimensions = 6;
    block.OutputPort(3).Dimensions = 6;
    
    % Set the output ports' sampling mode:
    block.OutputPort(1).SamplingMode = 'Sample';
    block.OutputPort(2).SamplingMode = 'Sample';
    block.OutputPort(3).SamplingMode = 'Sample';
    
    % Define the number of parameters:
    block.NumDialogPrms = 4;

    % Set block sample time to inherited:
    block.SampleTimes = [0,0];

    % Set the block simStateCompliance to default:
    block.SimStateCompliance = 'DefaultSimState';

    % Register methods:
    block.RegBlockMethod('InitializeConditions', @InitConditions); 
    block.RegBlockMethod('Outputs',              @Output);    
end

%% Initialize the trajectory object:
function InitConditions(block)
    % Extract the desired parameters:
    trj_type  = block.DialogPrm(1).Data;
    waypoints = block.DialogPrm(2).Data;
    duration  = block.DialogPrm(3).Data;
    dt        = block.DialogPrm(4).Data;
    
    % Initialize the trajectory object:
    trj = Trajectory(trj_type,waypoints,duration,dt);
    
    % Store the object in the block's user's data:
    set_param(block.BlockHandle, 'UserData', trj);
end

%% Output the desired position, velocity and acceleration:
function Output(block)
    % Access the trajectory object stored in the user's data:
    trj = get_param(block.BlockHandle, 'UserData');
    
    % Get the desired trajectory for the current time:
    trj = trj.trajectory_generation(block.CurrentTime);

    % Output the desired trajectory:
    block.OutputPort(1).Data = trj.des_disp;
    block.OutputPort(2).Data = trj.des_vel;
    block.OutputPort(3).Data = trj.des_acc;
end