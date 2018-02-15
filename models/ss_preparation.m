function ss_preparation(block)
% ss_preparation.m     e.anderlini@ucl.ac.uk     14/02/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is a Level-2 Matlab S-function for the off-line identification of
% the state-space model of the ROV.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    setup(block);
end

%% Set up the block:
function setup(block)
    % Register number of input and output ports:
    block.NumInputPorts  = 3;
    block.NumOutputPorts = 4;

    % Setup functional port properties to dynamically inherited:
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
    
    % Size the input ports correctly and specify whether there is direct feedthrough:
    block.InputPort(1).Dimensions        = 8;
    block.InputPort(1).DirectFeedthrough = true;
    block.InputPort(1).SamplingMode = 0;
    block.InputPort(2).Dimensions        = 4;
    block.InputPort(2).DirectFeedthrough = true;
    block.InputPort(2).SamplingMode = 0;
    block.InputPort(3).Dimensions        = 1;
    block.InputPort(3).DirectFeedthrough = true;
    block.InputPort(3).SamplingMode = 0;
    
    % Size the output ports correctly:
    block.OutputPort(1).Dimensions   = [8,8];
    block.OutputPort(2).Dimensions   = [8,5];
    block.OutputPort(3).Dimensions   = [8,8];
    block.OutputPort(4).Dimensions   = [8,5];
    % Set the output ports' sampling mode:
    for i=1:4
        block.OutputPort(i).SamplingMode = 'Sample';
    end
    
    % Define the number of parameters:
    block.NumDialogPrms = 8;

    % Set block sample time to fixed time:
    block.SampleTimes = [0.01,0];

    % Set the block simStateCompliance to default:
    block.SimStateCompliance = 'DefaultSimState';

    % Register methods:
    block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Outputs',              @Output);    
end

%% Set-up the dynamic work vector:
function DoPostPropSetup(block)
    block.NumDworks = 13;
    name = {'x','y','z','psi','x_dot','y_dot','z_dot','phi_dot','u_1',...
        'u_2','u_3','u_4','d'};
    for i=1:13
        block.Dwork(i).Name = name{i}; 
        block.Dwork(i).Dimensions      = 7001;
        block.Dwork(i).DatatypeID      = 0;    % double
        block.Dwork(i).Complexity      = 'Real';
        block.Dwork(i).UsedAsDiscState = true;
    end
end

%% Initialize the user data:
function InitConditions(block)
    % Initialize the DWork vectors:
    for i=1:13
        block.Dwork(i).Data = zeros(7001,1);
    end

    % Initialize the state-space model:
    sysd = ss(block.DialogPrm(1).Data,block.DialogPrm(2).Data,...
        block.DialogPrm(3).Data,block.DialogPrm(4).Data,0.1);
    % Store the structure in the block's user's data: 
    set_param(block.BlockHandle, 'UserData', sysd);
end

%% Output the desired position, velocity and acceleration:
function Output(block)
    % Access the identified state-space model stored in the user's data:
    sysd = get_param(block.BlockHandle, 'UserData');
    
    % Store the simulation data to memory for system identification:
    c = round(block.CurrentTime/0.01)+1;  % counter
    for i=1:8
        block.Dwork(i).Data(c) = block.InputPort(1).Data(i);
    end
    for i=1:4
        block.Dwork(i+8).Data(c) = block.InputPort(2).Data(i);
    end
    block.Dwork(13).Data(c) = block.InputPort(3).Data;
        
    % Estimate the matrices Ad and Bd on-line in batch mode:
    if block.CurrentTime>5 && mod(block.CurrentTime,5)==0
        % Prepare a state-space model with identifiable parameters:
        init_sys = idss(block.DialogPrm(5).Data,block.DialogPrm(6).Data,...
            block.DialogPrm(7).Data,0,'Ts',0);
        init_sys.Structure.A.Free(1:4,:) = false;
        init_sys.Structure.A.Free(5:8,1:4) = false;
        init_sys.Structure.B.Free(1:4,:) = false;
        init_sys.Structure.C.Free = false;
        init_sys.Structure.D.Free = false;
        % Initialize data object:
        states = [];
        in = [];
        for i=1:8
            states = [states,block.Dwork(i).Data(1:c)];
        end
        for i=1:5
            in = [in,block.Dwork(i+8).Data(1:c)];
        end
        data = iddata(states,in,0.01);
        % Estimate the values of the state-space model:
        sys = ssest(data,init_sys);
        % Convert the state-space model to discrete time:
        sysd = c2d(sys,0.1);
        % Store the identified state-space model in memory:
        set_param(block.BlockHandle, 'UserData', sysd);
    end    
    
    % Output the identified matrices:
    block.OutputPort(1).Data = sysd.A;
    block.OutputPort(2).Data = sysd.B;
    block.OutputPort(3).Data = sysd.C;
    block.OutputPort(4).Data = sysd.D;
end