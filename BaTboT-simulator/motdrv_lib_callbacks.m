function motdrv_lib_callbacks(block, callback)
%MOTDRV_LIB_CALLBACKS  Callback functons for blocks in MOTDRV_LIB.
%   MOTDRV_LIB_CALLBACKS(BLOCK, CALLBACKS) implements callback functions
%   used by blocks in the Simulink library MOTDRV_LIB.  For further help,
%   please use TYPE MOTDRV_LIB_CALLBACKS.

%   Copyright 2010 The MathWorks, Inc.

% Check that the block name is good:
try
    get_param(block, 'Name');
catch %#ok<CTCH>
    ME = MException([mfilename ':blockNotFound'], ...
            sprintf('Could not find block %s.', block));
    throwAsCaller(ME);
end

% Determine the appropriate callback:
switch(upper(callback))
    
    case 'INITIAL_DISPLACEMENT_SOURCE_DIALOG_CALLBACK'
        iInitialDisplacementSourceDialogCallback(block);
        
    case 'INITIAL_VELOCITY_SOURCE_DIALOG_CALLBACK'
        iInitialVelocitySourceDialogCallback(block);
        
    otherwise
        ME = MException([mfilename ':unrecognizedCallback'], ...
                sprintf('Unrecognised callback %s for block %s.', ...
                    callback, block));
        throwAsCaller(ME);
        
end % SWITCH/CASE

end % motdrv_lib_callbacks()



% ****************************************************************
% * Callbacks used by the Motion Driver With Displacement Input **
% ****************************************************************


% Function: iInitialDisplacementSourceDialogCallback() ====================
% Abstract:
%    This callback is used to update the dialog according to the value
%    selected for the initial displacement source: if we're inheriting it
%    from the input, we don't want the unused dialog parameter to be
%    displayed.
%

function iInitialDisplacementSourceDialogCallback(block)

SRC_PARAMETER_NAME = 'InitialDisplacementSource';
PARAMETER_NAME     = 'InitialDisplacement';

parametervalue = get_param(gcb, SRC_PARAMETER_NAME);

% Determine which mask parameter we are looking for:
masknames = get_param(block, 'MaskNames');
idx = find(strcmpi(PARAMETER_NAME, masknames));

% Get the existing mask visibilities:
maskvisibilities = get_param(block, 'MaskVisibilities');

% Work out the new visibilities:
newvisibilities = maskvisibilities;
switch upper(parametervalue)

    case 'DIALOG'
        newvisibilities{idx} = 'on';
    
    case 'INHERITED'
        newvisibilities{idx} = 'off';
    
    otherwise
        error('Unrecognized value of parameter %s in block %s: %s.', ...
            PARAMETER_NAME, block, parametervalue);

end % SWITCH/CASE

% If anything's changed, change the mask visibilities parameter value.
if any(~strcmpi(maskvisibilities, newvisibilities))
    set_param(block, 'MaskVisibilities', newvisibilities);
end

end % iInitialDisplacementSourceDialogCallback()



% ************************************************************
% * Callbacks used by the Motion Driver With Velocity Input **
% ************************************************************


% Function: iInitialVelocitySourceDialogCallback() ====================
% Abstract:
%    This callback is used to update the dialog according to the value
%    selected for the initial velocity source: if we're inheriting it
%    from the input, we don't want the unused dialog parameter to be
%    displayed.
%

function iInitialVelocitySourceDialogCallback(block)

SRC_PARAMETER_NAME = 'InitialVelocitySource';
PARAMETER_NAME     = 'InitialVelocity';

parametervalue = get_param(gcb, SRC_PARAMETER_NAME);

% Determine which mask parameter we are looking for:
masknames = get_param(block, 'MaskNames');
idx = find(strcmpi(PARAMETER_NAME, masknames));

% Get the existing mask visibilities:
maskvisibilities = get_param(block, 'MaskVisibilities');

% Work out the new visibilities:
newvisibilities = maskvisibilities;
switch upper(parametervalue)

    case 'DIALOG'
        newvisibilities{idx} = 'on';
    
    case 'INHERITED'
        newvisibilities{idx} = 'off';
    
    otherwise
        error('Unrecognized value of parameter %s in block %s: %s.', ...
            PARAMETER_NAME, block, parametervalue);

end % SWITCH/CASE

% If anything's changed, change the mask visibilities parameter value.
if any(~strcmpi(maskvisibilities, newvisibilities))
    set_param(block, 'MaskVisibilities', newvisibilities);
end

end % iInitialVelocitySourceDialogCallback()


% ====== END OF FILE MOTDRV_LIB_CALLBACKS.M ===============================
