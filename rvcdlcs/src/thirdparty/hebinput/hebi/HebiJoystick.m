classdef (Sealed) HebiJoystick < handle
    % HebiJoystick creates a joystick object
    %
    %   HebiJoystick is an alternative to MATLAB's built-in 'vrjoystick'
    %   for users who don't have access to the Simulink 3D Modelling
    %   Toolbox. The API is identical to 'vrjoystick' and can serve mostly
    %   as a drop-in replacement.
    %
    %   Note that the ordering of axes and buttons may be different on
    %   various operating systems, and that it may also be different from
    %   vrjoystick.
    %
    %   HebiJoystick Methods:
    %
    %       loadLibs - loads the required Java library
    %
    %       read     - reads the status of axes, buttons, and POVs
    %       axis     - reads the status of selected axes
    %       button   - reads the status of selected buttons
    %       pov      - reads the status of selected POV (point of view)
    %       caps     - returns a structure of joystick capabilities
    %       close    - closes and invalidates the joystick object
    %       force    - applies force feedback to selected axes
    %
    %   Example:
    %       % Connect to the first joystick and read its state
    %       joy = HebiJoystick(1);
    %       [axes, buttons, povs] = read(joy);
    %
    %   See also vrjoystick, loadLibs, read
    
    % Copyright (c) 2016-2017 HEBI Robotics
    
    properties (SetAccess = private)
        Name
        Axes
        Buttons
        POVs
        Forces
    end
    
    properties (Access = private)
        joy
    end
    
    methods (Static, Access = public)
        function loadLibs()
            % Loads the backing Java files and native binaries. This
            % method assumes that the jar file is located in the same
            % directory as this class-script, and that the file name
            % matches the string below.
            jarFileName = 'matlab-input-1.2.jar';
            
            % Load only once
            if ~exist('us.hebi.matlab.input.HebiJoystick', 'class')
                
                localDir = fileparts(mfilename('fullpath'));
                
                % Add binary libs
                java.lang.System.setProperty(...
                    'net.java.games.input.librarypath', ...
                    fullfile(localDir, 'lib'));
                
                % Add Java library
                javaaddpath(fullfile(localDir, jarFileName));
                
            end
        end
    end
    
    methods (Access = public)
        
        function this = HebiJoystick(index, ~)
            
            % creates a joystick object
            if nargin < 2
                % be compatible with the original API
            end
            
            % Create backing Java object
            HebiJoystick.loadLibs();
            this.joy = us.hebi.matlab.input.HebiJoystick(index);
            if ~ismac()
                % Increase event queue to not have to poll as often.
                % Doesn't work on mac.
                this.joy.setEventQueueSize(200);
            end
            
            % Set properties
            caps = this.caps();
            this.Name = this.joy.getName();
            this.Axes = caps.Axes;
            this.Buttons = caps.Buttons;
            this.POVs = caps.POVs;
            this.Forces = caps.Forces;
            
        end
        
        function varargout = read(this)
            % reads the status of axes, buttons, and POVs
            %
            % Example
            %   joy = HebiJoystick(1);
            %   [axes, buttons, povs] = read(joy);
            varargout = read(this.joy);
        end
        
        function axes = axis(this, mask)
            % reads the status of selected axes
            [axes, ~, ~] = read(this);
            if nargin > 1
                axes = axes(mask);
            end
        end
        
        function buttons = button(this, mask)
            % reads the status of selected buttons
            [~, buttons, ~] = read(this);
            if nargin > 1
                buttons = buttons(mask);
            end
        end
        
        function povs = pov(this, mask)
            % reads the status of selected POV (point of view)
            [~, ~, povs] = read(this);
            if nargin > 1
                povs = povs(mask);
            end
        end
        
        function out = caps(this)
            % returns a structure of joystick capabilities
            out = struct(caps(this.joy));
        end
        
        function [] = close(this)
            % closes and invalidates the joystick object
            close(this.joy);
        end
        
        function [] = force(this, indices, value)
            % applies force feedback to selected axes
            force(this.joy, indices, value);
        end
        
    end
    
    % Hide inherited methods (handle) from auto-complete
    % and docs
    methods(Access = public, Hidden = true)
        
        function [] = delete(this)
            % destructor disposes this instance
            close(this);
        end
        
        function varargout = addlistener(varargin)
            varargout{:} = addlistener@handle(varargin{:});
        end
        function varargout = eq(varargin)
            varargout{:} = eq@handle(varargin{:});
        end
        function varargout = findobj(varargin)
            varargout{:} = findobj@handle(varargin{:});
        end
        function varargout = findprop(varargin)
            varargout{:} = findprop@handle(varargin{:});
        end
        function varargout = ge(varargin)
            varargout{:} = ge@handle(varargin{:});
        end
        function varargout = gt(varargin)
            varargout{:} = gt@handle(varargin{:});
        end
        function varargout = le(varargin)
            varargout{:} = le@handle(varargin{:});
        end
        function varargout = lt(varargin)
            varargout{:} = lt@handle(varargin{:});
        end
        function varargout = ne(varargin)
            varargout{:} = ne@handle(varargin{:});
        end
        function varargout = notify(varargin)
            varargout{:} = notify@handle(varargin{:});
        end
        
    end
    
end