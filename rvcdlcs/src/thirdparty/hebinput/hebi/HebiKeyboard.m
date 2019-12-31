classdef (Sealed) HebiKeyboard < handle
    % HebiKeyboard creates a keyboard object
    %
    %   HebiKeyboard provides a way to get keyboard input
    %   in a non-blocking manner. The default driver requires
    %   focus on a window that is owned by MATLAB, i.e., the editor,
    %   the console, or any figure.
    %
    %   The read method returns a struct that contains a vector of 
    %   ascii keys (letters and numbers), as well as meta keys like CTRL,
    %   SHIFT, and ALT.
    %
    %   The key vector is laid out such that MATLAB can index it with
    %   ascii characters, i.e., state.keys('wasd') returns a vector of the
    %   states for the selected chracters. Upper and lower case characters
    %   are treated the same.
    %
    %   Example
    %       % Check if button 'x' is pressed
    %       kb = HebiKeyboard();
    %       while true
    %           state = read(kb);
    %           if state.keys('x')
    %               disp('X is pressed!')
    %           end
    %           pause(0.01);
    %       end
    %
    %   Example
    %       % Show all pressed letters whenever SHIFT is up
    %       kb = HebiKeyboard();
    %       while true
    %           state = read(kb);
    %           down = find(state.keys('a':'z')) + 'a';
    %           if ~state.SHIFT
    %               disp(char(down));
    %           end
    %           pause(0.01);
    %       end
    %
    %   Example
    %       % Select first keyboard with native driver
    %       kb = HebiKeyboard('native', 1);
    %       state = read(kb);

    % Copyright (c) 2016-2017 HEBI Robotics
    
    properties (SetAccess = private)
        Name
    end
    
    properties (Access = private)
        obj
    end
    
    methods (Static, Access = public)
        function loadLibs()
            HebiJoystick.loadLibs();
        end
    end
    
    methods (Access = public)
        
        function this = HebiKeyboard(driver, index)
            
            if nargin < 2
                index = 1;
            end
            
            if nargin < 1
                driver = 'AWT';
            end
            
            % Create backing Java object
            HebiKeyboard.loadLibs();
            this.obj = us.hebi.matlab.input.HebiKeyboard(driver, index);
            if ~ismac()
                % Increase event queue to not have to poll as often.
                % Doesn't work on mac.
                this.obj.setEventQueueSize(200);
            end
            
            % Set properties
            this.Name = this.obj.getName();
            
        end
        
        function out = read(this)
            % reads the current key state of the keyboard
            out = struct(read(this.obj));
        end
        
        function [] = close(this)
            % closes and invalidates the keyboard object
            close(this.obj);
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
