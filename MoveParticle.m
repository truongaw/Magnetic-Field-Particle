classdef MoveParticle < matlab.apps.AppBase
%%Andrew Truong
    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                        matlab.ui.Figure
        UIAxes                          matlab.ui.control.UIAxes
        UIAxes3                         matlab.ui.control.UIAxes
        UIAxes4                         matlab.ui.control.UIAxes
        UIAxes5                         matlab.ui.control.UIAxes
        UIAxes7                         matlab.ui.control.UIAxes
        UIAxes8                         matlab.ui.control.UIAxes
        UIAxes9                         matlab.ui.control.UIAxes
        UIAxes10                        matlab.ui.control.UIAxes
        UIAxes2                         matlab.ui.control.UIAxes
        GoalXSliderLabel                matlab.ui.control.Label
        GoalXSlider                     matlab.ui.control.Slider
        OnSliderOffKeyboardButton       matlab.ui.control.StateButton
        GoalYSliderLabel                matlab.ui.control.Label
        GoalYSlider                     matlab.ui.control.Slider
        GoalZSliderLabel                matlab.ui.control.Label
        GoalZSlider                     matlab.ui.control.Slider
        ArrowKeysMoveXYwsUpDownLabel    matlab.ui.control.Label
        ForceMultiplierSliderLabel      matlab.ui.control.Label
        ForceMultiplierSlider           matlab.ui.control.Slider
        ViscosityMultiplierSliderLabel  matlab.ui.control.Label
        ViscosityMultiplierSlider       matlab.ui.control.Slider
        OutputxyzvxvyvzaxayazButton     matlab.ui.control.StateButton
        FigureBackgroundmustbeclickedtoenablekeyboardLabel  matlab.ui.control.Label
        UIAxes6                         matlab.ui.control.UIAxes
    end

    properties (Access = private)
        clock=timer('ExecutionMode','fixedRate') ;
    end
    methods (Access = private)
        function timer_calback(app,~) 
            global clk dt x y z vx vy vz ax ay az xF yF zF b inv output
            clk=clk+dt;

            vx=vx+ax*dt;
            vy=vy+ay*dt;
            vz=vz+az*dt;
            x=x+vx*dt+1/2*ax*dt^2;
            y=y+vy*dt+1/2*ay*dt^2;
            z=z+vz*dt+1/2*az*dt^2;
            
            if inv==1%Toggle between Keyboard and Slider Control
                inverse(app)
                if (x>xF-0.1)&&(x<xF+0.1)%Zero Threshold Tolerance for Convergance
                    ax=0;
                end
                
                if (y>yF-0.1)&&(y<yF+0.1)
                    ay=0;
                end
                
                if (z>zF-0.1)&&(z<zF+0.1)
                    az=0;
                end
            else
                xF=0;
                yF=0;
                zF=0;
            end
            if output==1
                disp([x y z vx vy vz ax ay az])
            end
            if x>4.5||x<-4.5%Position Limits
                x=sign(x)*4.5;
                vx=0;
                ax=0;
            end
            
            if y>4.5||y<-4.5
                y=sign(y)*4.5;
                vy=0;
                ay=0;
            end
            
            if z>4.5||z<=-4.5
                z=sign(z)*4.5;
                vz=0;
                az=0;
            end
            if isempty(b)==1
                b=1;
            end
            if abs(vx)>0.075%Velocity Limits
                vx=sign(vx)*(abs(vx)-(b.*0.05*abs(vx^2)));%Viscous Drag Force
            else
                vx=0;%Zero Threshold Tolerance for Convergance
            end
            
            if abs(vy)>0.075
                vy=sign(vy)*(abs(vy)-(b.*0.05*abs(vy^2)));
            else
                vy=0;
            end
            
            if abs(vz)>0.075
                vz=sign(vz)*(abs(vz)-(b.*0.05*abs(vz^2)));
            else
                vz=0;
            end
            
            part(app,x,y,z);
            plotEqn(app);
        end
    
        function part(app,x,y,z)
            latlon=8;
            theta=linspace(0,pi,latlon);
            phi=linspace(0,2*pi,2*latlon);
            [th,ph]=meshgrid(theta,phi);
            rad=.5;
            xp=rad.*sin(th).*cos(ph);
            yp=rad.*sin(th).*sin(ph);
            zp=rad.*cos(th);
            surf(app.UIAxes,xp+x,yp+y,zp+z);
        end
        
        function plotEqn(app)
            global x y z vx vy vz ax ay az i X Y Z VX VY VZ AX AY AZ
            X(i)=x;
            Y(i)=y;
            Z(i)=z;
            VX(i)=vx;
            VY(i)=vy;
            VZ(i)=vz;
            AX(i)=ax;
            AY(i)=ay;
            AZ(i)=az;              
            i=i+1;
            plot(app.UIAxes2,X)
            plot(app.UIAxes3,Y)
            plot(app.UIAxes4,Z)
            plot(app.UIAxes5,VX)
            plot(app.UIAxes6,VY)
            plot(app.UIAxes7,VZ)           
            plot(app.UIAxes8,AX)
            plot(app.UIAxes9,AY)
            plot(app.UIAxes10,AZ)
            if i>=100
                X(2-1:end-1)=X(2:end);
                Y(2-1:end-1)=Y(2:end);
                Z(2-1:end-1)=Z(2:end);
                VX(2-1:end-1)=VX(2:end);
                VY(2-1:end-1)=VY(2:end);
                VZ(2-1:end-1)=VZ(2:end);
                AX(2-1:end-1)=AX(2:end);
                AY(2-1:end-1)=AY(2:end);
                AZ(2-1:end-1)=AZ(2:end);               
                i=100;
            end
        end
        
        function inverse(~)
            global xF yF zF x  y z vx vy vz ax ay az dt F
            if isempty(F)==1
                F=1;
            end
            ax=F*0.02.*(xF-x-vx.*dt)./dt.^2;
            ay=F*0.02.*(yF-y-vy.*dt)./dt.^2;
            az=F*0.02.*(zF-z-vz.*dt)./dt.^2;
        end
    end

    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            global clk dt x y z vx vy vz ax ay az i X Y Z VX VY VZ AX AY AZ
            X=zeros(100,1);
            Y=X;Z=X;VX=X;VY=X;VZ=X;AX=X;AY=X;AZ=X;
            dt=.05;
            i=3;
            clk=0+dt;
            x=0;y=0;z=0;vx=0;vy=0;vz=0;ax=0;ay=0;az=0;
            app.clock.TimerFcn=@(~,~) timer_calback(app);
            app.clock.Period = dt ;
            start(app.clock)
        end

        % Key press function: UIFigure
        function UIFigureKeyPress(app, event)
            global ax ay az
            key = event.Key;%Uniform Magnetic Field Force Vector
            if isequal(key,'rightarrow')
                ax=5;
            end
            if isequal(key,'leftarrow')
                ax=-5;
            end
            if isequal(key,'uparrow')
                ay=5;
            end
            if isequal(key,'downarrow')
                ay=-5;
            end           
            if isequal(key,'w')
                az=5;
            end  
            if isequal(key,'s')
                az=-5;
            end              
            if isequal(key,'0')%Pause Function
                if strcmp(app.clock.Running,'on')
                    stop(app.clock)
                    disp('App Paused. Press "0" to Resume')
                else
                    disp('Unpaused')
                    start(app.clock)
                end
            end
        end

        % Key release function: UIFigure
        function UIFigureKeyRelease(app, event)
            global ax ay az
            key = event.Key;
            if isempty(key)==false
                ax=0;ay=0;az=0;             
            end
        end

        % Callback function: GoalXSlider, GoalYSlider
        function GoalXSliderValueChanging(app, event)
            global xF
            xF = event.Value;
        end

        % Value changed function: OnSliderOffKeyboardButton, 
        % OutputxyzvxvyvzaxayazButton
        function OnSliderOffKeyboardButtonValueChanged(app, event)
            global inv output
            inv = app.OnSliderOffKeyboardButton.Value;
            output = app.OutputxyzvxvyvzaxayazButton.Value;
        end

        % Value changing function: GoalZSlider
        function GoalZSliderValueChanging(app, event)
            global zF
            zF = event.Value;  
        end

        % Value changing function: ViscosityMultiplierSlider
        function ViscosityMultiplierSliderValueChanging(app, event)
            global b
            b = event.Value;
        end

        % Value changing function: ForceMultiplierSlider
        function ForceMultiplierSliderValueChanging(app, event)
            global F
            F = event.Value;    
        end

        % Value changing function: GoalYSlider
        function GoalYSliderValueChanging(app, event)
            global yF
            yF = event.Value;       
        end
    end

    % App initialization and construction
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure
            app.UIFigure = uifigure;
            app.UIFigure.Position = [100 100 758 428];
            app.UIFigure.Name = 'UI Figure';
            app.UIFigure.KeyPressFcn = createCallbackFcn(app, @UIFigureKeyPress, true);
            app.UIFigure.KeyReleaseFcn = createCallbackFcn(app, @UIFigureKeyRelease, true);

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            title(app.UIAxes, {'Kinematics of a Particle in a Viscous Environment'; 'Subjected to a Magnetic Force Caused by Helmholtz Coils '})
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            app.UIAxes.PlotBoxAspectRatio = [1 1 1];
            app.UIAxes.XLim = [-5 5];
            app.UIAxes.YLim = [-5 5];
            app.UIAxes.ZLim = [-5 5];
            app.UIAxes.XGrid = 'on';
            app.UIAxes.YGrid = 'on';
            app.UIAxes.ZGrid = 'on';
            app.UIAxes.Position = [11 150 371 279];

            % Create UIAxes3
            app.UIAxes3 = uiaxes(app.UIFigure);
            title(app.UIAxes3, 'Position Y')
            xlabel(app.UIAxes3, '')
            ylabel(app.UIAxes3, '')
            app.UIAxes3.YLim = [-5 5];
            app.UIAxes3.XTickLabel = '';
            app.UIAxes3.Position = [641 245 117 89];

            % Create UIAxes4
            app.UIAxes4 = uiaxes(app.UIFigure);
            title(app.UIAxes4, 'Position Z')
            xlabel(app.UIAxes4, '')
            ylabel(app.UIAxes4, '')
            app.UIAxes4.YLim = [-5 5];
            app.UIAxes4.XTick = [];
            app.UIAxes4.Position = [642 157 117 89];

            % Create UIAxes5
            app.UIAxes5 = uiaxes(app.UIFigure);
            title(app.UIAxes5, 'Velocity X')
            xlabel(app.UIAxes5, '')
            ylabel(app.UIAxes5, '')
            app.UIAxes5.YLim = [-2.2 2.2];
            app.UIAxes5.XTickLabel = '';
            app.UIAxes5.Position = [525 333 117 89];

            % Create UIAxes7
            app.UIAxes7 = uiaxes(app.UIFigure);
            title(app.UIAxes7, 'Velocity Z')
            xlabel(app.UIAxes7, '')
            ylabel(app.UIAxes7, '')
            app.UIAxes7.YLim = [-2.2 2.2];
            app.UIAxes7.XTickLabel = '';
            app.UIAxes7.Position = [525 157 117 89];

            % Create UIAxes8
            app.UIAxes8 = uiaxes(app.UIFigure);
            title(app.UIAxes8, 'Accel X')
            xlabel(app.UIAxes8, '')
            ylabel(app.UIAxes8, '')
            app.UIAxes8.YLim = [-20 20];
            app.UIAxes8.XTickLabel = '';
            app.UIAxes8.YTick = [-20 0 20];
            app.UIAxes8.YTickLabel = '';
            app.UIAxes8.Position = [392 333 117 89];

            % Create UIAxes9
            app.UIAxes9 = uiaxes(app.UIFigure);
            title(app.UIAxes9, 'Accel Y')
            xlabel(app.UIAxes9, '')
            ylabel(app.UIAxes9, '')
            app.UIAxes9.YLim = [-20 20];
            app.UIAxes9.XTickLabel = '';
            app.UIAxes9.YTick = [-20 20];
            app.UIAxes9.YTickLabel = '';
            app.UIAxes9.Position = [392 245 117 89];

            % Create UIAxes10
            app.UIAxes10 = uiaxes(app.UIFigure);
            title(app.UIAxes10, 'Accel Z')
            xlabel(app.UIAxes10, '')
            ylabel(app.UIAxes10, '')
            app.UIAxes10.YLim = [-20 20];
            app.UIAxes10.XTickLabel = '';
            app.UIAxes10.YTick = [-20 20];
            app.UIAxes10.YTickLabel = '';
            app.UIAxes10.Position = [392 157 117 89];

            % Create UIAxes2
            app.UIAxes2 = uiaxes(app.UIFigure);
            title(app.UIAxes2, 'Position X')
            xlabel(app.UIAxes2, '')
            ylabel(app.UIAxes2, '')
            app.UIAxes2.YLim = [-5 5];
            app.UIAxes2.XTickLabel = '';
            app.UIAxes2.Position = [641 333 117 89];

            % Create GoalXSliderLabel
            app.GoalXSliderLabel = uilabel(app.UIFigure);
            app.GoalXSliderLabel.HorizontalAlignment = 'right';
            app.GoalXSliderLabel.Position = [11 124 42 22];
            app.GoalXSliderLabel.Text = 'Goal X';

            % Create GoalXSlider
            app.GoalXSlider = uislider(app.UIFigure);
            app.GoalXSlider.Limits = [-4.5 4.5];
            app.GoalXSlider.ValueChangingFcn = createCallbackFcn(app, @GoalXSliderValueChanging, true);
            app.GoalXSlider.Position = [74 133 150 3];

            % Create OnSliderOffKeyboardButton
            app.OnSliderOffKeyboardButton = uibutton(app.UIFigure, 'state');
            app.OnSliderOffKeyboardButton.ValueChangedFcn = createCallbackFcn(app, @OnSliderOffKeyboardButtonValueChanged, true);
            app.OnSliderOffKeyboardButton.Text = {'On: Slider'; 'Off: Keyboard'};
            app.OnSliderOffKeyboardButton.Position = [315 111 100 36];

            % Create GoalYSliderLabel
            app.GoalYSliderLabel = uilabel(app.UIFigure);
            app.GoalYSliderLabel.HorizontalAlignment = 'right';
            app.GoalYSliderLabel.Position = [8 82 42 22];
            app.GoalYSliderLabel.Text = 'Goal Y';

            % Create GoalYSlider
            app.GoalYSlider = uislider(app.UIFigure);
            app.GoalYSlider.Limits = [-4.5 4.5];
            app.GoalYSlider.ValueChangedFcn = createCallbackFcn(app, @GoalXSliderValueChanging, true);
            app.GoalYSlider.ValueChangingFcn = createCallbackFcn(app, @GoalYSliderValueChanging, true);
            app.GoalYSlider.Position = [71 91 150 3];

            % Create GoalZSliderLabel
            app.GoalZSliderLabel = uilabel(app.UIFigure);
            app.GoalZSliderLabel.HorizontalAlignment = 'right';
            app.GoalZSliderLabel.Position = [11 43 42 22];
            app.GoalZSliderLabel.Text = 'Goal Z';

            % Create GoalZSlider
            app.GoalZSlider = uislider(app.UIFigure);
            app.GoalZSlider.Limits = [-4.5 4.5];
            app.GoalZSlider.ValueChangingFcn = createCallbackFcn(app, @GoalZSliderValueChanging, true);
            app.GoalZSlider.Position = [74 52 150 3];

            % Create ArrowKeysMoveXYwsUpDownLabel
            app.ArrowKeysMoveXYwsUpDownLabel = uilabel(app.UIFigure);
            app.ArrowKeysMoveXYwsUpDownLabel.HorizontalAlignment = 'center';
            app.ArrowKeysMoveXYwsUpDownLabel.Position = [301 27 129 27];
            app.ArrowKeysMoveXYwsUpDownLabel.Text = {'Arrow Keys: Move X, Y'; '"w","s": Up/Down'};

            % Create ForceMultiplierSliderLabel
            app.ForceMultiplierSliderLabel = uilabel(app.UIFigure);
            app.ForceMultiplierSliderLabel.HorizontalAlignment = 'center';
            app.ForceMultiplierSliderLabel.Position = [480 120 54 27];
            app.ForceMultiplierSliderLabel.Text = {'Force'; 'Multiplier'};

            % Create ForceMultiplierSlider
            app.ForceMultiplierSlider = uislider(app.UIFigure);
            app.ForceMultiplierSlider.Limits = [0.1 5];
            app.ForceMultiplierSlider.ValueChangingFcn = createCallbackFcn(app, @ForceMultiplierSliderValueChanging, true);
            app.ForceMultiplierSlider.Position = [555 134 150 3];
            app.ForceMultiplierSlider.Value = 1;

            % Create ViscosityMultiplierSliderLabel
            app.ViscosityMultiplierSliderLabel = uilabel(app.UIFigure);
            app.ViscosityMultiplierSliderLabel.HorizontalAlignment = 'center';
            app.ViscosityMultiplierSliderLabel.Position = [480 71 54 27];
            app.ViscosityMultiplierSliderLabel.Text = {'Viscosity'; 'Multiplier'};

            % Create ViscosityMultiplierSlider
            app.ViscosityMultiplierSlider = uislider(app.UIFigure);
            app.ViscosityMultiplierSlider.Limits = [0.1 5];
            app.ViscosityMultiplierSlider.ValueChangingFcn = createCallbackFcn(app, @ViscosityMultiplierSliderValueChanging, true);
            app.ViscosityMultiplierSlider.Position = [555 85 150 3];
            app.ViscosityMultiplierSlider.Value = 1;

            % Create OutputxyzvxvyvzaxayazButton
            app.OutputxyzvxvyvzaxayazButton = uibutton(app.UIFigure, 'state');
            app.OutputxyzvxvyvzaxayazButton.ValueChangedFcn = createCallbackFcn(app, @OnSliderOffKeyboardButtonValueChanged, true);
            app.OutputxyzvxvyvzaxayazButton.Text = {'Output'; '[(x y z) (vx vy vz) (ax ay az)]'};
            app.OutputxyzvxvyvzaxayazButton.Position = [279 62 166 36];

            % Create FigureBackgroundmustbeclickedtoenablekeyboardLabel
            app.FigureBackgroundmustbeclickedtoenablekeyboardLabel = uilabel(app.UIFigure);
            app.FigureBackgroundmustbeclickedtoenablekeyboardLabel.FontSize = 10;
            app.FigureBackgroundmustbeclickedtoenablekeyboardLabel.Position = [555 24 135 22];
            app.FigureBackgroundmustbeclickedtoenablekeyboardLabel.Text = {'*Figure Background must be '; 'clicked to enable keyboard'};

            % Create UIAxes6
            app.UIAxes6 = uiaxes(app.UIFigure);
            title(app.UIAxes6, 'Velocity Y')
            xlabel(app.UIAxes6, '')
            ylabel(app.UIAxes6, '')
            app.UIAxes6.YLim = [-2.2 2.2];
            app.UIAxes6.XTickLabel = '';
            app.UIAxes6.Position = [526 245 117 89];
        end
    end

    methods (Access = public)

        % Construct app
        function app = MoveParticle

            % Create and configure components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end