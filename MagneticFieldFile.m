classdef MagneticFieldFile < matlab.apps.AppBase
%%Andrew Truong
    % Properties that correspond to app components
    properties (Access = public)
        UIFigure              matlab.ui.Figure
        UIAxes                matlab.ui.control.UIAxes
        RadiusSliderLabel     matlab.ui.control.Label
        RadiusSlider          matlab.ui.control.Slider
        SpacingSliderLabel    matlab.ui.control.Label
        SpacingSlider         matlab.ui.control.Slider
        dLSliderLabel         matlab.ui.control.Label
        dLSlider              matlab.ui.control.Slider
        XampSliderLabel       matlab.ui.control.Label
        XampSlider            matlab.ui.control.Slider
        YampSliderLabel       matlab.ui.control.Label
        YampSlider            matlab.ui.control.Slider
        ZampSliderLabel       matlab.ui.control.Label
        ZampSlider            matlab.ui.control.Slider
        FieldVolSliderLabel   matlab.ui.control.Label
        FieldVolSlider        matlab.ui.control.Slider
        NoArrowsSliderLabel   matlab.ui.control.Label
        NoArrowsSlider        matlab.ui.control.Slider
        NoStreamSpinnerLabel  matlab.ui.control.Label
        NoStreamSpinner       matlab.ui.control.Spinner
        UIAxes2               matlab.ui.control.UIAxes
        UIAxes3               matlab.ui.control.UIAxes
        UIAxes4               matlab.ui.control.UIAxes
        NormalizedStandardDeviationinOutputLabel  matlab.ui.control.Label
        DistributionofMagneticVectorsLabel  matlab.ui.control.Label
    end

    
    methods (Access = private)
        function [DL] = BiotSavart(~,r,I,fn)
            t=0:pi/fn:2*pi;
            xdl=I(1)*[-r*sin(t(:)) zeros(length(t),1) r*cos(t(:))];
            xdt=I(2)*[-r*sin(t(:)) zeros(length(t),1) r*cos(t(:))];
            ydl= I(3)*[zeros(length(t),1) r*cos(t(:)) -r*sin(t(:))];
            ydt=I(4)*[zeros(length(t),1) r*cos(t(:)) -r*sin(t(:))];
            zdl= I(5)*[-r*sin(t(:)) r*cos(t(:)) zeros(length(t),1)];
            zdt=I(6)*[-r*sin(t(:)) r*cos(t(:)) zeros(length(t),1)];
            DL=[xdl;xdt;ydl;ydt;zdl;zdt];
        end
    end
    

    methods (Access = private)

        % Callback function: FieldVolSlider, FieldVolSlider, 
        % NoArrowsSlider, NoStreamSpinner, RadiusSlider, 
        % SpacingSlider, XampSlider, YampSlider, ZampSlider, dLSlider
        function MagneticField(app, event)
        app.UIAxes.cla;
        
        r = app.RadiusSlider.Value;
        s = app.SpacingSlider.Value;
        fn = app.dLSlider.Value;
        us = app.XampSlider.Value;
        ws = app.YampSlider.Value;
        es = app.ZampSlider.Value;
        fl = app.FieldVolSlider.Value;
        na = app.NoArrowsSlider.Value;
        ns = app.NoStreamSpinner.Value;
        
        t = 0:pi/fn:2*pi;
        xl=[r*cos(t(:)) s*ones(length(t),1) r*sin(t(:))];
        xt=[r*cos(t(:)) -s*ones(length(t),1) r*sin(t(:))];
        yl=[s*ones(length(t),1) 0.7*r*sin(t(:)) 0.7*r*cos(t(:))];
        yt=[-s*ones(length(t),1) 0.7*r*sin(t(:)) 0.7*r*cos(t(:))];
        zl=[0.65*r*cos(t(:)) 0.65*r*sin(t(:)) s*ones(length(t),1)];
        zt=[0.65*r*cos(t(:)) 0.65*r*sin(t(:)) -s*ones(length(t),1)];
        L=[xl;xt;yl;yt;zl;zt];
        
        plot3(app.UIAxes,xl(:,1),xl(:,2),xl(:,3),'b');hold(app.UIAxes,'all');
        plot3(app.UIAxes,xt(:,1),xt(:,2),xt(:,3),'b');hold(app.UIAxes,'all');
        plot3(app.UIAxes,yl(:,1),yl(:,2),yl(:,3),'r');hold(app.UIAxes,'all');
        plot3(app.UIAxes,yt(:,1),yt(:,2),yt(:,3),'r');hold(app.UIAxes,'all');
        plot3(app.UIAxes,zl(:,1),zl(:,2),zl(:,3),'g');hold(app.UIAxes,'all');
        plot3(app.UIAxes,zt(:,1),zt(:,2),zt(:,3),'g');hold(app.UIAxes,'all');
        
        [DL]=BiotSavart(app,r,[us,us,ws,ws,es,es],fn);
        
        d=-fl:fl/na:fl;
        [X,Y,Z]=meshgrid(d,d,d);
        R=[X(:) Y(:) Z(:)];
        qv=zeros(size(R));
        q=qv;
        for j=1:length(R)
            for i=1:length(DL)
                a=R(j,:)-L(i,:);
                qv(i,:)=cross(DL(i,:),a)/norm(a)^3;
            end
            q(j,:)=sum(qv,'omitnan');
        end
        g=size(X,1);
        u=reshape(q(:,1),[g,g,g]);
        v=reshape(q(:,2),[g,g,g]);
        w=reshape(q(:,3),[g,g,g]);
        plot(app.UIAxes2,abs(q(:,1)))
        plot(app.UIAxes3,abs(q(:,2)))
        plot(app.UIAxes4,abs(q(:,3)))      
        disp([std(q(:,1))/mean(q(:,1)) std(q(:,2))/mean(q(:,2)) std(q(:,3))/mean(q(:,3))]);
        quiver3(app.UIAxes,X,Y,Z,u,v,w);hold(app.UIAxes,'all');
        x=-0.9*fl:0.9*fl/ns:0.9*fl;
        [x,y,z]=meshgrid(x,x,x);
        streamline(app.UIAxes,stream3(X,Y,Z,u,v,w,x,y,z));
        end
    end

    % App initialization and construction
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure
            app.UIFigure = uifigure;
            app.UIFigure.Position = [100 100 800 551];
            app.UIFigure.Name = 'UI Figure';

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            title(app.UIAxes, {'Magnetic Field of Cartesian Helmholtz Coils '; 'Calculated Iteratively Using Biot-Savarts Law '})
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            app.UIAxes.XLim = [-1 1];
            app.UIAxes.YLim = [-1 1];
            app.UIAxes.ZLim = [-1 1];
            app.UIAxes.Position = [350 197 420 333];

            % Create RadiusSliderLabel
            app.RadiusSliderLabel = uilabel(app.UIFigure);
            app.RadiusSliderLabel.HorizontalAlignment = 'right';
            app.RadiusSliderLabel.Position = [638 109 43 22];
            app.RadiusSliderLabel.Text = 'Radius';

            % Create RadiusSlider
            app.RadiusSlider = uislider(app.UIFigure);
            app.RadiusSlider.Limits = [0.1 2.5];
            app.RadiusSlider.ValueChangingFcn = createCallbackFcn(app, @MagneticField, true);
            app.RadiusSlider.Position = [707 118 52 3];
            app.RadiusSlider.Value = 1;

            % Create SpacingSliderLabel
            app.SpacingSliderLabel = uilabel(app.UIFigure);
            app.SpacingSliderLabel.HorizontalAlignment = 'right';
            app.SpacingSliderLabel.Position = [637 164 49 22];
            app.SpacingSliderLabel.Text = 'Spacing';

            % Create SpacingSlider
            app.SpacingSlider = uislider(app.UIFigure);
            app.SpacingSlider.Limits = [0.25 1];
            app.SpacingSlider.ValueChangingFcn = createCallbackFcn(app, @MagneticField, true);
            app.SpacingSlider.Position = [707 173 52 3];
            app.SpacingSlider.Value = 0.5;

            % Create dLSliderLabel
            app.dLSliderLabel = uilabel(app.UIFigure);
            app.dLSliderLabel.HorizontalAlignment = 'right';
            app.dLSliderLabel.Position = [638 50 25 22];
            app.dLSliderLabel.Text = 'dL';

            % Create dLSlider
            app.dLSlider = uislider(app.UIFigure);
            app.dLSlider.Limits = [20 50];
            app.dLSlider.ValueChangingFcn = createCallbackFcn(app, @MagneticField, true);
            app.dLSlider.Position = [707 60 52 3];
            app.dLSlider.Value = 20;

            % Create XampSliderLabel
            app.XampSliderLabel = uilabel(app.UIFigure);
            app.XampSliderLabel.HorizontalAlignment = 'right';
            app.XampSliderLabel.Position = [57 495 37 22];
            app.XampSliderLabel.Text = 'Xamp';

            % Create XampSlider
            app.XampSlider = uislider(app.UIFigure);
            app.XampSlider.ValueChangingFcn = createCallbackFcn(app, @MagneticField, true);
            app.XampSlider.Position = [115 504 150 3];

            % Create YampSliderLabel
            app.YampSliderLabel = uilabel(app.UIFigure);
            app.YampSliderLabel.HorizontalAlignment = 'right';
            app.YampSliderLabel.Position = [57 440 36 22];
            app.YampSliderLabel.Text = 'Yamp';

            % Create YampSlider
            app.YampSlider = uislider(app.UIFigure);
            app.YampSlider.ValueChangingFcn = createCallbackFcn(app, @MagneticField, true);
            app.YampSlider.Position = [114 449 150 3];

            % Create ZampSliderLabel
            app.ZampSliderLabel = uilabel(app.UIFigure);
            app.ZampSliderLabel.HorizontalAlignment = 'right';
            app.ZampSliderLabel.Position = [57 385 36 22];
            app.ZampSliderLabel.Text = 'Zamp';

            % Create ZampSlider
            app.ZampSlider = uislider(app.UIFigure);
            app.ZampSlider.ValueChangingFcn = createCallbackFcn(app, @MagneticField, true);
            app.ZampSlider.Position = [114 394 150 3];

            % Create FieldVolSliderLabel
            app.FieldVolSliderLabel = uilabel(app.UIFigure);
            app.FieldVolSliderLabel.HorizontalAlignment = 'right';
            app.FieldVolSliderLabel.Position = [50 314 48 22];
            app.FieldVolSliderLabel.Text = 'FieldVol';

            % Create FieldVolSlider
            app.FieldVolSlider = uislider(app.UIFigure);
            app.FieldVolSlider.Limits = [0.1 1];
            app.FieldVolSlider.ValueChangedFcn = createCallbackFcn(app, @MagneticField, true);
            app.FieldVolSlider.ValueChangingFcn = createCallbackFcn(app, @MagneticField, true);
            app.FieldVolSlider.Position = [119 323 150 3];
            app.FieldVolSlider.Value = 0.1;

            % Create NoArrowsSliderLabel
            app.NoArrowsSliderLabel = uilabel(app.UIFigure);
            app.NoArrowsSliderLabel.HorizontalAlignment = 'right';
            app.NoArrowsSliderLabel.Position = [42 248 58 22];
            app.NoArrowsSliderLabel.Text = 'NoArrows';

            % Create NoArrowsSlider
            app.NoArrowsSlider = uislider(app.UIFigure);
            app.NoArrowsSlider.Limits = [1 5];
            app.NoArrowsSlider.ValueChangingFcn = createCallbackFcn(app, @MagneticField, true);
            app.NoArrowsSlider.Position = [121 257 150 3];
            app.NoArrowsSlider.Value = 1;

            % Create NoStreamSpinnerLabel
            app.NoStreamSpinnerLabel = uilabel(app.UIFigure);
            app.NoStreamSpinnerLabel.HorizontalAlignment = 'right';
            app.NoStreamSpinnerLabel.Position = [50 185 59 22];
            app.NoStreamSpinnerLabel.Text = 'NoStream';

            % Create NoStreamSpinner
            app.NoStreamSpinner = uispinner(app.UIFigure);
            app.NoStreamSpinner.ValueChangingFcn = createCallbackFcn(app, @MagneticField, true);
            app.NoStreamSpinner.Limits = [1 5];
            app.NoStreamSpinner.Position = [124 185 100 22];
            app.NoStreamSpinner.Value = 1;

            % Create UIAxes2
            app.UIAxes2 = uiaxes(app.UIFigure);
            title(app.UIAxes2, '')
            xlabel(app.UIAxes2, '')
            ylabel(app.UIAxes2, '')
            app.UIAxes2.XTickLabel = '';
            app.UIAxes2.Position = [31 22 193 130];

            % Create UIAxes3
            app.UIAxes3 = uiaxes(app.UIFigure);
            title(app.UIAxes3, '')
            xlabel(app.UIAxes3, '')
            ylabel(app.UIAxes3, '')
            app.UIAxes3.XTickLabel = '';
            app.UIAxes3.Position = [235 22 193 130];

            % Create UIAxes4
            app.UIAxes4 = uiaxes(app.UIFigure);
            title(app.UIAxes4, '')
            xlabel(app.UIAxes4, '')
            ylabel(app.UIAxes4, '')
            app.UIAxes4.XTickLabel = '';
            app.UIAxes4.Position = [427 22 193 130];

            % Create NormalizedStandardDeviationinOutputLabel
            app.NormalizedStandardDeviationinOutputLabel = uilabel(app.UIFigure);
            app.NormalizedStandardDeviationinOutputLabel.Position = [235 9 225 22];
            app.NormalizedStandardDeviationinOutputLabel.Text = 'Normalized Standard Deviation in Output';

            % Create DistributionofMagneticVectorsLabel
            app.DistributionofMagneticVectorsLabel = uilabel(app.UIFigure);
            app.DistributionofMagneticVectorsLabel.FontWeight = 'bold';
            app.DistributionofMagneticVectorsLabel.Position = [252 151 191 22];
            app.DistributionofMagneticVectorsLabel.Text = 'Distribution of Magnetic Vectors';
        end
    end

    methods (Access = public)

        % Construct app
        function app = MagneticFieldFile

            % Create and configure components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

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