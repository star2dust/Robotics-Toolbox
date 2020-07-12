% use MATLAB's built-in 'vrjoystick'
close all
clear

joy = vrjoystick(2);% 1 for ST LIS3LV02DL Accelerometer; 2 for BETOP Controller
for i=1:1000
    ax = axis(joy);
    btn = button(joy);
    subplot(3,2,1);
    plot(ax(1),-ax(2),'bo');
    axis([-1 1 -1 1])
    title('Axis Left')
    
    subplot(3,2,2);
    plot(ax(3),-ax(4),'bo');
    axis([-1 1 -1 1])
    title('Axis Right')
    
    subplot(3,2,[3,4]);
    bar(ax);
    axis([0 7 -1 1])
    title('Axis 1-6')
    
    subplot(3,2,[5,6]);
    bar(btn);
    axis([0 13 0 1])
    title('Button 1-12')
    pause(0.1);
end