
% delete all variables
clear;

% close existing figure windows
close ALL;

% load array from file into matlab
% note that columns can be separated by space / tab or commas in matlab
A = load('my_sim.csv');

t  = A(:,1);
wb = A(:,2); % Reading at A1
wf = A(:,3); % Reading at A2
y3 = A(:,4); % Reading at A5
r_rpm = A(:,5); % rpm reference
angle = A(:,6); % steering angle

figure(1);
plot(t,wb);
ylabel('Back Wheel velocity wb (rad/s)');
xlabel('time (s)');

figure(2);
plot(t,wf);
ylabel('Front velocity, v (m/s)');
xlabel('time (s)');

figure(3);
plot(t,y3);
ylabel('y3');
xlabel('time (s)');

figure(4);
plot(t,r_rpm);
ylabel('r_rpm');
xlabel('time (s)');

figure(5);
plot(t,angle);
ylabel('angle Pulse width');
xlabel('time (s)');
