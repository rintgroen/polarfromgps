close all
clear all
clc

% Declared constants
NM = 1852;   % 1 Nautical Mile = 1852 meter, note 1 minute arclength along the earth surface is 1 NM
c1 = -0.9;   % Reference Moyes Litespeed S sink rate (m/s) polar point 1
v1 = 40/3.6; % Reference Moyes Litespeed S speed (m/s) polar point 1
c2 = -2.5;   % Reference Moyes Litespeed S sink rate (m/s) polar point 2
v2 = 80/3.6; % Reference Moyes Litespeed S speed (m/s) polar point 2

% Derived constants
a = (c1*v1-c2*v2)/(v1^4-v2^4); % polar constant a for use in relation c = a * v^3 + b / v
b = (c2*v2*v1^4-c1*v1*v2^4)/(v1^4-v2^4); % polar constant b for use in relation c = a * v^3 + b / v

% Plot reference polar
v = 5:35;
c = a*v.^3+b./v;
figure 1
plot(v,c);

% Load tracklog
% csv file should contain the following colomns: #, lat, lon, alt, date, time
[nr,lat,lon,alt,date,time] = textread( 'flight_sample.csv', '%u %f %f %f %s %s' ,'delimiter' , ',');
figure 2
plot(lon,lat);

% Determine time differences between tracklog points
datetime = strcat(date,', ',time);
datemat = datevec(datetime,'yyyy/mm/dd, HH:MM:SS');
dt = etime(datemat(2:end,:),datemat(1:end-1,:));

% Determine distances (arclen) and headings (az) between tracklogpoints
% (Azimuth angle is bearing wrt North)
[arclen,az] = distance([lat(1:end-1),lon(1:end-1)],[lat(2:end),lon(2:end)]);

% Calculate horizontal and vertical speeds
v_gps = arclen*60*NM*3.6./dt;
c_gps = (alt(2:end)-alt(1:end-1))./dt;

% Determine wind speed and direction
% 1. consider a polar plot with rho = speed and theta = heading
% 2. fit circle through tracklog points
% 3. offset of circle center and origin is the wind vector
% 4. do some filtering/smoothing
rho = v_gps;
theta = az;
% figure 3
% polar(theta,rho,'.');

xy = [rho.*cos(deg2rad(theta)),rho.*sin(deg2rad(theta))];
len = size(xy,1);
windsamplesize = 100;
wind = zeros(len-windsamplesize,4);
for i=1:len-windsamplesize
    circ = CircleFitByPratt(xy(i:i+windsamplesize,:));
    wind(i,1:2) = circ(1:2); % wind circle offset coordinates
end

wind(:,3) = sqrt(wind(:,1).^2+wind(:,2).^2); % wind strength
wind(:,4) = 270 - rad2deg(atan(wind(:,1)./wind(:,2))); % wind direction

windstrength = wind(:,3);
windstrength = windstrength(windstrength<20); % simple outlier cut off for now
winddirection = wind(:,4);

% Display wind
display(strcat('wind strength = ',num2str(median(windstrength))))
display(strcat('wind direction = ',num2str(median(winddirection))))
figure 4
plot(windstrength);
figure 5
plot(winddirection);
figure 6
plot(rho.*cos(deg2rad(theta)),rho.*sin(deg2rad(theta)),'.');
hold
plot(windstrength.*cos(deg2rad(winddirection)),windstrength.*sin(deg2rad(winddirection)),'r.');

% Next, to determine aerodynamic sink rate, I guess it should be possible with the following relations ..
%
% For every segment i, between two tracklog points:
%
% v_gps(i) = v_tas(i) * cos( gamma(i) ) + v_wind(i),
% with v_gps is horizontal component of velocity over ground, v_tas is true
% airspeed, gamma is glide angle, v_wind is horizontal component of wind velocity
%
% c_gps(i) = c_true(i) + c_turb(i), [1]
% with c_gps the vertical component of the gps velocity, c_true the aerodynamic
% sink rate, and c_turb the vertical disturbance due to sink/lift
%
% v_tas(i)^2 = (v_gps(i)-v_wind(i))^2 + c_true(i)^2 [2]
%
% c_true(i) = p1(i) * v_tas(i)^3 + p2(i) / v_tas(i), [3]
% p1(i) = (c_true(i-1)*v_tas(i-1)-c_true(i)*v_tas(i))/(v_tas(i-1)^4-v_tas(i)^4),
% p2(i) = (c_true(i)*v_tas(i)*v_tas(i-1)^4-c_true(i-1)*v_tas(i-1)*v_tas(i)^4)/(v_tas(i-1)^4-v_tas(i)^4)
%
% Since we already calculated c_gps, v_gps, v_wind, the 3 relations [1],[2],[3] hold 3 unknowns: c_true, c_turb, v_tas.






% Non organized stuff ..

%dazdt = (az(2:end)-az(1:end-1))./(0.5*dt(2:end)+0.5*dt(1:end-1));
%turns = dazdt>10;
%straight = dazdt<1;

%c1g = c_gps(3070:3210);
%c2g = c_gps(3071:3211);
%v1g = v_gps(3070:3210);
%v2g = v_gps(3071:3211);
%
%a_gps = (c1g.*v1g-c2g.*v2g)./(v1g.^4-v2g.^4);
%b_gps = (c2g.*v2g.*v1g.^4-c1g.*v1g.*v2g.^4)./(v1g.^4-v2g.^4);




