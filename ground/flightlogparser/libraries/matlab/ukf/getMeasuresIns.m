function [ raIn, xyIn, time, n, U ] = getMeasuresIns( path )
% 'E:\Booklist\video\msys2\home\Santy\SantyPilot
% \logs\2024_05_05_21_12_29_stylog.mat'
% clear all
load(path);
data_len = length(FILTERSTATES);
% X(0) = FILTERSTATES(1);

NUMX = 13;
NUMY = 10;
NUMU = 6;
X=zeros(NUMX,data_len);
U=zeros(NUMU,data_len);
Y=zeros(NUMY,data_len);
T=zeros(1, data_len);
for i = 1:data_len
    cell = FILTERSTATES(i);
    GPSNorth = getfield(cell{1}, 'GPSNorth');
    GPSEast = getfield(cell{1}, 'GPSEast');
    GPSDown = getfield(cell{1}, 'GPSDown');
    GPSVelNorth = getfield(cell{1}, 'GPSVelNorth');
    GPSVelEast = getfield(cell{1}, 'GPSVelEast');
    GPSVelDown = getfield(cell{1}, 'GPSVelDown');
    
    dT = getfield(cell{1}, 'dT');
    gx = getfield(cell{1}, 'gx');
    gy = getfield(cell{1}, 'gy');
    gz = getfield(cell{1}, 'gz');
    
    ax = getfield(cell{1}, 'ax');
    ay = getfield(cell{1}, 'ay');
    az = getfield(cell{1}, 'az');
    
    mx = getfield(cell{1}, 'mx');
    my = getfield(cell{1}, 'my');
    mz = getfield(cell{1}, 'mz');
    
    q1 = getfield(cell{1}, 'q1');
    q2 = getfield(cell{1}, 'q2');
    q3 = getfield(cell{1}, 'q3');
    q4 = getfield(cell{1}, 'q4');
    gyro_biasx = getfield(cell{1}, 'gyro_biasx');
    gyro_biasy = getfield(cell{1}, 'gyro_biasy');
    gyro_biasz = getfield(cell{1}, 'gyro_biasz');
    
    Roll = getfield(cell{1}, 'Roll');
    Pitch = getfield(cell{1}, 'Pitch');
    Yaw = getfield(cell{1}, 'Yaw');
    North = getfield(cell{1}, 'North');
    East = getfield(cell{1}, 'East');
    Down = getfield(cell{1}, 'Down');
    NorthVel = getfield(cell{1}, 'NorthVel');
    EastVel = getfield(cell{1}, 'EastVel');
    DownVel = getfield(cell{1}, 'DownVel');
    
    CalibratedAirspeed = getfield(cell{1}, 'CalibratedAirspeed');
    TrueAirspeed = getfield(cell{1}, 'TrueAirspeed');
    Altitude = getfield(cell{1}, 'Altitude');
    FlightTime = getfield(cell{1}, 'FlightTime');
    
    % estimated with ekf
    X(1,i) = str2double(North); 
    X(2,i) = str2double(East); 
    X(3,i) = str2double(Down);
    X(4,i) = str2double(NorthVel); 
    X(5,i) = str2double(EastVel); 
    X(6,i) = str2double(DownVel);
    X(7,i) = str2double(q1); 
    X(8,i) = str2double(q2); 
    X(9,i) = str2double(q3); 
    X(10,i) = str2double(q4);
    X(11,i) = str2double(gyro_biasx); 
    X(12,i) = str2double(gyro_biasy); 
    X(13,i) = str2double(gyro_biasz);
    
    U(1,i) = str2double(gx); 
    U(2,i) = str2double(gy); 
    U(3,i) = str2double(gz);
    U(4,i) = str2double(ax); 
    U(5,i) = str2double(ay); 
    U(6,i) = str2double(az);
    
    Y(1,i) = str2double(GPSNorth); 
    Y(2,i) = str2double(GPSEast); 
    Y(3,i) = str2double(GPSDown);
    Y(4,i) = str2double(GPSVelNorth); 
    Y(5,i) = str2double(GPSVelEast); 
    Y(6,i) = str2double(GPSVelDown);
    Y(7,i) = str2double(mx); 
    Y(8,i) = str2double(my); 
    Y(9,i) = str2double(mz);
    if i > 1
        T(1,i) = T(1, i - 1) + str2double(dT);
    else
        T(1, i) = str2double(dT);
    end
    % T(1,i) = str2double(FlightTime);
end
xyIn = X;
raIn = Y;
time = T;
n = data_len;