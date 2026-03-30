% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2023.3.3
% -------------------------------------------------------------------------

function cfg = ProcessConfig3()

    param = Param();

    %% filepath
    cfg.imufilepath = 'dataset3/ADIS16465.txt';
    cfg.gnssfilepath = 'dataset3/GNSS-POS.txt';
    cfg.odofilepath = 'dataset3/ODO.txt';
    cfg.outputfolder = 'dataset3';

    %% configure
    cfg.usegnssvel = false;
    cfg.useodonhc = true;
    cfg.odoupdaterate = 10; % [Hz]

    %% initial information
    cfg.starttime = 292460;
    cfg.endtime = 293196;

    cfg.initpos = [30.5285037471; 114.3556962503; 21.091]; % [deg, deg, m]
    cfg.initvel = [0.0; 0.0; 0.0]; % [m/s]
    cfg.initatt = [0.25; 0.16; 175.6]; % [deg]

    cfg.initposstd = [0.1; 0.1; 0.2]; %[m]
    cfg.initvelstd = [0.1; 0.1; 0.1]; %[m/s]
    cfg.initattstd = [0.1; 0.1; 0.5]; %[deg]

    cfg.initgyrbias = [0; 0; 0]; % [deg/h]
    cfg.initaccbias = [0; 0; 0]; % [mGal]
    cfg.initgyrscale = [0; 0; 0]; % [ppm]
    cfg.initaccscale = [0; 0; 0]; % [ppm]

    cfg.initgyrbiasstd = [50; 50; 50]; % [deg/h]
    cfg.initaccbiasstd = [50; 50; 50]; % [mGal]
    cfg.initgyrscalestd = [1000; 1000; 1000]; % [ppm]
    cfg.initaccscalestd = [1000; 1000; 1000]; % [ppm]

    cfg.gyrarw = 0.1; % [deg/sqrt(h)]
    cfg.accvrw = 0.1; % [m/s/sqrt(h)]
    cfg.gyrbiasstd = 50; % [deg/h]
    cfg.accbiasstd = 50; % [mGal]
    cfg.gyrscalestd = 1000; % [ppm]
    cfg.accscalestd = 1000; % [ppm]
    cfg.corrtime = 1; % [h]

    %% install parameters 安装参数
    cfg.antlever = [-0.37; 0.008; 0.353]; % [m]
    cfg.odolever = [0; 0; 1.099]; %[m]
    cfg.installangle = [0; -0.338; -0.92]; %[deg]

    %% ODO/NHC measurement noise 观测噪声
    cfg.odonhc_measnoise = [0.1; 0.1; 0.1]; % [m/s]


    %% convert unit to standard unit (单位转换)
    cfg.initpos(1) = cfg.initpos(1) * param.D2R;
    cfg.initpos(2) = cfg.initpos(2) * param.D2R;
    cfg.initatt = cfg.initatt * param.D2R;

    cfg.initattstd = cfg.initattstd * param.D2R;

    cfg.initgyrbias = cfg.initgyrbias * param.D2R / 3600;
    cfg.initaccbias = cfg.initaccbias * 1e-5;
    cfg.initgyrscale = cfg.initgyrscale * 1e-6;
    cfg.initaccscale = cfg.initaccscale * 1e-6;
    cfg.initgyrbiasstd = cfg.initgyrbiasstd * param.D2R / 3600;
    cfg.initaccbiasstd = cfg.initaccbiasstd * 1e-5;
    cfg.initgyrscalestd = cfg.initgyrscalestd * 1e-6;
    cfg.initaccscalestd = cfg.initaccscalestd * 1e-6;

    cfg.gyrarw = cfg.gyrarw * param.D2R / 60;
    cfg.accvrw = cfg.accvrw / 60;
    cfg.gyrbiasstd = cfg.gyrbiasstd * param.D2R / 3600;
    cfg.accbiasstd = cfg.accbiasstd * 1e-5;
    cfg.gyrscalestd = cfg.gyrscalestd * 1e-6;
    cfg.accscalestd = cfg.accscalestd * 1e-6;
    cfg.corrtime = cfg.corrtime * 3600;

    cfg.installangle = cfg.installangle * param.D2R;
    cfg.cbv = euler2dcm(cfg.installangle);

end

