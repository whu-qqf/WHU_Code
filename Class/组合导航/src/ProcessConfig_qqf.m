function cfg = ProcessConfig_qqf()

    param = Param();

    %% filepath
    cfg.imufilepath = 'dataset_qqf/IMU.txt';
    cfg.gnssfilepath = 'dataset_qqf/MOV.txt';
    cfg.odofilepath = 'dataset_qqf/ODO.txt';
    cfg.outputfolder = 'dataset_qqf';

    %% configure
    
    cfg.usegnssvel = true;
    cfg.useodonhc = true;
    cfg.useodoonly = false;
    cfg.usenhconly = false;
    if cfg.useodoonly==true && cfg.usenhconly==true
        disp('ODO/NHC都不使用？那就全部使用了');
        cfg.usenhconly=false;
        cfg.useodoonly=false;
    end
    cfg.odoupdaterate = 1; % [Hz]

    cfg.usezupt = false;

    %% initial information
    cfg.starttime = 527000;
    cfg.endtime = inf;

    cfg.initpos = [28.1421701503; 112.9585396051; 38.591]; % [deg, deg, m]
    cfg.initvel = [-0.273; -3.942; -0.020]; % [m/s]
    cfg.initatt = [1.40822092; -0.95903515; 267.77618701]; % [deg]

    cfg.initposstd = [0.05; 0.05; 0.1]; %[m]
    cfg.initvelstd = [0.05; 0.05; 0.05]; %[m/s]
    cfg.initattstd = [0.1; 0.1; 0.5]; %[deg]

    cfg.initgyrbias = [100; -300; -100]; % [deg/h]
    cfg.initaccbias = [-1500; -600; 0]; % [mGal]
    cfg.initgyrscale = [1000; 6000; -4000]; % [ppm]
    cfg.initaccscale = [2600; 8400; 300]; % [ppm]

    cfg.initgyrbiasstd = [48; 48; 48]; % [deg/h]
    cfg.initaccbiasstd = [50; 50; 50]; % [mGal]
    cfg.initgyrscalestd = [500; 500; 500]; % [ppm]
    cfg.initaccscalestd = [500; 500; 500]; % [ppm]

    cfg.gyrarw = 0.2; % [deg/sqrt(h)]
    cfg.accvrw = 0.2; % [m/s/sqrt(h)]
    cfg.gyrbiasstd = 48; % [deg/h]
    cfg.accbiasstd = 50; % [mGal]
    cfg.gyrscalestd = 500; % [ppm]
    cfg.accscalestd = 500; % [ppm]
    cfg.corrtime = 1; % [h]

    %% install parameters 安装参数
    cfg.antlever = [0.505; -0.145; -1.105]; % [m]
    cfg.odolever = [0.605; -1.025; 0.705]; %[m]
    cfg.installangle = [0.0; -0.532; 1.38]; %[deg]

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

