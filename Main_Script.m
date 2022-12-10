run('call_scripts.m')
load('Linear_Main.mat')

noise = false;
disturbance = false;
sample = false;

% m = 1.9;
I = [0.01151*1.1    0       0;
        0   0.01169*1.1      0;
        0       0   0.01275*1.1 ];


sim('Quadcopter_Model_V2',50)
logsout = ans.logsout;
graphing(logsout)

