function [ string ] = sec2time(ETA)
% Convert e.g. 3661sec to 01:01:01
ETA_h   = floor(ETA/3600);
ETA_m   = floor((ETA-3600*ETA_h)/60);
ETA_s   = ETA - 3600*ETA_h - 60*ETA_m;
if ETA_h <10
    ETA_h = strcat(['0' num2str(ETA_h)]);
else
    ETA_h = num2str(ETA_h);
end
if ETA_m <10
    ETA_m = strcat(['0' num2str(ETA_m)]);
else
    ETA_m = num2str(ETA_m);
end
if ETA_s <10
    ETA_s = strcat(['0' num2str(ETA_s)]);
else
    ETA_s = num2str(ETA_s);
end
string  = strcat([ETA_h ':' ETA_m ':' ETA_s]);
end