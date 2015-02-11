clear;clc

% whichconst = 84;
% typerun = 'c';
%%
% [satrec, startmfe, stopmfe, deltamin] = twoline2rvMOD(whichconst, longstr1,longstr2, typerun);
[satrec] = twoline2rvMOD(longstr1,longstr2);
%%
jdNow = jday;
tsince = (jdNow-satrec.jdsatepoch)*24*60;
%%
[satrec, r, v] = sgp4(satrec,tsince);