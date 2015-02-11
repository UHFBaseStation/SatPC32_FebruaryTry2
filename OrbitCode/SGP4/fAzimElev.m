function [A,E] = fAzimElev(satEcef,recEcef)

% This function calculates azimuth and elevation angles between a 
% reference receiver on earth and a satellite.
%
% INPUT :
% satEcef - satellite coordinates, earth centered earth framed (vector)
% recEcef - receiver coordinates, earth centered earth framed (vector)
%
% OUTPUT: 
% A - Azimuth angle in radians  
% E - elevation angle in radians
% help fAzimElev;
% References: B.Hoffman-Wellenhof et al. (2001) pp 148-149
%
% Function written by Johan Vium Andersson
% Last updated 2005-02-22

    d = satEcef - recEcef;

    r = sqrt (sum(d.^2));
    
    rho = d./r;
  
    [laloh(1), laloh(2), laloh(3)] = fGC2GL(recEcef); % Calculate ellipsoidal coordinates in 
                                     % WGS 84 and not ITRF which should be 
                                     % the correct. The influence is very 
                                     % small and therefore neglectable.
    
      n =   [ -sin(laloh(1))*cos(laloh(2));
              -sin(laloh(1))*sin(laloh(2));
               cos(laloh(1))];

      e =   [ -sin(laloh(2));
               cos(laloh(2));
               0];

      u =   [cos(laloh(1))*cos(laloh(2));
             cos(laloh(1))*sin(laloh(2));
             sin(laloh(1))];
   z = acos(sum(rho'.*u));
  
    E = pi/2 - z; % Elevation

    A = atan2(sum(rho'.*e), sum(rho'.*n)); % Azimuth
    if A < 0
        A = A + 2*pi;
    end
     