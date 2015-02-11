function [latb longb h] = fGC2GL(XYZ)

% This function takes a vector with ECEF cartesian coordinates and transform 
% them into ellipsoidal coordinates (longitude,latitude and height).
%
% lolah=fGeoCen2GeoLat(XYZ)
%
% INPUT:
% XYZ - Cartesian coordinates 
%
% OUTPUT: 
% laloh - [Latitude (rad) Longitude (rad) Height (m)] on ellipsoid WGS 84
% help fGC2GL;
% Function written by Johan Vium Andersson
% Last updated 2005-02-22

    % reference ellipsoid data WGS84
        a=6378137;
        f=1/298.257223563;
        
  X = XYZ(1);
  Y = XYZ(2);
  Z = XYZ(3);
% Program

    b=a-f*a;
    
    ee=(a^2-b^2)/a^2;
    p=sqrt(X^2+Y^2);
	tata=atan(Z/(p*sqrt(1-ee)));
	longb=atan(Y/X);
	latb=atan((Z+((a*ee)/sqrt(1-ee))*sin(tata)^3)/(p-a*ee*(cos(tata))^3));
    N=a/sqrt(1-ee*(sin(latb))^2);

% resultat
   h = p/cos(latb)-N;
   
   laloh =[latb longb  h];