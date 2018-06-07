
clear all
incre=90;
%testdis=zeros(1,incre^2);
%boatpsx=zeros(1,incre^2);
%boatpsy=zeros(1,incre^2);
testdis=zeros(1,incre);
boatpsx=zeros(1,incre);
boatpsy=zeros(1,incre);
pres=2*pi/incre;
angles=0:pres:2*pi;
%boat param [xpos ypos v bangle boatmass wcfd acfd watercrss aircrss]
boat_init_param=[0 0 0.05 0 20000 0.09 0.04 10 10];
%dest [xpos ypos]
destination=[3000000 6000000];
%sail [area angle wcfd acfd efficiency]
sail_init_param=[100 0 0 0.09 1];
sare=sail_init_param(1);
%keel [area angle wcfd acfd efficiency]
keel_init_param=[4 pi .04 0 1];
kare=keel_init_param(1);
keff=keel_init_param(5);
seff=sail_init_param(5);
 %air speeds
 airy=windspeed;
 airx=windspeed;
 angair=atan(airy/airx);
 airtot=15*sqrt(mod(airy^2+airx^2,1));
 airy=airtot*sin(angair);
 airx=airtot*cos(angair);
 %water speeds
 waty=windspeed;
 watx=windspeed;
 angw=atan(waty/watx);
 wattot=2.5*sqrt(mod(waty^2+watx^2,1));
 waty=wattot*sin(angw);
 watx=wattot*cos(angw);
% angair=0;
% angw=0;
% airx=1;
% airy=0;
% watx=0;
% waty=0;
%densities
watden=1;%kg/m^3
airden=0.001225;%kg/m^3
%sail/keel angle
sangle=sail_init_param(2);
kangle=keel_init_param(2);
%wind/sail/water interval time interval in seconds
dt=100;
%sail change time ignored for now
sailchangetime=.1;
boatposx=boat_init_param(1);
boatposy=boat_init_param(2);
step=1;
tim=0;
bm=boat_init_param(5);
dirang=boat_init_param(4);
velmag=boat_init_param(3);
velboatx=velmag*sin(boat_init_param(4));
velboaty=velmag*cos(boat_init_param(4));

tolanddis=sqrt((boatposx(step)-destination(1))^2+...
        ((boatposy(step)-destination(2)))^2);
count=1;
counter=1;
hour=3600/dt;
rigop=0:0.1:1;
dismag=sqrt(destination(1)^2+destination(2)^2);
%moving the boat

adragco=1/2*airden*boat_init_param(7)*boat_init_param(9);
wdragco=1/2*watden*boat_init_param(6)*boat_init_param(8);
sailcoeff=sare*seff*airden*dt;
keelcoeff=kare*keff*watden*dt;
flag=1;
I=1;
Bestsearch(I)=0;
signage=0;
while (flag)
    clc
    xdis=destination(1)-boatposx(step);
    ydis=destination(2)-boatposy(step);
    fprintf('Calculating...\n Distance to land =%f km \n xdis=%f \n ydis=%f \n Best metric =%d \n sign=%f \n',tolanddis/1000,xdis,ydis, Bestsearch(I),signage)
    if (tolanddis>50000)
    angletodes=atan((boatposy(step)-destination(2))/(boatposx(step)-destination(1)));
    %relative speeds
    arelx=airx(count)-velboatx(step);
    arely=airy(count)-velboaty(step);
    wrelx=watx(count)-velboatx(step);
    wrely=waty(count)-velboaty(step);
    %Drag forces
    afx=adragco*arelx^2;
    afy=adragco*arely^2;
    wfx=wdragco*wrelx^2;
    wfy=wdragco*wrely^2;
    %Sail/Keel Forces possibilities
    for i=1:incre
        %for j=1:length(rigop)
            %for j=1:incre
                %ii=i+(j-1)*incre;
                ii=i;
                sfx=sailcoeff*arelx*cos(angair-angles(i));
                sfy=sailcoeff*arely*cos(angair-angles(i));
                %kfx=keelcoeff*wrelx*cos(angw-angles(j));
                %kfy=keelcoeff*wrely*sin(angw-angles(j));
                kfx=keelcoeff*wrelx*cos(angw-kangle(step));
                kfy=keelcoeff*wrely*cos(angw-kangle(step));
                %total forces
                fx=(sfx+kfx+afx+wfx)/bm;
                fy=(sfy+kfy+afy+wfy)/bm;
                velboatxtst(ii)=velboatx(step)+fx*dt;
                velboatytst(ii)=velboaty(step)+fy*dt;
                velboatmagtst(ii)=velboatxtst(ii)^2+velboatytst(ii)^2;
                topointvel(ii)=velboatmagtst(ii)*cos(abs(angletodes-atan(velboatytst(ii)/velboatxtst(i))));
                boatpsx(ii)=boatposx(step)+velboatxtst(ii)*dt;
                boatpsy(ii)=boatposy(step)+velboatytst(ii)*dt;
                testdis(ii)=sqrt((boatpsx(ii)-destination(1))^2+...
                (boatpsy(ii)-destination(2))^2);
            %end
    end
    %Choosing best angles
    for ii=1:length(topointvel)
    Bestsearch(ii)=sign(topointvel(ii))*abs(topointvel(ii)*(1/testdis(ii)));
    end
    signage=sign(topointvel(ii));
    [M I]=max(Bestsearch);
   if (sign(Bestsearch)==-1)
       boatposx(step+1)=boatposx(step)+velboatx(step)*dt;
       boatposy(step+1)=boatposy(step)+velboatx(step)*dt;
       velboatx(step+1)=velboatx(step);
       velboaty(step+1)=velboaty(step);
       dirang(step+1)=dirang(step);
       sangle(step+1)=sangle(step);
       tim(step+1)=tim(step)+dt/86400;
       step=step+1;
       kangle(step)=dirang(step);
   else
    imax=mod(I,incre);
    if imax==0
        imax=i;
    end
   % jmax=(I-imax)/incre+1;
    %new Velocities
    velboatx(step+1)=velboatxtst(I);
    velboaty(step+1)=velboatytst(I);
    %new position
    boatposx(step+1)=boatpsx(I);
    boatposy(step+1)=boatpsy(I);
    tim(step+1)=tim(step)+dt/86400;
    step=step+1;
    dirang(step)=atan(velboaty(step)/velboatx(step));
    sangle(step)=angles(I);
    %kangle(step)=angles(jmax);
    kangle(step)=dirang(step);
    %checking to change air and water speeds once an hour
   end
    tolanddis=sqrt((boatposx(step)-destination(1))^2+...
    (boatposy(step)-destination(2))^2);

     if (count>hour|(tolanddis<50000))
         counter=counter+1;
         count=1;
%          figure(1)
%          clf(1)
% %         subplot(2,2,1)
% %         %ploting boat movement
% %         hold on
% %         plot(destination(1)/1000,destination(2)/1000,'.k','Markersize',5)
% %         plot(boatposx(step)/1000,boatposy(step)/1000,'*r','Markersize',5)
% %         title('Boat Position')
% %         axis([0 destination(1)/1000 0 destination(2)/1000])
% %         hold off
% %         subplot(2,2,2)
% %         hold on
% %         plot(tim(step),tolanddis/1000,'.b','Markersize',5)
% %         title('Distance To Land vs Time')
% %         xlabel('Time (days)')
% %         ylabel('Distance (km)')
% %         axis([0 15 0 dismag/1000])
% %         hold off
% %         subplot(2,2,3)
% %         hold on
% %         plot(sangle(step),tolanddis/1000,'.b','Markersize',5)
% %         axis([0 2*pi 0 dismag/1000])
% %         hold off
% %         subplot(2,2,4)
% %         hold on
% %         plot(kangle(step),tolanddis/1000,'.b','Markersize',5)
% %         axis([0 2*pi 0 dismag/1000])
% %         hold off
% %         drawnow
         airy(counter)=windspeed;
         airx(counter)=windspeed;
         angair=atan(airy(counter)/airx(counter));
         airtot(counter)=15*sqrt(mod(airy(counter)^2+airx(counter)^2,1));
         airy(counter)=airtot(counter)*sin(angair);
         airx(counter)=airtot(counter)*cos(angair);
         %new water speeds
         waty(counter)=windspeed;
         watx(counter)=windspeed;
         angw=atan(waty(counter)/watx(counter));
         wattot(counter)=2.5*sqrt(mod(waty(counter)^2+watx(counter)^2,1));
         waty(counter)=wattot(counter)*sin(angw);
         watx(counter)=wattot(counter)*cos(angw);

     end
     %count=count+1;

    %new air speeds
    %angle to destination
    %dirang=atan((boatposy(step)-destination(2))/(boatposx(step)-destination(1)));
    else
        flag=0;
    end

    pause(0.001)

end
fprintf('Plotting...\n')
for plt=1:(length(boatposx)/100)
    figure(1)
    clf(1)
    hold on
    plot(destination(1)/1000,destination(2)/1000,'*k','Markersize',10)
    plot(boatposx(plt*100)/1000,boatposy(plt*100)/1000,'*r','Markersize',6)
    title('Boat Position')
    xlabel('Kilometers')
    ylabel('Kilometers')
    axis([0 destination(1)/800 0 destination(2)/800])
    drawnow
end
plot(boatposx/1000,boatposy/1000)
hold off
figure(2)
plot(kangle,tim,'.r',sangle,tim,'.b')
title('Boat Position')
xlabel('Angle (rad)')
ylabel('Time(s)')