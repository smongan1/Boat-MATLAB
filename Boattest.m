
clear all

incre=20;
pres=2*pi/incre;
angles=0:pres:2*pi;

%boat param [xpos ypos v bangle boatmass wcfd acfd watercrss aircrss]
boat_init_param=[0 0 0.05 0 20000 0.09 0.04 10 10];

%dest [xpos ypos]
destination=[000000 6000000];

%sail [area angle wcfd acfd efficiency]
sail_init_param=[1000 0 0 0.09 1];
sare=sail_init_param(1);

%keel [area angle wcfd acfd efficiency]
keel_init_param=[50 pi .04 0 1];
kare=keel_init_param(1);

keff=keel_init_param(5);
seff=sail_init_param(5);

 %air speeds
 airy=windspeed;
 airx=windspeed;
 angair=arctan(airx,airy);
 airtot=25*sqrt(mod(airy^2+airx^2,1));
 airy=airtot*sin(angair);
 airx=airtot*cos(angair);
 
 %water speeds
 waty=windspeed;
 watx=windspeed;
 angw=arctan(watx,waty);
 wattot=.005*sqrt(mod(waty^2+watx^2,1));
 waty=wattot*sin(angw);
 watx=wattot*cos(angw);

%densities
watden=1;%kg/m^3
airden=0.001225;%kg/m^3

%sail/keel angle
sangle=sail_init_param(2);
kangle=keel_init_param(2);

%wind/sail/water interval time interval in seconds
dt=1000;

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
count=0;
counter=1;
hour=3600/dt;
rigop=0:0.1:1;
dismag=sqrt(destination(1)^2+destination(2)^2);
%moving the boat

adragco=boat_init_param(7)*boat_init_param(9);
wdragco=boat_init_param(6)*boat_init_param(8);
sailcoeff=sare*seff;
keelcoeff=kare*keff;
flag=1;
I=1;
Bestsearch(I)=0;
signage=0;
metric=5e4;
flagin=1;
while (flag)
    clc
    xdis=boatposx(step);
    ydis=boatposy(step);

    velmag=sqrt(velboatx(step)^2+velboaty(step)^2);
    velangle=arctan(velboatx(step),velboaty(step));
    fprintf('Calculating...\n Distance to land =%f km \n xdis=%f \n ydis=%f \n time elapsed=%d',tolanddis(step)/1000,xdis,ydis,tim(step))
    if (tolanddis(step)>50000)

%boatstats=[boatposx1 boatposy2 boatvelocityangle3 boatvelocitymagnitude4 
%sailarea*saileffectiveness5 keelarea*keeleffectiveness6 
%airdragcoeffcient7 waterdragcoefficient8 boatmass9]
    boatstats=[boatposx(step) boatposy(step) velangle velmag sailcoeff keelcoeff adragco wdragco bm];
    
%airstats=[airangle airvelocitymagnitude airdensity]
    
    airstats=[angair airtot airden];
    
%waterstats=[waterangle watervelocitymagnitude waterdensity]

    waterstats=[angw wattot watden];
    
    step=step+1;
    
    [sangle(step) kangle(step) boatposx(step) boatposy(step) velboatx(step) velboaty(step)]=...
        Bestangle(angles, angles, airstats, waterstats, destination, boatstats, dt);

   
%      boatposx(step)=boatposx(step-1)+velboatx(step)*dt;
%      boatposy(step)=boatposy(step-1)+velboaty(step)*dt;
    
    tolanddis(step)=sqrt((boatposx(step)-destination(1))^2+...
    (boatposy(step)-destination(2))^2);

    tim(step)=tim(step-1)+dt/86400;
    
    if (count>hour)
        count=0;
        airy=windspeed;
        airx=windspeed;
        angair=arctan(airx,airy);
        airtot=25*sqrt(mod(airy^2+airx^2,1));
        airy=airtot*sin(angair);
        airx=airtot*cos(angair);
        %water speeds
        waty=windspeed;
        watx=windspeed;
        angw=arctan(watx,waty);
        wattot=.005*sqrt(mod(waty^2+watx^2,1));
        waty=wattot*sin(angw);
        watx=wattot*cos(angw);
    end
    else
        flag=0;
    end
 figure(1)    
 plot(boatposx(step),boatposy(step),'*r')
 hold on
 %plot(destination(1),destination(2),'*k')
 quiver(boatposx(step),boatposy(step),airx*metric*adragco*airden,airy*metric*adragco*airden,'y')
 quiver(boatposx(step),boatposy(step),watx*metric*wdragco*watden,waty*metric*wdragco*watden,'b')
 quiver(boatposx(step),boatposy(step),velboatx(step)*metric,velboaty(step)*metric,'k')
 drawnow
 hold off
    count=count+1;

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