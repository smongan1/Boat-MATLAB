function [ sailangle keelangle newboatposx newboatposy newboatvelocityx newboatvelocityy] = Bestangle( sailangles, keelangles, airstats, waterstats, destination, boatstats, dt )
%sailangles is the possible sail positions 
%keelangles is the possible keel positions
%airstats should be a vector with:
%airstats=[airangle airvelocitymagnitude airdensity]
%waterstats should be a vector with:
%waterstats=[waterangle watervelocitymagnitude waterdensity]
%destination should be a vector with:
%destination=[destinationxpos destinationypos]
%boatstats should be a vector with:
%boatstats=[boatposx1 boatposy2 boatvelocityangle3 boatvelocitymagnitude4... 
%sailarea*saileffectiveness5 keelarea*keeleffectiveness6... 
%airdragcoeffcient7 waterdragcoefficient8 boatmass9]
%   Using this information this function will output the keel position and
%   sail posistion that will minimize the distance between the boat and its
%   destination and the positions and velocities that result.

s=length(sailangles);
k=length(keelangles);

%The following line preallocates space for the matricies used in finding
%the best angles for the sail and keel

testmetric=zeros(1,s*k+1);testdistances=zeros(s,k); boatpositionsx=zeros(s,k); boatpositionsy=zeros(s,k); boatvelocitiesx=zeros(s,k); boatvelocitiesy=zeros(s,k); sailtestangles=zeros(s,k); keeltestangles=zeros(s,k);

%These lines 29-41 get the relative velocities of the air with the boat and
%the water with the boat

relativeairspeedx=airstats(2)*cos(airstats(1))-boatstats(4)*cos(boatstats(3));

relativeairspeedy=airstats(2)*sin(airstats(1))-boatstats(4)*sin(boatstats(3));

relativeairspeedmagnitde=relativeairspeedx^2+relativeairspeedy^2;

relativewaterspeedx=waterstats(2)*cos(waterstats(1))...
                    -boatstats(4)*cos(boatstats(3));
                
relativewaterspeedy=waterstats(2)*sin(waterstats(1))...
                    -boatstats(4)*sin(boatstats(3));
                
relativewaterspeedmagnitude=relativewaterspeedx^2+relativewaterspeedy^2;

%Lines 45-53 find the drag assciated with the air and water

accelerationduetoairdragx=.5*airstats(3)*boatstats(7)*relativeairspeedx*relativeairspeedmagnitde/boatstats(9);
accelerationduetoairdragy=.5*airstats(3)*boatstats(7)*relativeairspeedy*relativeairspeedmagnitde/boatstats(9);

accelerationduetowaterdragx=.5*waterstats(3)*boatstats(8)*relativewaterspeedx...
                     *relativewaterspeedmagnitude/boatstats(9);
accelerationduetowaterdragy=.5*waterstats(3)*boatstats(8)*relativewaterspeedy...
                     *relativewaterspeedmagnitude/boatstats(9);

angletodestination=arctan(destination(1),destination(2));

%The double loop below allocates all of the possible results
%with changing the sail and keel position                 
for i=1:s
    for j=1:k
        
        %these two lines create an array for the sail and keel to grab the
        %correct value later on
        sailtestangles(i,j)=sailangles(i);
        keeltestangles(i,j)=keelangles(j);
        
        %The two lines below get the relative angle of the sail to the air
        %and the relative angle of the keel to the water
        sailforceangle=sailangles(i)-airstats(1);
        keelforceangle=keelangles(j)-waterstats(1);
        
      %The dot products between the velocity of the boat/sail with the air
        airdotsail=airstats(2)*boatstats(4)*cos(sailforceangle);
     %The dot products between the velocity of the boat/keel with the water
        waterdotkeel=waterstats(2)*boatstats(4)*cos(keelforceangle);
        
        %How much the air changes momentum
        momentumchangeduetosailx=-dt*airstats(3)*boatstats(5)*airdotsail*cos(sailforceangle);
        %How much the water changes momentum
        momentumchangeduetosaily=-dt*airstats(3)*boatstats(5)*airdotsail*sin(sailforceangle);
        
        momentumchangeduetokeelx=dt*waterstats(3)*boatstats(6)*waterdotkeel*cos(keelforceangle);
        momentumchangeduetokeely=dt*waterstats(3)*boatstats(6)*waterdotkeel*sin(keelforceangle);
        
        changeinboatvelocityx=(momentumchangeduetosailx+momentumchangeduetokeelx)/boatstats(9);
        changeinboatvelocityy=(momentumchangeduetosaily+momentumchangeduetokeely)/boatstats(9);
        
        boatvelocitiesx(i,j)=(accelerationduetoairdragx+accelerationduetowaterdragx)*dt+changeinboatvelocityx+boatstats(4)*cos(boatstats(3));
        %boatvelocitiesx(i,j)
        boatvelocitiesy(i,j)=(accelerationduetoairdragy+accelerationduetowaterdragy)*dt+changeinboatvelocityy+boatstats(4)*sin(boatstats(3));
        %boatvelocitiesy(i,j)
        
        velocityangle(i,j)=arctan(boatvelocitiesx(i,j),boatvelocitiesy(i,j));
        
        velocitytopointfactor(i,j)=cos(velocityangle(i,j)-angletodestination);
        
        boatpositionsx(i,j)=boatstats(1)+ boatvelocitiesx(i,j)*dt;
        %boatpositionsx(i,j)
        boatpositionsy(i,j)=boatstats(2)+ boatvelocitiesy(i,j)*dt;
        %boatpositionsy(i,j)
        testdistances(i,j)=sqrt((boatpositionsx(i,j)-destination(1))^2+(boatpositionsy(i,j)-destination(2))^2);
    end
end



for ii=1:length(testdistances)
    testmetric(ii)=velocitytopointfactor(ii)/testdistances(ii);
end

%these varables add a lay anchor option to the test, the if statement
%handles if that happens
testmetric(s*k+1)=0;

I=find(testmetric==max(testmetric));

if (length(I)>1)
    I=I(1+floor(length(I)*rand));
    
end

if (testmetric(I)==0)

    sailangle=0;
    keelangle=0;
    newboatvelocityx=boatstats(4)*cos(boatstats(3))+(accelerationduetoairdragx+accelerationduetowaterdragx)*dt;
    newboatvelocityy=boatstats(4)*sin(boatstats(3))+(accelerationduetoairdragy+accelerationduetowaterdragy)*dt;
    newboatposx=boatstats(1)+newboatvelocityx*dt;
    newboatposy=boatstats(2)+newboatvelocityy*dt;

else

    sailangle=sailtestangles(I);
    keelangle=keeltestangles(I);
    newboatposx=boatpositionsx(I);
    newboatposy=boatpositionsy(I);
    newboatvelocityx=boatvelocitiesx(I);
    newboatvelocityy=boatvelocitiesy(I);

        
end
end

