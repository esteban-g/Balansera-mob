
xdatai1= indiv1(:,1); % X AXIS
ydatai1= indiv1(:,2); % Y AXIS
zdatai1= indiv1(:,3); % Z AXIS
tdatai1=indiv1(:,4); % TIME


figure(1);
plot(tdatai1,zdatai1,'b');
title('Individual 20. Chair standing test. Acceleration(Z comp) vs. time ');
xlabel('Time (seconds)');
ylabel('Acceleration (m/seg^2)');
grid on;

%PEAKFINDER PARAMETERS
sel = (max(zdatai1)-min(zdatai1))/4;
mins = min(zdatai1);
maxs = max(zdatai1);
thresh= 0.8*sel;

%SET OF MAXIMAL PEAKS
maximal_peak = peakfinder(zdatai1);

%SET OF MINIMAL PEAKS
minimal_peak = peakfinder(zdatai1,sel,thresh,-1);

%NUMBER OF MAXIMAL PEAKS
sizemaximal_peak = size(maximal_peak);

%NUMBER OF MINIMAL PEAKS
sizeminimal_peak = size(minimal_peak);

minmaximal_peak = 0:sizemaximal_peak/2;



%STARTING POINT

if sizemaximal_peak >0    
    if ~isempty(maximal_peak) 
%        disp('The set of maximal points is not empty')
       

%        peakfinder(zdatai1);
        [peakMaxLoc, peakMaxMag] = peakfinder(zdatai1);
%        peakfinder(zdatai1,sel,thresh,-1);
        [peakMinLoc, peakMinMag] = peakfinder(zdatai1,1,10,-1);

        

%        disp('** 1st PEAK TIME ***')
        peakMaxStart = tdatai1(peakMaxLoc(1));
        %peakMaxStart %TIME PEAK MAX
        magmax = peakMaxMag(1); %MAG PEAK MAX
        
        peakMinStart = tdatai1(peakMinLoc(1));
        %peakMinStart %TIME PEAK MIN
        maginpoint= peakMinMag(1); %MAG PEAK MIN

        
        %Check the minimal before the 1st maximal 
        for v = peakMinLoc(1):1:peakMaxLoc(1) 
           if zdatai1(v)> 0.1*magmax
               % 1 SECOND BEFORE TO THE 10% MAX IS REACHED 
               % START TIME
               StartTime =tdatai1(v)-1               
           end
        end
            
            
        
    
        
        
        %ENDING POINT
        
        %THE NINTH PEAK IS USUALLY THE LAST PEAK OF THE 
        %SIT-2-STAND EXERCISE
        if ~isempty(maximal_peak(9)) 
           EndTime = tdatai1(maximal_peak(9))
        else
           disp('The plot has a unusual form')
        end
    end   
else 
    disp('There is no maximal peaks')
end




        %MIDDLE POINTS BETWEEN MAX PEAKS
        [maxima_peak_a, maxima_peak_b] = peakfinder(zdatai1,sel,thresh,1);
        peakfinder(zdatai1,sel,thresh,1);
        grid on;
        %peakfinder(tdatai1)
        
        %disp(maxima_peak);
        
        %xmaxima_peak = indiv0(:,3)


        x = ones(1,10);
        sizemaxima_peak = size(maxima_peak_a);
        minmaxima_peak = 0:sizemaxima_peak/2;
        vector_rise_end = ones(0,(sizemaxima_peak(1)));
        vector_sit_start = ones(0,(sizemaxima_peak(1)));
        
        for n = 1:sizemaxima_peak/2
                    interm = zdatai1(maxima_peak_a(2*n-1):maxima_peak_a(2*n));
            %disp(zdatai1(maxima_peak(2*n-1):maxima_peak(2*n)));
            % zdatai1 = indiv1(:,3)
%            disp('------------------------------')
%            disp(interm)
%            disp('------------------------------')
           % interm_min_peak = peakfinder(interm,sel,thresh,-1)%min peaks in the middle of max peaks
            %peakfinder(zdatai1(maxima_peak(2*n-1):maxima_peak(2*n)),sel,thresh,-1)

            
            sel_interm =(max(interm)-min(interm))/4;
            thresh_interm= 0.8*sel;
            
            
            
            
            format long
            [maxima_peak_x, maxima_peak_y] = peakfinder(interm,sel_interm,10,-1);
            

%                    disp('_X________www___________________')
%                    maxima_peak_x

                %tdatai1
%                    disp('_Y________www___________________')
%                    maxima_peak_y
                size_interm_y=size(maxima_peak_y);
%                 disp('__________________________')

                %maxima_peak_xss = vpa(maxima_peak_y, 15)

                indx_rise_end = find(indiv1(:,3)==maxima_peak_y(1));
                time_rise_end = tdatai1(indx_rise_end);

                indx_sit_start = find(indiv1(:,3)==maxima_peak_y(size_interm_y(1)));
                time_sit_start = tdatai1(indx_sit_start);

                vector_rise_end(end+1)=time_rise_end;
                vector_sit_start(end+1)= time_sit_start;
%                 disp('____________________________')

                %maxima_peak_b
                %peakMinMag



%            peakfinder(interm,-6,6,-1)
%            grid on;
            %middle_min =peakfinder(interm,sel_interm,thresh_interm,-1)
            middle_min =peakfinder(interm,-6,6,-1);
            size_mid = size(middle_min);
%            disp('_2________www___________________')
%            disp(middle_min(1));
%            disp(middle_min(size_mid));
%            disp('_3________www___________________')
            
            
            
            %TODO: YA SACA LOS PUNTOS PARA HACER EL DELTA RISE - DELTA SIT
            % LA CUESTION ES PASAR DICHOS PUNTOS A TIEMPO, LUEGO HACER LA
            % RESTA PARA LOS DELTA Y EL IF ELSE < > DELTA PARA SACAR EL 
            
            
            

            
            minmaxima_peak(n) =min(zdatai1(maxima_peak_a(2*n-1):maxima_peak_a(2*n)));
            
            %disp(min(zdatai1((2*n-1):(2*n))))
            %disp(zdatai1(n,:))
            %disp(zdatai1(:))
            %x(n) = 2 * x(n - 1);
        end
            
%            disp('_4________www___________________')

%        disp(minmaxima_peak)        
%            disp('_5________www___________________')





% 
% 
% disp(minmaximal_peak)
%     disp('__1__')
% for n = 1:sizemaximal_peak/2
%     %falta el if si es par o impar
%     disp(min(zdatai1(maximal_peak(2*n-1):maximal_peak(2*n))))
%     minmaximal_peak(n) =min(zdatai1(maximal_peak(2*n-1):maximal_peak(2*n)));
% 
%     %disp(min(zdatai1((2*n-1):(2*n))))
%     %disp(zdatai1(n,:))
%     %disp(zdatai1(:))
%     %x(n) = 2 * x(n - 1);
% end
%     disp('__0__')



%peakfinder(zdatai1,-10,10,-1);
peakfinder(zdatai1,3,5,-1)
 grid on;

[index_min, total_min_peak] =peakfinder(zdatai1,sel,thresh,-1);
 size_min = size(total_min_peak)
 disp('=====================')
 total_min_peak
% 
time_min_peaks=ones(0,size_min(1));

for m = 1:size_min(1)

%      disp('**')
      indx_min_peaks = find(indiv1(:,3)==total_min_peak(m));
      
      time_min_peaks(end+1) = tdatai1(indx_min_peaks(1));
 end 
 
%  disp('=====================')
%  disp(vector_rise_end)
%  
%   disp('=====================')
% disp(vector_sit_start)
% 
% disp('================')
%  disp(time_min_peaks)
%  
% disp('=============xx===')
time_min_peaks(1)=StartTime;

% disp('==============yy==')
%  disp(time_min_peaks)
% disp('==============zz==')

size_vec_rise = size(vector_rise_end);
size_vec_sit = size(vector_sit_start);
size_vec_timemins = size(time_min_peaks);

 delta_rise = ones(0,(size_vec_rise(1)));
 delta_sit = ones(0,(size_vec_sit(1)));

 disp('*deltas*')
  for p = 1:4

        delta_rise(end+1)= vector_rise_end(p) - time_min_peaks(p);
            
  end 
 
 for q = 1:4

        delta_sit(end+1)= time_min_peaks(q+1) - vector_sit_start(q);
            
 end 
  
%  disp('******Delta-rise vector*********')
  disp(delta_rise)
%  
%  disp('******Delta-sit vector*********')
  disp(delta_sit)
%  
%  disp('===================\n')
  delta_difference = ones(0,4);

  for r = 1:4

        delta_difference(end+1)= delta_rise(r) - delta_sit(r);
            
         if delta_rise(r) - delta_sit(r)> 0
            disp('profile: rise slow - sit fast ')
        elseif delta_rise(r) - delta_sit(r)< 0
            disp('profile: rise fast - sit slow ')
        elseif delta_rise(r) - delta_sit(r)== 0
            disp('profile: same rise - same sit')
        end
        
  end 
  disp('===================\n')
 