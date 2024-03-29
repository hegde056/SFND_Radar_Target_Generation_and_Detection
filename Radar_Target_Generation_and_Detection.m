clear all
clc;

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%speed of light = 3e8

Radar_maxRange = 200;
Radar_rangeResolution = 1;
Target_maxVelocity = 100;
c = 3e8;

%% User Defined Range and Velocity of target
% *%TODO* : DONE
% define the target's initial position and velocity. Note : Velocity
% remains contant
Target_initPos = 100;
Target_initVelocity = -20;

%% FMCW Waveform Generation

% *%TODO* : DONE
%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.


%Operating carrier frequency of Radar 
fc= 77e9;             %carrier freq

B = c/(2*Radar_rangeResolution);  %Bandwidth (B) , also Bsweep

Tchirp = 5.5 * 2 * Radar_maxRange / c;  %Chirp Time (Tchirp)

slope = B/Tchirp; %Slope (slope)
                                                          
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd=128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr=1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t=linspace(0,Nd*Tchirp,Nr*Nd); %total time for samples


%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t=zeros(1,length(t));
td=zeros(1,length(t));


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    
    
    % *%TODO* : DONE
    %For each time stamp update the Range of the Target for constant velocity. 
    r_t(i) = Target_initPos + Target_initVelocity * t(i) ;
    
    td(i) = 2 * r_t(i)/c ;
    
    % *%TODO* : DONE
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i) = cos(2*pi*(fc*t(i) + 0.5*slope*t(i)^2));
    Rx(i) = cos(2*pi*((fc*(t(i) - td(i)) + 0.5*slope*(t(i) - td(i))^2)));
    
    % *%TODO* : DONE
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = (Tx(i).*Rx(i));
    
end

%% RANGE MEASUREMENT


 % *%TODO* : DONE
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
Mix=reshape(Mix,1,Nr*Nd);

 % *%TODO* : DONE
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
signal_fft = fft(Mix,Nr)/Nr;

 % *%TODO* : DONE
% Take the absolute value of FFT output
signal_fft = abs(signal_fft);

 % *%TODO* : DONE
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
signal_fft = signal_fft(1:Nr/2+1);


%plotting the range
figure ('Name','Range from First FFT')
%subplot(2,1,1)
 % *%TODO* : DONE
 % plot FFT output 
 plot(signal_fft)
axis ([0 200 0 1]);



%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM


% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure,surf(doppler_axis,range_axis,RDM);

%% CFAR implementation

%Slide Window through the complete Range Doppler Map

% *%TODO* :DONE
%Select the number of Training Cells in both the dimensions.
Tr = 10 ;
Td = 8 ;

% *%TODO* :DONE
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
Gr = 4;
Gd = 4;

% *%TODO* :DONE
% offset the threshold by SNR value in dB
offset  = 8;

% *%TODO* : DONE
%Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(1,1);
training_cells = (2*Tr+2*Gr+1)*(2*Td+2*Gd+1) - (2*Gr+1)*(2*Gd+1); %total number of training cells

% *%TODO* : DONE

   % Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
   % CFAR
  
%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
for i = Tr+Gr+1:Nr/2 - (Gr+Tr)
    for j= Td+Gd+1:Nd - (Gd + Td)
      
      %initialize noise_level for each iteration on training cells
      noise_level = zeros(1,1);
      
      %For every iteration sum the signal level within all the training
      %cells.      
      for p = i -(Tr+Gr) : i+Tr+Gr
          for q = j - (Td+Gd) : j+ Td+Gd
              if(abs(i-p)>Gr || abs(j-q) > Gd)
                % To sum convert the value from logarithmic to linear using db2pow
                %function
                  noise_level = noise_level + db2pow(RDM(p,q));
              end
          end
      end
     %Average the summed values for all of the training
     %cells used. After averaging convert it back to logarithimic using pow2db.
     %Further add the offset to it to determine the threshold
      threshold = pow2db(noise_level /training_cells) + offset;
      
      %Next,compare the
      %signal under CUT with this threshold. If the CUT level > threshold assign
      %it a value of 1, else equate it to 0.
      CUT = RDM(i,j);
      
      if(CUT < threshold)
          RDM(i,j) = 0;
      else
          RDM(i,j) = 1;
      end
    end
 
end



% *%TODO* : DONE
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 
 

for x=1:Nr/2
    for y = 1:Nd
        if(RDM(x,y)~=0 && RDM(x,y)~=1) 
            RDM(x,y) = 0;   %equating all the non-thresholded cells to 0.
        end
    end
end



% *%TODO* : DONE
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
figure,surf(doppler_axis,range_axis,RDM);
colorbar;


 
 