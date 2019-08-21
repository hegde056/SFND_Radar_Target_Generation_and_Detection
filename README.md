# SFND : Radar Target Generation and Detection
Radar Target Generation and Detection implemented using Matlab as part of project submission for Sensor Fusion Nanodegree (Udacity).

####  Project Layout
<img src="https://github.com/hegde056/SFND_Radar_Target_Generation_and_Detection/blob/master/media/project_Layout.png" width="700" height="400" />


####  Radar Specifications
- Frequency of operation = 77GHz
- Max Range = 200m
- Range Resolution = 1 m
- Max Velocity = 100 m/s

-------------
- ####  User Defined Range and Velocity of target
  ```
  Target_initPos = 100;
  Target_initVelocity = -20;
  ```
- ####  FMCW Waveform Generation
	Designing FMCW waveform by considering the Radar specifications. Calculations for its parameters namely  Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) 
    ```
    B = c/(2*Radar_rangeResolution);  			%Bandwidth (B) , also Bsweep
	Tchirp = 5.5 * 2 * Radar_maxRange/c;  		%Chirp Time (Tchirp)
	slope = B/Tchirp; 							%Slope (slope)
    ```
    
- ####  Signal generation and Moving Target simulation
	Simulating the signal propagation and moving target scenario. 
    Both the FMCW transmit and receive are modeled and these two signals are mixed to get beat signal which holds the values for both range as well as doppler.
    ```
  for i=1:length(t)         

      %For each time stamp update the Range of the Target for constant velocity. 
      r_t(i) = Target_initPos + Target_initVelocity * t(i) ;
      td(i) = 2 * r_t(i)/c ;

      %For each time sample we need update the transmitted and
      %received signal. 
      Tx(i) = cos(2*pi*(fc*t(i) + 0.5*slope*t(i)^2));
      Rx(i) = cos(2*pi*((fc*(t(i) - td(i)) + 0.5*slope*(t(i) - td(i))^2)));

      %Now by mixing the Transmit and Receive generate the beat signal
      %This is done by element wise matrix multiplication of Transmit and
      %Receiver Signal
      Mix(i) = (Tx(i).*Rx(i));

  end
    ```
   
 - ####  Range Measurement (1st FFT)
 	- Reshaping the vector into Nr*Nd array.
 		```
        Mix=reshape(Mix,1,Nr*Nd);
        ```
    - Running the FFT on the beat signal along the range bins dimension (Nr) and normalize.
        ```
        signal_fft = fft(Mix,Nr)/Nr;
        ```
    - Taking the absolute value of FFT output
        ```
        signal_fft = abs(signal_fft);
        ```
     - Keeping one half of the signal
        ```
        signal_fft = signal_fft(1:Nr/2+1);
        ```   
     - Plot of output of 1st FFT for target 
   <img src="https://github.com/hegde056/SFND_Radar_Target_Generation_and_Detection/blob/master/media/FFT_1D.png" width="700" height="400" />
   
 - ####  Range Doppler Response (2nd FFT)
 	- Plot of output of 2nd FFT for target 
<img src="https://github.com/hegde056/SFND_Radar_Target_Generation_and_Detection/blob/master/media/FFT_2D.png" width="700" height="400" />

 - ####  CFAR implementation
 	Solutions to the false-alarm problem in radar involve implementation of constant false-alarm rate (CFAR) schemes that vary the detection threshold as a function of the sensed environment.The CFAR implementation assumes that the noise or interference is spatially or temporarily homogeneous.
    One of the CFAR scheme is Cell-Averaging constant false-alarm rate (CA-CFAR).
    
    - To determine the number of Training cells and guard cells for each dimension :
    
      The number of training cells is decided based on the environment. If a dense traffic scenario then the fewer training cells are used, as closely spaced targets can impact the noise estimate.
      
      The purpose of the Guard Cells is to avoid the target signal from leaking into the training cells that could adversely affect the noise estimate. The number of guard cells are decided based on the leakage of the target signal out of the cell under test. If target reflections are strong they often get into surrounding bins.
      ```
      Tr = 10 ;
      Td = 8 ;
      Gr = 4;
      Gd = 4;
      ```
      Offset value is used to scale the noise threshold.
      ```
      offset  = 8;
      ```
   		Total training cells : 
   		```
        training_cells = (2*Tr+2*Gr+1)*(2*Td+2*Gd+1) - (2*Gr+1)*(2*Gd+1); %total number of training cells
        ```
   - Slide the cell under test across the complete matrix :
      ```
      for i = Tr+Gr+1:Nr/2 - (Gr+Tr)
          for j= Td+Gd+1:Nd - (Gd + Td)
            %initialize noise_level for each iteration on training cells
      		noise_level = zeros(1,1);
          	....
          end
      end
      ```
   - For every iteration sum the signal level within all the training cells. To sum convert the value from logarithmic to linear using db2pow function : 
    	```      
         for p = i -(Tr+Gr) : i+Tr+Gr
              for q = j - (Td+Gd) : j+ Td+Gd
                  if(abs(i-p)>Gr || abs(j-q) > Gd)
                      noise_level = noise_level + db2pow(RDM(p,q));
                  end
              end
      	 end
      	```
  	- Average the summed values for all of the training cells used. After averaging convert it back to logarithmic using pow2db.
Further add the offset to it to determine the threshold : 
      ```
      threshold = pow2db(noise_level /training_cells) + offset;
      ```
	- Next, compare the signal under CUT against this threshold.
If the CUT level > threshold assign it a value of 1, else equate it to 0.

      ```
        CUT = RDM(i,j);
        if(CUT < threshold)
            RDM(i,j) = 0;
        else
            RDM(i,j) = 1;
        end
      ```
	- The above process  will generate a thresholded block, which is smaller 
than the Range Doppler Map as the CUT cannot be located at the edges of
matrix. Hence,few cells will not be thresholded. To keep the map size same
set those values to 0.
      ```
      for x=1:Nr/2
          for y = 1:Nd
              if(RDM(x,y)~=0 && RDM(x,y)~=1) 
                  RDM(x,y) = 0;   %equating all the non-thresholded cells to 0.
              end
          end
      end
      ```
    - Plot the CFAR output : 
    	```
        figure,surf(doppler_axis,range_axis,RDM);
		colorbar;
        ```
        <img src="https://github.com/hegde056/SFND_Radar_Target_Generation_and_Detection/blob/master/media/CFAR_2D.png" width="700" height="400" />

    
   

