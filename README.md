# RTES_2023_GenUnlock
Team 40 work presentation is here:

- Seperate the LCD Thread from the Main Thread to show the real-time information
1) Implementation of the Record Button and Attempt Button on the display screen 
2) Implementation of generating the log information for each running
   The log will show: Recorded, Attempting, Starting Comparing, Compare Done, and Unlock Success/Unlock Failed
3) Implementation of displaying the Team number and Group Title
4) Implementation of attempting times(here we only allow user to record one time; but attempt for 4 times)

   it will show the attempting times you already tried;
   
   ---------if you already reach the maximum attempt time
   
   ----------------Once you reached, it will give user different response
   
   ---------------------if it is successfully unlocked, it will show "Finally it works"
   
   ---------------------if it is not, it will show "Try again later"
5) Implementation of avoiding to repetitively press the record button (since it designed to record the sequence for one time)
   
   For the attempt button, if you do not reach the maximum attempt times, you can press it after this comparing iteration finished
   
   In order to show the next iteration log information, we also allow user to use USER-BUTTON to clear the log information if he wants
 
 - Design the screen detection function  to detect if the specific area of the screen is pressed or not.
 1) For the record button area, once it is pressed, you could not press it again
 2) For the attempt button area, it could be pressed if your current attempting is not correct
 
 - Design the SPI communication with the specific gryoscope control register content
 1) Here we set the 245 dps, and then we process the data with its defined sensitivity value to convert from degrees to radians
 2) Previously, we also try 500dps, but the result is not too sensitive
 3) keep same like the SPI demo provided by the lecture
 
 - Design the USER-BUTTON to clear the log information 
 - Compare several filter to find out our best solution
 5) Average Moving filter: windows size 5 (it could work better for our design)
 6) Gaussian filter: Kernel size 5 and sigma 1.0 (it could not detect too sensitive gesture)
    Before the designed filter, we also do the Normalization, Calibration to set the original datapoint(0,0,0) and remove some noise
 
Above this design, our embedded system could work correctly.

- the video link is here:
https://youtu.be/D185W0Kz8zg

- the team member
Zeng Wang(zw3464), Yuxi Liu(yl10673), Yujing Zhang(yz8887)
