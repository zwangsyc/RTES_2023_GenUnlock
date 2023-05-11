# RTES_2023_GenUnlock
Todo list:
1. moving average filter and Guassian filter 
2. calibrating function to set the holding position 
3. led blinking for the recording and attempting 
4. screen style design
5. might try to use the thread 
6. might try to save the record and attempt sequence to the SDRAM
7. double checking the gyroscope mannual to find the best sample rating, cut off or other infos
8. ...
10. write a report
11. thread
     main ------------------------------------
              |---------------------gyroscope----
              |---------------------record/attempt-------------


Tried:  
update the UI
* after insert GUI driver, it has conflict with the original STMxxx driver, seems like they are not compatiable 
* idea come from https://www.instructables.com/STM32F4-Discovery-part1-Touch-Buttons/
* GUI driver: https://github.com/gopal-amlekar/stm32f4-arm-blink.  https://github.com/Infineon/emwin

Done:
* led blink during recoridng and attempting 
* Gaussian filter  https://stackoverflow.com/questions/54614167/trying-to-implement-gaussian-filter-in-c
