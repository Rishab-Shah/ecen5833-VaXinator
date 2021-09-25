Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

*Be sure to take measurements with logging disabled to ensure your logging logic is not impacting current/time measurements.*

*Please include screenshots of the profiler window detailing each current measurement captured.  See the file Instructions to add screenshots in assignment.docx in the ECEN 5823 Student Public Folder.* 

1. What is the average current per period?
   Answer: 9.90 uA
   <br>Screenshot:  
   ![Avg_current_per_period](https://github.com/CU-ECEN-5823/ecen5823-assignment4-vido2373/blob/master/questions/Assignment4-Screenshots/avg_current_per_period.JPG)  

2. What is the average current when the Si7021 is Powered Off?
   Answer: 1.61 uA
   <br>Screenshot:  
   ![Avg_current_LPM_Off](https://github.com/CU-ECEN-5823/ecen5823-assignment4-vido2373/blob/master/questions/Assignment4-Screenshots/avg_current_lpm_off.JPG)  

3. What is the average current when the Si7021 is Powered On?
   Answer: 199.58 uA
   <br>Screenshot:  
   ![Avg_current_LPM_Off](https://github.com/CU-ECEN-5823/ecen5823-assignment4-vido2373/blob/master/questions/Assignment4-Screenshots/avg_current_lpm_on.JPG)  

4. How long is the Si7021 Powered On for 1 temperature reading?
   Answer: 97.5 ms
   <br>Screenshot:  
   ![duration_lpm_on](https://github.com/CU-ECEN-5823/ecen5823-assignment4-vido2373/blob/master/questions/Assignment4-Screenshots/avg_current_lpm_on.JPG)  

5. Compute what the total operating time of your design for assignment 4 would be in hours, assuming a 1000mAh battery power supply?
   Answer: 1000 mAh / (9.90 uA/3 sec * 60 sec/min * 60 min/hr) = 84.175 hours
   
6. How has the power consumption performance of your design changed since the previous assignment?
   Answer: With interrupt-based I2C transactions, the average current for the transaction is much lower than in the previous assignment, and thus has lower power consumption.
   
7. Describe how you have tested your code to ensure you are sleeping in EM1 mode during I2C transfers.
   Answer: I would look at the screenshots from Assignment 2 and check the average current for when the Gecko is sleeping in EM1 there, and compared the currents for when I know the Gecko to be sleeping in this assignment. The current measured for sleeping in EM1 in Assignment 2 was around 3.5 mA, and from the screenshot below, we can see that the current at the point where we are sleeping in EM1 is 3.87 mA, which is very close.
    <br>Screenshot:
   ![EM1_Sleep_Current](https://github.com/CU-ECEN-5823/ecen5823-assignment4-vido2373/blob/master/questions/Assignment4-Screenshots/EM1_sleep_current.JPG)
   

