Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

**1. How much current does the system draw (instantaneous measurement) when a single LED is on with the GPIO pin set to StrongAlternateStrong?**
   Answer: The system draws about 5 mA of current. 


**2. How much current does the system draw (instantaneous measurement) when a single LED is on with the GPIO pin set to WeakAlternateWeak?**
   Answer: The system draws about 5.2 mA of current.


**3. Is there a meaningful difference in current between the answers for question 1 and 2? Please explain your answer, 
referencing the [Mainboard Schematic](https://www.silabs.com/documents/public/schematic-files/WSTK-Main-BRD4001A-A01-schematic.pdf) and [AEM Accuracy](https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf) section of the user's guide where appropriate. Extra credit is avilable for this question and depends on your answer.**
   Answer: There isn't a meaningful difference. Since the LED has a 1.8V voltage drop and the GPIO pin supplies 3.3V, the voltage drop across the resistor is 1.5V. Since the value of the resistor is 3K ohms, using Ohm's Law, V=IR, the current across the resistor and through the LED circuit is 0.5 mA. WeakAlternateWeak can drive 1 mA and StrongAlternateStrong can drive 10 mA, so both of these settings can supply the LED circuit with sufficient current, meaning there is no difference between the two settings for this circuit.


**4. With the WeakAlternateWeak drive strength setting, what is the average current for 1 complete on-off cycle for 1 LED with an on-off duty cycle of 50% (approximately 1 sec on, 1 sec off)?**
   Answer: The average current for the system is 4.82 mA.


**5. With the WeakAlternateWeak drive strength setting, what is the average current for 1 complete on-off cycle for 2 LEDs (both on at the time same and both off at the same time) with an on-off duty cycle of 50% (approximately 1 sec on, 1 sec off)?**
   Answer: The average current for the system is 4.91 mA.


