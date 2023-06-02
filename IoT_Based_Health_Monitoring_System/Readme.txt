Project Description : 

This project monitors the temperature and oxygen level of human body and also the environment temperature in which person is there.

There is button i.e the panic or Continuous monitor button which is used to measure the temperature and oxygen level continuously
and if the value exceed the predefined limit then buzzer would be ON.


How to use this project : 




User Functionalities implemented :





Technical Functionalities implemented :
1. One LED for indicating error in code or not connecting components needed in system (Red indication), indication if timeout occurs
   (blue color indication), and green indication if all components are connected and no timeout occurs.
2. One Buzzer which beep when the temperature, oxygen level and  hear rate cross predefined set limit. This buzzer also beep for one 
   time when only measure the temperature, oxygen level and heart rate.
3. One panic button which is used for both continuous measure of temperature, oxygen level and heart rate and also stop the 
   buzzer when the limit is crossed for above.
4. Two LED which generate the breathing effect once either or both the oxygen level, heart rate and the temperature sensor gets connected.
5. System which can detect whether actually the human body is placed to sensor or not.
6. Ultrasonic sensor which only show the reading on LCD once we place any hand or any object nearby.
7. Also shows the reading on Mobile based on App.

 







Devices/Microcontroller Used :




Toolchain/Softwares Used :





Steps Followed :
1. Understand the I2C Master send data api (without interrupt).
2. Making the accurate delay of 1 microseconds and 1 milliseconds (blocking mode).
3. Writing the library for I2C HD44780 lcd.
4. Adding the folder for buzzer and error led(when connected devices needed in system is missing or error occur in code) and 
   buzzer for indication when the value of temperature and oxygen level exceed the predefined limit.
5. Adding the DHT11 Library.



Problems faced:
1. Delay is working only when the clock is selected as PLL_P and not for internal RC oscillator i.e 16MHz.
2. In I2C HD44780 lcd library sometimes it works and sometimes it doesn't i.e it sometimes shows the character and sometimes it
   doesn't show any of character.
3. DHT11 library isn't working .





