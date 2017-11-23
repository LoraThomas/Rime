2017-06-21
1) Smart adapt LoRa Node(TDMA) and LoRaWAN Node(FDMA);
2) Support 2 Node's low power that sleep current is 1.6uA;
3) Config GPIO_1/2 as:
   LoRa Node(TDMA)   : GPIO_1=Input, GPIO_2=Output; 
   LoRaWAN Node(FDMA): GPIO_1=Output, GPIO_2=Input;
4) Sample and Send temperature and humidity when KEY been pressed.

2016-10-03
1) Add "AtomicSetBit()" and "AtomicTestClearBit()" to avoid race condition;
2) Configu GPIO to low power mode;
3) Set MCU to "WFI(Wait For Interrupt)" when it is idle.

2016-05-20
1) Create the project;
2) Sample and Send temperature and humidity per 1 second. 