/////////////////////////////////////////////////////////////////
//                           Settings                          //
/////////////////////////////////////////////////////////////////

#define BUILD_NUMBER          01                // Build number is something like change some variables in code and building binary file. 

/***************************************************
          NodeMCU Pin Assignment
 **************************************************/
// пины на которые садим esp или что то подобное
#define RX_PIN_NODEMCU        14                  // D5 or GPIO 14 || RX or GPIO 03 -->  Nodemcu pin, used has RX pin
#define TX_PIN_NODEMCU        12                  // D6 or GPIO 12 || TX or GPIO 01 -->  Nodemcu pin, used has TX pin


//статические адреса для назначение пземкам - подключить каждую по отдельности и раскоментироват ь в коде строчку изменив адресс  - changeAddress(0XF8, 0x01); 
#define PZEM_SLAVE_1_ADDRESS     0x01                       // Make sure you assign address to pzem first before you use
#define PZEM_SLAVE_2_ADDRESS     0xF7
#define PZEM_SLAVE_3_ADDRESS     0x02

/***************************************************
How often you would like to call function? Enter time in milliseconds  
 **************************************************/

// время для определения длительности выполнения setInterval
#define GET_PZEM_DATA_TIME          20000                        // How often you wish to get PZEM data. Enter time in milliseconds 


#define OTA_HOSTNAME "3 Phase Failure Automation"            // OTA name
