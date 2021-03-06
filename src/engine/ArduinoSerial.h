//
//  BYBArduino.h
//  SpikeRecorder
//
//  Created by Stanislav Mircic on 11/26/14.
//  Copyright (c) 2014 UNIT. All rights reserved.
//


#ifndef BACKYARDBRAINS_ARDUINOSERIAL_H
#define BACKYARDBRAINS_ARDUINOSERIAL_H

#define USB_SERIAL_PORT "/dev/tty.usbmodemfa131" // (OSX Mac Book Air) Right
#define SIZE_OF_CIRC_BUFFER 1024
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <list>
#include <string>


//just MAC classes
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/IOBSD.h>



namespace BackyardBrains {
    
    class ArduinoSerial {
    public:
        int openPort(const char *portName);
        int readPort(int16_t *);
        std::list<std::string> list;
        void getAllPortsList();
        void closeSerial(void);
    private:
        char circularBuffer[SIZE_OF_CIRC_BUFFER];
        int cBufHead;
        int cBufTail;
        int fd;//file descriptor
        
        int serialCounter;
        
        bool returnTailForOneAndCheck();
        bool checkIfNextByteExist();
        void macos_ports(io_iterator_t  * PortIterator);
    };
    
}

#endif /* defined(__SpikeRecorder__BYBArduino__) */
