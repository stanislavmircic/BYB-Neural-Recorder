//
//  ArduinoSerial.cpp
//  SpikeRecorder
//
//  Created by Stanislav Mircic on 11/26/14.
//  Copyright (c) 2014 UNIT. All rights reserved.
//

#include "ArduinoSerial.h"
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>


//added for port scan
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <paths.h>
#include <termios.h>
#include <sysexits.h>
#include <sys/param.h>
#include <sys/select.h>
#include <sys/time.h>
#include <time.h>




namespace BackyardBrains {
    
    
    
    int ArduinoSerial::openPort(const char *portName)
    {
        
       
        
       
        closeSerial();
        fd = 0;
        struct termios options;
        
        fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY | O_SHLOCK);
        sleep(2);
        int bits;
#ifdef __APPLE__
        if (fd < 0) {
            std::cout<<"Unable to open "<<portName<<", "<<strerror(errno)<<"\n";
            return -1;
        }
        if (ioctl(fd, TIOCEXCL) == -1) {
            close(fd);
            std::cout<<"Unable to get exclussive access to port "<<portName<<"\n";
            return -1;
        }
        if (ioctl(fd, TIOCMGET, &bits) < 0) {
            close(fd);
            std::cout<<"Unable to query serial port signals on "<<portName<<"\n";
            return -1;
        }
        bits &= ~(TIOCM_DTR | TIOCM_RTS);
        if (ioctl(fd, TIOCMSET, &bits) < 0) {
            close(fd);
            std::cout<<"Unable to control serial port signals on "<<portName<<"\n";
            return -1;
        }
        struct termios settings_orig;
        if (tcgetattr(fd, &settings_orig) < 0) {
            close(fd);
            std::cout<<"Unable to access baud rate on port "<<portName<<"\n";
            return -1;
        }
#endif
#ifdef __linux__
        
#endif
        if (fd == -1)
        {
            std::cout<<"Can't open serial port\n";
            return -1;
        }
        fcntl(fd, F_SETFL, 0);    // clear all flags on descriptor, enable direct I/O
        tcgetattr(fd, &options);   // read serial port options
        // enable receiver, set 8 bit data, ignore control lines
        options.c_cflag |= (CLOCAL | CREAD | CS8);
        // disable parity generation and 2 stop bits
        options.c_cflag &= ~(PARENB | CSTOPB);

        //cfsetispeed(&options, B9600);
        //cfsetospeed(&options, B9600);
        
        cfsetispeed(&options, B230400);
        cfsetospeed(&options, B230400);

        // set the new port options
        tcsetattr(fd, TCSANOW, &options);
        circularBuffer[0] = '\n';
        cBufHead = 0;
        cBufTail = 0;
        serialCounter = 0;
        return fd;
    }
    
  
    int ArduinoSerial::readPort(int16_t * obuffer)
    {
        
        
        // Initialize file descriptor sets
        fd_set read_fds, write_fds, except_fds;
        FD_ZERO(&read_fds);
        FD_ZERO(&write_fds);
        FD_ZERO(&except_fds);
        FD_SET(fd, &read_fds);
        // Set timeout to 300 ms
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 6000;
        
        char buffer[1024];

        int writeInteger = 0;
        ssize_t size = -1;
        int obufferIndex = 0;
        
        /*if(serialCounter != 1)
        {
            serialCounter++;
            return -1;
        }
        serialCounter = 0;*/
        
        if (select(fd + 1, &read_fds, &write_fds, &except_fds, &timeout) == 1)
        {
            size = read(fd, buffer, 1000);
        }
        
       // std::cout<<"Size: "<<size<<"\n";
        for(int i=0;i<size;i++)
        {
            circularBuffer[cBufHead++] = buffer[i];
           // uint MSB  = ((uint)(buffer[i])) & 0xFF;
           // std::cout<<"M: " << MSB<<"\n";
            if(cBufHead>=SIZE_OF_CIRC_BUFFER)
            {
                cBufHead = 0;
            }
            
        }
        bool insideLimits = true;
        if(size==-1)
        {
            return -1;
        }
        uint LSB;
        uint MSB;


            while (1)
            {

                MSB  = ((uint)(circularBuffer[cBufTail])) & 0xFF;
                if(MSB > 127)
                {
                    if(checkIfNextByteExist())
                    {
                        MSB  = ((uint)(circularBuffer[cBufTail])) & 0x7F;
                        cBufTail++;
                        if(cBufTail>=SIZE_OF_CIRC_BUFFER)
                        {
                            cBufTail = 0;
                        }
                        LSB  = ((uint)(circularBuffer[cBufTail])) & 0x7F;
                        MSB = MSB<<7;
                        writeInteger = LSB | MSB;
                    }
                    else
                    {
                        break;
                    }
                }
                cBufTail++;
                if(cBufTail>=SIZE_OF_CIRC_BUFFER)
                {
                    cBufTail = 0;
                }
                if(cBufTail==cBufHead)
                {
                    break;
                }
                if(insideLimits)
                {
                    obuffer[obufferIndex++] = writeInteger;
                }
            }
            
        
        return obufferIndex;
    }
    
    bool ArduinoSerial::checkIfNextByteExist()
    {
        int tempTail = cBufTail + 1;
        if(tempTail>= SIZE_OF_CIRC_BUFFER)
        {
            tempTail = 0;
        }
        if(tempTail==cBufHead)
        {
            return false;
        }
        return true;
    }
    
    
    
    
    
  
    
    void ArduinoSerial::macos_ports(io_iterator_t  * PortIterator)
    {
        io_object_t modemService;
        CFTypeRef nameCFstring;
        char s[MAXPATHLEN];
        
        while ((modemService = IOIteratorNext(*PortIterator))) {
            nameCFstring = IORegistryEntryCreateCFProperty(modemService,
                                                           CFSTR(kIOCalloutDeviceKey), kCFAllocatorDefault, 0);
            if (nameCFstring) {
                if (CFStringGetCString((const __CFString *)nameCFstring,
                                       s, sizeof(s), kCFStringEncodingASCII)) {
                    list.push_back(s);
                }
                CFRelease(nameCFstring);
            }
            IOObjectRelease(modemService);
        }
    }
   
    
    
    
   // Return a list of all serial ports
    void ArduinoSerial::getAllPortsList()
    {
        list.clear();

        // adapted from SerialPortSample.c, by Apple
        // http://developer.apple.com/samplecode/SerialPortSample/listing2.html
        // and also testserial.c, by Keyspan
        // http://www.keyspan.com/downloads-files/developer/macosx/KesypanTestSerial.c
        // www.rxtx.org, src/SerialImp.c seems to be based on Keyspan's testserial.c
        // neither keyspan nor rxtx properly release memory allocated.
        // more documentation at:
        // http://developer.apple.com/documentation/DeviceDrivers/Conceptual/WorkingWSerial/WWSerial_SerialDevs/chapter_2_section_6.html
        mach_port_t masterPort;
        CFMutableDictionaryRef classesToMatch;
        io_iterator_t serialPortIterator;
        if (IOMasterPort(NULL, &masterPort) != KERN_SUCCESS) return;
        // a usb-serial adaptor is usually considered a "modem",
        // especially when it implements the CDC class spec
        classesToMatch = IOServiceMatching(kIOSerialBSDServiceValue);
        if (!classesToMatch) return;
        CFDictionarySetValue(classesToMatch, CFSTR(kIOSerialBSDTypeKey),
                             CFSTR(kIOSerialBSDModemType));
        if (IOServiceGetMatchingServices(masterPort, classesToMatch,
                                         &serialPortIterator) != KERN_SUCCESS) return;
        macos_ports(&serialPortIterator);
        IOObjectRelease(serialPortIterator);
        // but it might be considered a "rs232 port", so repeat this
        // search for rs232 ports
        classesToMatch = IOServiceMatching(kIOSerialBSDServiceValue);
        if (!classesToMatch) return;
        CFDictionarySetValue(classesToMatch, CFSTR(kIOSerialBSDTypeKey),
                             CFSTR(kIOSerialBSDRS232Type));
        if (IOServiceGetMatchingServices(masterPort, classesToMatch,
                                         &serialPortIterator) != KERN_SUCCESS) return;
        macos_ports(&serialPortIterator);
        IOObjectRelease(serialPortIterator);

        list.sort();
        return;
    }

    
    
    
    
    
    
    
    // Close the port
    void ArduinoSerial::closeSerial(void)
    {
       // if (!port_is_open) return;
        //Output_flush();
        //Input_discard();
        //port_is_open = 0;
        //port_name = "";

        //#if defined(LINUX) || defined(MACOSX)
        //tcsetattr(fd, TCSANOW, &settings_orig);
        close(fd);
        /*#elif defined(WINDOWS)
        //SetCommConfig(port_handle, &port_cfg_orig, sizeof(COMMCONFIG));
        CloseHandle(port_handle);
        #endif*/
    }
    
    
    
    
    
}
