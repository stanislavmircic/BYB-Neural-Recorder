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
#include <sstream>

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
        
       
        _portName = std::string(portName);
        _portOpened = false;
        closeSerial();
        fd = 0;
        _numberOfChannels = 1;
        struct termios options;
        
        fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY | O_SHLOCK);
        sleep(2);
        int bits;
#ifdef __APPLE__
        std::stringstream sstm;
        
        if (fd < 0) {
            sstm << "Unable to open " << portName << ", " << strerror(errno);
            errorString = sstm.str();
            std::cout<<"Unable to open "<<portName<<", "<<strerror(errno)<<"\n";
            return -1;
        }
        if (ioctl(fd, TIOCEXCL) == -1) {
            close(fd);
            sstm << "Unable to get exclussive access to port " << portName;;
            errorString = sstm.str();
            std::cout<<"Unable to get exclussive access to port "<<portName<<"\n";
            return -1;
        }
        if (ioctl(fd, TIOCMGET, &bits) < 0) {
            close(fd);
            sstm <<"Unable to query serial port signals on " << portName;
            errorString = sstm.str();
            std::cout<<"Unable to query serial port signals on "<<portName<<"\n";
            return -1;
        }
        bits &= ~(TIOCM_DTR | TIOCM_RTS);
        if (ioctl(fd, TIOCMSET, &bits) < 0) {
            close(fd);
            sstm <<"Unable to control serial port signals on " << portName;
            errorString = sstm.str();
            std::cout<<"Unable to control serial port signals on "<<portName<<"\n";
            return -1;
        }
        struct termios settings_orig;
        if (tcgetattr(fd, &settings_orig) < 0) {
            close(fd);
            sstm <<"Unable to access baud rate on port " << portName;
            errorString = sstm.str();
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
        _portOpened = true;
        
        
        //std::stringstream configString;
        //configString << "conf s:" << _samplingRate<<";c:"<<_numberOfChannels<<";\n";
        //writeToPort(configString.str().c_str(),configString.str().length());
        
        
        return fd;
    }
    
    const char * ArduinoSerial::currentPortName()
    {
        return _portName.c_str();
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
        int numberOfFrames = 0;
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
        
       std::cout<<"------------------ Size: "<<size<<"\n";
        for(int i=0;i<size;i++)
        {
            circularBuffer[cBufHead++] = buffer[i];
            //uint MSB  = ((uint)(buffer[i])) & 0xFF;
            //std::cout<<"M: " << MSB<<"\n";
            if(cBufHead>=SIZE_OF_CIRC_BUFFER)
            {
                cBufHead = 0;
            }
            
        }
        if(size==-1)
        {
            return -1;
        }
        uint LSB;
        uint MSB;

        bool haveData = true;
            while (haveData)
            {

                MSB  = ((uint)(circularBuffer[cBufTail])) & 0xFF;
                if(MSB > 127)//if we are at the begining of frame
                {
                    
                    if(checkIfHaveWholeFrame())
                    {
                        //std::cout<<"Inside serial "<< numberOfFrames<<"\n";
                        numberOfFrames++;
                        while (1)
                        {
                            //make sample value from two consecutive bytes
                           // std::cout<<"Tail: "<<cBufTail<<"\n";
                           //  MSB  = ((uint)(circularBuffer[cBufTail])) & 0xFF;
                            //std::cout<< cBufTail<<" -M "<<MSB<<"\n";
                            MSB  = ((uint)(circularBuffer[cBufTail])) & 0x7F;
                            
                            cBufTail++;
                            if(cBufTail>=SIZE_OF_CIRC_BUFFER)
                            {
                                cBufTail = 0;
                            }
                            LSB  = ((uint)(circularBuffer[cBufTail])) & 0xFF;
                            //if we have error in frame (lost data)
                            if(LSB>127)
                            {
                                numberOfFrames--;
                                break;//continue as if we have new frame
                            }
                           // std::cout<< cBufTail<<" -L "<<LSB<<"\n";
                            LSB  = ((uint)(circularBuffer[cBufTail])) & 0x7F;
                            
                            MSB = MSB<<7;
                            writeInteger = LSB | MSB;
                            
                            //std::cout<< obufferIndex<<" - "<<MSB<<":"<<LSB<<"\n";
                            obuffer[obufferIndex++] = writeInteger;
                            if(areWeAtTheEndOfFrame())
                            {
                                break;
                            }
                            else
                            {
                                cBufTail++;
                                if(cBufTail>=SIZE_OF_CIRC_BUFFER)
                                {
                                    cBufTail = 0;
                                }
                            }
                        }
                    }
                    else
                    {
                        haveData = false;
                        break;
                    }
                }
                if(!haveData)
                {
                    break;
                }
                cBufTail++;
                if(cBufTail>=SIZE_OF_CIRC_BUFFER)
                {
                    cBufTail = 0;
                }
                if(cBufTail==cBufHead)
                {
                    haveData = false;
                    break;
                }
                
                
            }
            
        
        return numberOfFrames;
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
    
    bool ArduinoSerial::checkIfHaveWholeFrame()
    {
        int tempTail = cBufTail + 1;
        if(tempTail>= SIZE_OF_CIRC_BUFFER)
        {
            tempTail = 0;
        }
        while(tempTail!=cBufHead)
        {
            uint nextByte  = ((uint)(circularBuffer[tempTail])) & 0xFF;
            if(nextByte > 127)
            {
                return true;
            }
            tempTail++;
            if(tempTail>= SIZE_OF_CIRC_BUFFER)
            {
                tempTail = 0;
            }
        }
        return false;
    }
    
    bool ArduinoSerial::areWeAtTheEndOfFrame()
    {
        int tempTail = cBufTail + 1;
        if(tempTail>= SIZE_OF_CIRC_BUFFER)
        {
            tempTail = 0;
        }
        uint nextByte  = ((uint)(circularBuffer[tempTail])) & 0xFF;
        if(nextByte > 127)
        {
            return true;
        }
        return false;
    }
    
    int ArduinoSerial::maxSamplingRate()
    {
        return 10000;
    }
    
    int ArduinoSerial::maxNumberOfChannels()
    {
    
        return 6;
    }
    
    int ArduinoSerial::numberOfChannels()
    {
        return _numberOfChannels;
    }
    
    
    bool ArduinoSerial::portOpened()
    {
        return _portOpened;
    }
    
    void ArduinoSerial::setNumberOfChannelsAndSamplingRate(int numberOfChannels, int samplingRate)
    {
        _numberOfChannels = numberOfChannels;
        _samplingRate = samplingRate;
        //"conf s:%d;c:%d;"
        std::stringstream sstm;
        sstm << "conf s:" << samplingRate<<";c:"<<numberOfChannels<<";\n";
        writeToPort(sstm.str().c_str(),sstm.str().length());
    }
    
    int ArduinoSerial::writeToPort(const void *ptr, int len)
    {
        int n, written=0;
        fd_set wfds;
        struct timeval tv;
        while (written < len) {
            n = write(fd, (const char *)ptr + written, len - written);
            if (n < 0 && (errno == EAGAIN || errno == EINTR)) n = 0;
            //printf("Write, n = %d\n", n);
            if (n < 0) return -1;
            if (n > 0) {
                written += n;
            } else {
                tv.tv_sec = 10;
                tv.tv_usec = 0;
                FD_ZERO(&wfds);
                FD_SET(fd, &wfds);
                n = select(fd+1, NULL, &wfds, NULL, &tv);
                if (n < 0 && errno == EINTR) n = 1;
                if (n <= 0) return -1;
            }
        }
        return written;
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
        _portOpened = false;
        /*#elif defined(WINDOWS)
        //SetCommConfig(port_handle, &port_cfg_orig, sizeof(COMMCONFIG));
        CloseHandle(port_handle);
        #endif*/
    }
    
    
    
    
    
}
