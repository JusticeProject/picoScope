#include <stdio.h>
#include <unistd.h> // sleep
#include <sstream>
#include <iostream>
#include <fstream>
#include "math.h"

// Project headers
#include "connection.hpp"

//*************************************************************************************************

void displayMsg(const std::vector<uint8_t>& msg, bool displayChar)
{
    printf("Received %i bytes\n", msg.size());
    for (int i = 0; i < msg.size(); i++)
    {
        if (displayChar)
        {
            printf("%c 0x%x ", msg[i], msg[i]);
        }
        else
        {
            printf("0x%x ", msg[i]);
        }
    }
}

//*************************************************************************************************

uint16_t hexStrToInt(const std::string& input)
{
    std::stringstream ss;
    uint16_t result = 0;
    ss << std::hex << input;
    ss >> result;
    return result;
}

//*************************************************************************************************

void capture(Connection& conn, const std::string& sineWave, const std::string& clkDiv, const std::string& wrap, const std::string& duty, const std::string& filename)
{
    if (clkDiv.size() > 0 && wrap.size() > 0 && duty.size() > 0)
    {
        // turn on the PWM and set the clock divider
        conn.sendMulti("p" + clkDiv);
        conn.recv(clkDiv + "k");

        // set the wrap value for PWM
        conn.sendMulti("w" + wrap);
        conn.recv(wrap + "k");
    
        // set the duty cycle (number of cycles the signal is on)
        conn.sendMulti("d" + duty);
        conn.recv(duty + "k");
    
        sleep(1);
    }
    
    if (sineWave.size() > 0)
    {
        // turn on the sin wave and set the frequency, wait for the calculations to finish
        conn.sendMulti("s" + sineWave);
        conn.recv(sineWave + "k");
        printf("sin wave on\n");
        conn.waitMilliSec(250);
    }
    
    // start the ADC capture
    conn.send('c');
    conn.recv("k");
    
    conn.waitMilliSec(400);
    
    // turn off the sin wave
    conn.send('i');
    conn.recv("k");
    printf("sin wave off\n");
    conn.waitMilliSec(10);
    
    // set the duty cycle to 0
    conn.sendMulti("d00000");
    conn.recv("00000k");
    sleep(1);
    
    // turn off the PWM
    conn.send('m');
    conn.recv("k");
    printf("PWM off\n");
        
    // read all the ADC data, one block at a time
    std::vector<uint8_t> data;
    for (int i = 0; i < 2048; i++)
    {
        conn.send('l');
        bool success = conn.recvBlock(data);
        if (!success)
        {
            printf("failed to read block\n");
        }

        int leftOverBytes = conn.bytesAvailToRead();
        if (leftOverBytes > 0)
        {
            conn.clearRecvBuf();
        }
    }
    
    // clear the buffers of any remaining data
    conn.send('z');
    conn.clearRecvBuf();
    conn.send('z');
    conn.clearRecvBuf();
    conn.waitMilliSec(10);
    conn.clearRecvBuf();
    
    // write the data to a file
    std::ofstream myfile;
    myfile.open(filename, std::ios::out | std::ios::trunc);
    for (int i = 0; i < data.size(); i++)
    {
        std::string s = std::to_string(data[i]);
        myfile << s << std::endl;
    }
    myfile.close();
}

//*************************************************************************************************

int main()
{
    Connection conn;
    if (!conn.isConnected())
    {
        printf("not connected, not continuing\n");
        return -1;
    }

    // turn on the onboard LED
    int num_bytes = conn.send('n');
    conn.recv("k");
    
    conn.waitMilliSec(999);
    
    // turn off the LED
    num_bytes = conn.send('f');
    conn.recv("k");
    
    /*for (int i = 0; i < 10; i++)
    {
        num_bytes = conn.send('b');
        printf("Wrote %i bytes.\n", num_bytes);
        std::string adc = conn.recvThreeHex();
        printf("ADC hex = %s\n", adc.c_str());
        uint16_t adc_int = hexStrToInt(adc);
        printf("ADC int = %i\n", adc_int);
        sleep(1);
    }*/
    
    // 125MHz / 125 = 1MHz counter.
    // 60000 counts / 1MHz = 60ms PWM period
    capture(conn, "", "00125", "60000", "30000", "data1.txt");
    
    // The PWM duty cycle will be adjusted on the fly at 10 Hz
    // 125MHz / 5 = 25MHz counter.
    // 125 counts / 25MHz = 200kHz PWM
    capture(conn, "00010", "00005", "00125", "00062", "data2.txt");
    
    // no PWM
    capture(conn, "", "", "", "", "data3.txt");
    
    printf("end of program\n");
    return 0;
}

