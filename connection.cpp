// tty config is from https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <time.h>
#include <sys/ioctl.h>

#include "connection.hpp"

//*************************************************************************************************

Connection::Connection()
{
    m_fd = -1;
    connect();
}

//*************************************************************************************************

Connection::~Connection()
{
    disconnect();
}

//*************************************************************************************************

bool Connection::isConnected()
{
    return m_fd > 0;
}

//*************************************************************************************************

int Connection::send(char msg)
{
    if (!isConnected())
    {
        return -1;
    }
    
    // Write to serial port
    int num_bytes = write(m_fd, &msg, 1);
    if (num_bytes < 0)
    {
        printf("Error writing: %s\n", strerror(errno));
    }
    return num_bytes;
}

//*************************************************************************************************

int Connection::sendMulti(const std::string& msg)
{
    if (!isConnected())
    {
        return -1;
    }
    
    // Write to serial port
    int num_bytes = write(m_fd, msg.c_str(), msg.size());
    if (num_bytes < 0)
    {
        printf("Error writing: %s\n", strerror(errno));
    }
    return num_bytes;
}

//*************************************************************************************************

void Connection::waitMilliSec(uint32_t ms)
{
    timespec t;
    t.tv_sec = 0;
    t.tv_nsec = ms * 1000000;
    nanosleep(&t, NULL);
}
        
//*************************************************************************************************

bool Connection::recv(const std::string& expected)
{
    if (!isConnected())
    {
        return false;
    }
    
    // Allocate memory for read buffer
    uint8_t read_buf[256];
    memset(&read_buf, '\0', sizeof(read_buf));
    
    const int MAX_TRIES = 5;
    int tries = 0;
    int num_bytes = 0;
    for (; tries < MAX_TRIES; tries++)
    {
        num_bytes += read(m_fd, read_buf + num_bytes, sizeof(read_buf) - num_bytes);
        if (num_bytes >= expected.size())
        {
            break;
        }
        //printf("waiting to read more of block, num_bytes = %i\n", num_bytes);
        waitMilliSec(1);
    }
    
    if (num_bytes < expected.size())
    {
        printf("Error reading: %s num_bytes = %u\n", strerror(errno), num_bytes);
        return false;
    }
    
    std::string recvString = "";
    for (int i = 0; i < num_bytes; i++)
    {
        recvString += read_buf[i];
    }
    
    if (recvString == expected)
    {
        printf("recv Success\n");
        return true;
    }
    else
    {
        printf("recv FAIL!! num_bytes = %u\n", num_bytes);
        for (int i = 0; i < num_bytes; i++)
        {
            printf("%u\n", read_buf[i]);
        }
        return false;
    }
}

//*************************************************************************************************

int Connection::clearRecvBuf()
{
    if (!isConnected())
    {
        return 0;
    }
    
    uint8_t read_buf[1024];
    int num_bytes = read(m_fd, read_buf, sizeof(read_buf));
    //printf("cleared %i bytes from the receive buffer:\n", num_bytes);
    for (int i = 0; i < num_bytes; i++)
    {
        printf("%u ", read_buf[i]);
    }
    printf("\n");
    return num_bytes;
}

//*************************************************************************************************

int Connection::bytesAvailToRead()
{
    int bytes = 0;
    ioctl(m_fd, FIONREAD, &bytes);
    return bytes;
}

//*************************************************************************************************

std::string Connection::recvThreeHex()
{
    if (!isConnected())
    {
        return "";
    }
    
    char read_buf[4];
    int num_bytes = read(m_fd, read_buf, 3);
    
    if (num_bytes < 0)
    {
        printf("Error in recvThreeHex: %s\n", strerror(errno));
    }
    
    read_buf[3] = 0; // make sure the char array is null-terminated
    std::string result = read_buf;
    return result;
}

//*************************************************************************************************

bool Connection::recvBlock(std::vector<uint8_t>& data)
{
    if (!isConnected())
    {
        return false;
    }
    
    uint8_t read_buf[BLOCK_SIZE];
    memset(read_buf, 0, sizeof(read_buf));

    const int MAX_TRIES = 5;
    int tries = 0;
    int num_bytes = 0;
    for (; tries < MAX_TRIES; tries++)
    {
        num_bytes += read(m_fd, read_buf + num_bytes, sizeof(read_buf) - num_bytes);
        if (num_bytes >= sizeof(read_buf))
        {
            break;
        }
        //printf("waiting to read more of block, num_bytes = %i\n", num_bytes);
        waitMilliSec(1);
    }
    
    //printf("read %u bytes of block\n", num_bytes);
    if (num_bytes < sizeof(read_buf))
    {
        printf("recvBlock failed. num_bytes = %i\n", num_bytes);
        return false;
    }
    
    for (int i = 0; i < sizeof(read_buf); i++)
    {
        data.push_back(read_buf[i]);
    }
    
    return true;
}

//*************************************************************************************************

void Connection::connect()
{
    m_fd = open("/dev/ttyACM0", O_RDWR);

    // Check for errors
    if (m_fd < 0)
    {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        return;
    }
    else
    {
        printf("Successfully opened tty file\n");
    }
    
    struct termios tty;
    if(tcgetattr(m_fd, &tty) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        disconnect();
        return;
    }
    
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= (CREAD | CLOCAL); // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 20;    // Wait for up to 2s (20 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    
    // Save tty settings, also checking for error
    if (tcsetattr(m_fd, TCSANOW, &tty) != 0)
    {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        disconnect();
        return;
    }
}

//*************************************************************************************************

void Connection::disconnect()
{
    if (m_fd > 0)
    {
        printf("disconnecting\n");
        close(m_fd);
    }
}

