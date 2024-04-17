#pragma once

#include <vector>
#include <stdint.h>
#include <string>

//*************************************************************************************************

#define BLOCK_SIZE 64

class Connection
{
    public:
        Connection();
        virtual ~Connection();
        bool isConnected();
        int send(char msg);
        int sendMulti(const std::string& msg);
        bool recv(const std::string& expected);
        int clearRecvBuf();
        int bytesAvailToRead();
        std::string recvThreeHex();
        bool recvBlock(std::vector<uint8_t>& data);
        
        void waitMilliSec(uint32_t ms);

    private:
        void connect();
        void disconnect();
        int m_fd;
};

