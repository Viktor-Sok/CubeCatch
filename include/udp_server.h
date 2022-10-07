#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include <string>
#include <winsock2.h>
#include <ws2tcpip.h>

class UdpServer
{
private:
    WSADATA _wsaData;
    SOCKET _sock;
    sockaddr_in _serverAddr, _clientAddr;
    std::string _status;
    int _size;
    
public:
    UdpServer(int portNumber);
    ~UdpServer();
    int read(char* readBuffer);
    int write(std::string& writeBuffer); 
    std::string const& status() const; 
    
    // preventing copying the object which contains socket.
    UdpServer(const UdpServer&) = delete;
    void operator=(const UdpServer&) = delete;
};

#endif /* UDP_SERVER_H */
