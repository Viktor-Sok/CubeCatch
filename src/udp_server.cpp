#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <fstream>
#include <cstring> // provides the additional functionality to work with strings and chars ( like <string>.c_str() )
#include <string>
#include "udp_server.h"

inline void make_addr(sockaddr_in& _serverAddr, int portNumber)
{
    memset(&_serverAddr, sizeof(_serverAddr), 0);
    _serverAddr.sin_family = AF_INET;
    _serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    _serverAddr.sin_port = htons(portNumber);  
}

UdpServer::UdpServer(int portNumber) 
{
    if (WSAStartup(MAKEWORD(2,2), &_wsaData) != 0) // initializes winsock.dll
	{
	    std::fprintf(stderr, "Could not open Windows connection.\n");
	    
	}
    else
    {
        std::cout << "The Winsok status: " << _wsaData.szSystemStatus << std::endl;
    }
    
    _sock = INVALID_SOCKET;
    _sock = socket(AF_INET,SOCK_DGRAM,0);
    if (_sock == INVALID_SOCKET)
    {
        std::cout << "Error at socket: "<< WSAGetLastError()<< std::endl;
        WSACleanup(); // stop using Windows Socket dll (resources clean up)
    }
    else
    {
        std::cout << "Socket is working!" << std::endl;
	}

    
    make_addr(_serverAddr, portNumber);
    _size = sizeof(_clientAddr);
    
    if (bind(_sock, (struct sockaddr *)&_serverAddr, sizeof(_serverAddr)) == SOCKET_ERROR)
    { 
        std::cout << "socket bind failure: " << WSAGetLastError() << std::endl;
        closesocket(_sock);
        WSACleanup();
    }
    else
    {
        std::cout << "Socket bind is successful!" << std::endl;
        std::cout <<"Ready to receive messages from the Robot." << std::endl;
    }
}

UdpServer::~UdpServer() // destructor
{
    closesocket(_sock);
    WSACleanup();
}

int UdpServer::read(char* readBuffer)
{
    int bytesReceived = recvfrom(_sock, readBuffer, 1400, 0, (struct sockaddr *)&_clientAddr, &_size);
    if (bytesReceived == SOCKET_ERROR)
    {
        _status = "Could not receive datagram: " + WSAGetLastError();
        return -1;
    }
    return bytesReceived;
}



int UdpServer::write(std::string& writeBuffer)
{
    int bytesSent = sendto(_sock, writeBuffer.c_str(), writeBuffer.length(), 0, (struct sockaddr *)&_clientAddr, _size);
    if (bytesSent == SOCKET_ERROR)
    {
        _status = "Could not receive datagram." + WSAGetLastError();
        return -1;
    }
    return bytesSent;
}

std::string const& UdpServer::status() const // getter
{
    return _status;
}
    

    




