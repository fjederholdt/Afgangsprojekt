/*
 * TCPServer.h
 *
 *  Created on: 28. apr. 2021
 *      Author: jeppe
 */

#ifndef TCPSERVER_H_
#define TCPSERVER_H_

#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string>
#include <iostream>
#define PORT 2100

class TCPServer {
public:
	TCPServer();
	std::string recvSocket();
	void sendSocket(std::string message);
	virtual ~TCPServer();
private:
	int server_fd, new_socket;
	std::string messageReceived;
	struct sockaddr_in address;
	int opt = 1;
	int addrlen = sizeof(address);
	char *buffer[1024] = {0};
};

#endif /* TCPSERVER_H_ */
