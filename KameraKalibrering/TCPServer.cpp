/*
 * TCPServer.cpp
 *
 *  Created on: 28. apr. 2021
 *      Author: jeppe
 */

#include "TCPServer.h"

TCPServer::TCPServer() {
	//setup socket
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
	{
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	//attaching socket to port 21
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
	{
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}

	address.sin_family = AF_INET;
	address.sin_addr.s_addr = htonl(INADDR_ANY);
	address.sin_port = htons(PORT);

	if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}

	if (listen(server_fd, 3) < 0)
	{
		perror("listen");
		exit(EXIT_FAILURE);
	}

	if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0)
	{
		perror("accept");
		exit(EXIT_FAILURE);
	}
}

std::string TCPServer::recvSocket() {
	messageReceived = recv(new_socket, buffer, 1024,0);
	return messageReceived;
}

void TCPServer::sendSocket(std::string message) {
	std::cout << "Sending message: " << message << std::endl;
	send(new_socket, message.c_str(), message.length(), 0);
}

TCPServer::~TCPServer() {
	// TODO Auto-generated destructor stub
}

