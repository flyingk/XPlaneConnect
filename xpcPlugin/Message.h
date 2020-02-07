// Copyright (c) 2013-2018 United States Government as represented by the Administrator of the
// National Aeronautics and Space Administration. All Rights Reserved.
#ifndef XPCPLUGIN_MESSAGE_H_
#define XPCPLUGIN_MESSAGE_H_

#include "UDPSocket.h"

namespace XPC
{
	/// Represents a message received from an XPC client.
	///
	/// \author Jason Watkins
	/// \version 1.1
	/// \since 1.0
	/// \date Initial Version: 2015-04-11
	/// \date Last Updated: 2015-05-11
	class Message
	{
	public:
		/// Reads a datagram from the specified socket and interprets it as a
		/// message.
		///
		/// \param sock The socket to read from.
		/// \returns    A message parsed from the data read from sock. If no
		///             data was read or an error occurs, returns a message
		///             with the size set to 0.
		static Message ReadFrom(const UDPSocket& sock);

		/// Gets the message header.
		std::string GetHead() const;

		/// If the Head is DREF the return the dref name.
		std::string GetDrefName() const;

		/// Gets the buffer underlying the message.
		const unsigned char* GetBuffer() const;

		/// Gets the size of the message in bytes.
		std::size_t GetSize() const;

		/// Gets the address this message was read from.
		struct sockaddr GetSource() const;

		/// Prints the contents of the message to the XPC log.
		void PrintToLog() const;

		/// Copies a message
		void CopyMessage(Message &msg);

		Message();

	private:

		static const std::size_t bufferSize = 4096;
		unsigned char buffer[bufferSize];
		std::size_t size = 0;
		struct sockaddr source;
	};
}
#endif
