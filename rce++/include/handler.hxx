/*
 *    handler.hxx
 *
 *    This file is part of the RoboEarth Cloud Engine rce++ client.
 *
 *    This file was originally created for RoboEearth
 *    http://www.roboearth.org/
 *
 *    The research leading to these results has received funding from
 *    the European Union Seventh Framework Programme FP7/2007-2013 under
 *    grant agreement no248942 RoboEarth.
 *
 *    Copyright 2012 RoboEarth
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 *
 *     \author/s: Dominique Hunziker
 */

#ifndef HANDLER_HXX_
#define HANDLER_HXX_

#include <websocketpp/roles/client.hpp>
#include <websocketpp/websocketpp.hpp>

namespace rce
{

template<class Handler>
class RCE_Client_Handler: public websocketpp::client::handler
{
public:
	RCE_Client_Handler(Handler &handler) :
		_handler(handler)
	{
	}

	void on_message(connection_ptr con, message_ptr msg);
	void on_fail(connection_ptr con);
	void on_open(connection_ptr con);
	void on_close(connection_ptr con);

private:
	const Handler &_handler;
};

template<class Client>
class Handler_impl
{
public:
	typedef typename websocketpp::client::handler::connection_ptr
			Connection_type;
	typedef typename Client::Interface_type Interface_type;
	typedef typename Client::Object_type Object_type;
	typedef typename Client::Value_type Value_type;

private:
	typedef typename Client::Config_type Config_type;
	typedef std::pair<std::string, Interface_type*> _Interface_ref;
	typedef std::vector<_Interface_ref> _Interface_ref_vector;
	typedef std::vector<std::string> _Binary_vector;

public:
	Handler_impl(const std::string &userID, const std::string &robotID) :
		_userID(userID), _robotID(robotID), _con(NULL), _interfaces(
				_Interface_ref_vector())
	{
	}

	void receive(const std::string &message, bool bianary);

	void registerConnection(Connection_type con);
	void unregisterConnection(Connection_type con);

	void registerInterface(const std::string &tag,
			const Interface_type *interface);
	void unregisterInterface(const std::string &tag,
			const Interface_type *interface);

	void send(const std::string &tag, const std::string &type,
			const Value_type &msg, const std::string &msgID);
	void send(const std::string &type, const Object_type &data);

private:
	void send(const std::string &message, bool binary);

	void processData(const Object_type &data);
	void processStatus(const Object_type &data);
	void processError(const Object_type &data);

	Connection_type _con; // Pointer to session object of websocketpp

	const std::string &_userID;
	const std::string &_robotID;

	_Interface_ref_vector &_interfaces;
};

} /* namespace rce */

#endif /* CLIENT_HXX_ */
