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

#define DEBUG 1

#ifdef DEBUG
#include <iostream>
#endif

#include <websocketpp/roles/client.hpp>
#include <websocketpp/websocketpp.hpp>
#include "json_spirit/json_spirit_reader_template.h"
#include "json_spirit/json_spirit_writer_template.h"
#include "json_spirit/json_spirit_utils.h"
#include "types.hxx"

namespace rce
{

class HandlerException: public std::runtime_error
{
	public:
		HandlerException(const std::string &e) :
			std::runtime_error::runtime_error(e)
		{
		}
};

template<class Client>
class Handler_Base_impl: public websocketpp::client::handler
{
	private:
		typedef typename Client::Config_type Config_type;
		typedef typename Client::Object_type Object_type;
		typedef typename Client::Value_type Value_type;
		typedef typename Client::Message_type::Binary _Binary;
		typedef std::pair<std::string, _Binary*> _Binary_in_type;
		typedef std::vector<_Binary_in_type> _Binary_in_vector;
		typedef std::pair<Object_type, _Binary_in_vector> _Pending_Message_type;
		typedef std::vector<_Pending_Message_type> _Pending_Message_vector;
		typedef std::vector<std::string> _Binary_out_vector;

	public:
		Handler_Base_impl(const std::string &mngr) :
			_manager(mngr)
		{
		}

		void on_message(connection_ptr con, message_ptr msg);
		void on_fail(connection_ptr con);
		void on_open(connection_ptr con);
		void on_close(connection_ptr con);

	protected:
		void processMessage(const std::string &msg);
		void processMessage(const Object_type &msg);
		void processBinary(const std::string &msg);

		virtual void processData(const Object_type &data);
		virtual void processStatus(const std::string &data);
		virtual void processError(const std::string &data);
		virtual void processInit(const Object_type &data);

		virtual void onConnect();

		void send(const std::string &type, const Object_type &data);

	private:
		void send(const std::string &message, bool binary);

	protected:
		const std::string _manager;

	private:
		connection_ptr _con; // Pointer to session object of websocketpp
		_Pending_Message_vector _pending;
};

template<class Client>
class Master_Handler_impl: public Handler_Base_impl<Client>
{
	private:
		typedef typename Client::Config_type Config_type;
		typedef typename Client::Object_type Object_type;
		typedef typename Client::Value_type Value_type;

	public:
		Master_Handler_impl(Client &client, const std::string &userID,
				const std::string &robotID) :
			Handler_Base_impl<Client> ("Master Manager"), _client(client),
					_userID(userID), _robotID(robotID)
		{
		}

	private:
		void onConnect();
		void processInit(const Object_type &data);

		Client &_client;

		const std::string _userID;
		const std::string _robotID;
};

template<class Client>
class Robot_Handler_impl: public Handler_Base_impl<Client>
{
	public:
		typedef typename Client::Interface_type Interface_type;
		typedef typename Client::Object_type Object_type;
		typedef typename Client::Value_type Value_type;

	private:
		typedef typename Client::Config_type Config_type;
		typedef std::pair<std::string, Interface_type*> _Interface_ref;
		typedef std::vector<_Interface_ref> _Interface_ref_vector;

	public:
		Robot_Handler_impl(const std::string &userID,
				const std::string &robotID, const std::string &key) :
			Handler_Base_impl<Client> ("Robot Manager"), _userID(userID),
					_robotID(robotID), _key(key)
		{
		}

		void registerInterface(const std::string &tag,
				const Interface_type *interface);
		void unregisterInterface(const std::string &tag,
				const Interface_type *interface);

		void send(const std::string &tag, const std::string &type,
				const Value_type &msg, const std::string &msgID);

	private:
		void onConnect();
		void processData(const Object_type &data);

		const std::string _userID;
		const std::string _robotID;
		const std::string _key;

		_Interface_ref_vector _interfaces;
};

///////////////////////////////////////////////////////////////////////////////
/////  Implementation
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
////	Handler_Base_impl
template<class Client>
void Handler_Base_impl<Client>::on_message(connection_ptr con, message_ptr msg)
{
	switch (msg->get_opcode())
	{
	case websocketpp::frame::opcode::TEXT:
		this->processMessage(msg->get_payload());
		break;

	case websocketpp::frame::opcode::BINARY:
		this->processBinary(msg->get_payload());
		break;

	default:
		throw HandlerException("Received message from " + _manager
				+ " has invalid type (websocket).");
	}
}

template<class Client>
void Handler_Base_impl<Client>::on_fail(connection_ptr con)
{
	throw HandlerException("Connection to " + _manager + " failed: "
			+ con->get_fail_reason());
}

template<class Client>
void Handler_Base_impl<Client>::on_open(connection_ptr con)
{
	if (_con != NULL)
		throw HandlerException(
				"Can not register connection. There is already a connection registered.");

	_con = con;

	this->onConnect();
}

template<class Client>
void Handler_Base_impl<Client>::on_close(connection_ptr con)
{
	if (_con == NULL)
		throw HandlerException(
				"Can not unregister connection. There is no connection registered.");

	_con = connection_ptr();
}

template<class Client>
void Handler_Base_impl<Client>::processMessage(const std::string &msg)
{
	Value_type messageValue;
	_Binary_in_vector binaries;

	json_spirit::read_string_or_throw(msg, messageValue, binaries);

	if (messageValue.type() != json_spirit::obj_type)
		throw HandlerException("Received a message from " + _manager
				+ " with invalid JSON format.");

	Object_type message = messageValue.get_obj();

	if (!binaries.empty())
		_pending.push_back(_Pending_Message_type(message, binaries));
	else
		this->processMessage(message);
}

template<class Client>
void Handler_Base_impl<Client>::processBinary(const std::string &msg)
{
	if (msg.length() < 32)
		throw HandlerException("Received a binary message with invalid header.");

	std::string uid = msg.substr(0, 32);
	std::string binary = msg.substr(32);

	typename _Pending_Message_vector::iterator msgIt;
	typename _Binary_in_vector::iterator binIt;

	for (msgIt = _pending.begin(); msgIt != _pending.end(); ++msgIt)
		for (binIt = msgIt->second.begin(); binIt != msgIt->second.end(); ++binIt)
			if (binIt->first == uid)
				goto match;

	match:

	if (msgIt != _pending.end() && binIt != msgIt->second.end())
	{
		binIt->second->set_data(binary);
		msgIt->second.erase(binIt);

		if (msgIt->second.empty())
		{
			this->processMessage(msgIt->first);
			_pending.erase(msgIt);
		}
	}
	else
		throw HandlerException("Received binary message was not expected.");
}

template<class Client>
void Handler_Base_impl<Client>::processMessage(const Object_type &msg)
{
	Value_type type = json_spirit::find_value<Object_type, std::string>(msg,
			"type");
	Value_type data = json_spirit::find_value<Object_type, std::string>(msg,
			"data");

	if (type.type() != json_spirit::str_type)
		throw HandlerException("Received a message from " + _manager
				+ " with invalid type for field 'type'.");

	std::string dataType = type.get_str();

	if (dataType == RCE_DATA_MESSAGE)
	{
		if (data.type() != json_spirit::obj_type)
			throw HandlerException("Received a message from " + _manager
					+ " with invalid type for field 'data'.");

		this->processData(data.get_obj());
	}
	else if (dataType == RCE_STATUS)
	{
		if (data.type() != json_spirit::str_type)
			throw HandlerException("Received a message from " + _manager
					+ " with invalid type for field 'data'.");

		this->processStatus(data.get_str());
	}
	else if (dataType == RCE_ERROR)
	{
		if (data.type() != json_spirit::str_type)
			throw HandlerException("Received a message from " + _manager
					+ " with invalid type for field 'data'.");

		this->processError(data.get_str());
	}
	else if (dataType == RCE_INIT)
	{
		if (data.type() != json_spirit::obj_type)
			throw HandlerException("Received a message from " + _manager
					+ " with invalid type for field 'data'.");

		this->processInit(data.get_obj());
	}
	else
		throw HandlerException("Received a message from " + _manager
				+ " with invalid type: " + dataType);
}

template<class Client>
void Handler_Base_impl<Client>::processData(const Object_type &data)
{
}

template<class Client>
void Handler_Base_impl<Client>::processStatus(const std::string &data)
{
	std::cout << data << std::endl;
}

template<class Client>
void Handler_Base_impl<Client>::processError(const std::string &data)
{
	std::cout << data << std::endl;
}

template<class Client>
void Handler_Base_impl<Client>::processInit(const Object_type &data)
{
}

template<class Client>
void Handler_Base_impl<Client>::onConnect()
{
#ifdef DEBUG
	std::cout << "Connection established." << std::endl;
#endif
}

template<class Client>
void Handler_Base_impl<Client>::send(const std::string &type,
		const Object_type &data)
{
	Object_type message;
	Config_type::add(message, "type", type);
	Config_type::add(message, "data", data);

	_Binary_out_vector binaries = _Binary_out_vector();
	_Binary_out_vector::iterator it;

	this->send(json_spirit::write_string(Value_type(message), binaries, false),
			false);

	for (it = binaries.begin(); it != binaries.end(); ++it)
		this->send(*it, true);
}

template<class Client>
void Handler_Base_impl<Client>::send(const std::string &message, bool binary)
{
#ifdef DEBUG
	if (!binary)
		std::cout << "Send message: " << message << std::endl;
#endif
	websocketpp::message::data_ptr msg = _con->get_data_message();

	if (binary)
		msg->reset(websocketpp::frame::opcode::BINARY);
	else
		msg->reset(websocketpp::frame::opcode::TEXT);

	msg->set_payload(message);
	_con->send(msg);
}

///////////////////////////////////////////////////////////////////////////////
////	Master_Handler_impl
template<class Client>
void Master_Handler_impl<Client>::onConnect()
{
	Object_type initMsg;
	Config_type::add(initMsg, "userID", _userID);
	Config_type::add(initMsg, "robotID", _robotID);

	Handler_Base_impl<Client>::send(RCE_INIT, initMsg);
}

template<class Client>
void Master_Handler_impl<Client>::processInit(const Object_type &msg)
{
	Value_type key = json_spirit::find_value<Object_type, std::string>(msg,
			"key");
	Value_type url = json_spirit::find_value<Object_type, std::string>(msg,
			"url");

	if (key.type() != json_spirit::str_type || url.type()
			!= json_spirit::str_type)
		throw HandlerException("Received a message " + this->_manager
				+ " with invalid format.");

	_client._handler = typename Client::Robot_ptr(
			new typename Client::Robot_type(_userID, _robotID, key.get_str()));
	typename Client::_Handler_ptr robotHandler(_client._handler);
	_client._endpoint = typename Client::_Client_ptr(
			new typename Client::_Client_type(robotHandler));

#ifdef DEBUG
	_client._endpoint->alog().set_level(websocketpp::log::alevel::ALL);
	_client._endpoint->elog().set_level(websocketpp::log::elevel::ALL);
#else
    _client._endpoint->alog().unset_level(websocketpp::log::alevel::ALL);
    _client._endpoint->elog().unset_level(websocketpp::log::elevel::ALL);
#endif

	_client._endpoint->connect(url.get_str());
	_client._endpoint->run();
}

///////////////////////////////////////////////////////////////////////////////
////	Robot_Handler_impl
template<class Client>
void Robot_Handler_impl<Client>::registerInterface(const std::string &tag,
		const Interface_type *interface)
{
	typename _Interface_ref_vector::iterator it;

	for (it = _interfaces.begin(); it < _interfaces.end(); it++)
		if (it->first == tag && it->second == interface)
			throw HandlerException(
					"Can not register Interface. It is already registered.");

	_interfaces.push_back(_Interface_ref(tag, interface));
}

template<class Client>
void Robot_Handler_impl<Client>::unregisterInterface(const std::string &tag,
		const Interface_type *interface)
{
	typename _Interface_ref_vector::iterator it;

	for (it = _interfaces.begin(); it < _interfaces.end(); it++)
		if (it->first == tag && it->second == interface)
			break;

	if (it != _interfaces.end())
		_interfaces.erase(it);
	else
		throw HandlerException(
				"Can not unregister Interface. It is not registered.");
}

template<class Client>
void Robot_Handler_impl<Client>::send(const std::string &tag,
		const std::string &type, const Value_type &msg,
		const std::string &msgID)
{
	Object_type data;
	Config_type::add(data, "dest", tag);
	Config_type::add(data, "orig", _robotID);
	Config_type::add(data, "type", type);
	Config_type::add(data, "msgID", msgID);
	Config_type::add(data, "msg", msg);

	Handler_Base_impl<Client>::send(RCE_DATA_MESSAGE, data);
}

template<class Client>
void Robot_Handler_impl<Client>::onConnect()
{
	Object_type initMsg;
	Config_type::add(initMsg, "userID", _userID);
	Config_type::add(initMsg, "robotID", _robotID);
	Config_type::add(initMsg, "key", _key);

	Handler_Base_impl<Client>::send(RCE_INIT, initMsg);
}

template<class Client>
void Robot_Handler_impl<Client>::processData(const Object_type &data)
{
	Value_type orig = json_spirit::find_value<Object_type, std::string>(data,
			"orig");
	Value_type dest = json_spirit::find_value<Object_type, std::string>(data,
			"dest");
	Value_type type = json_spirit::find_value<Object_type, std::string>(data,
			"type");
	Value_type msgID = json_spirit::find_value<Object_type, std::string>(data,
			"msgID");
	Value_type msg = json_spirit::find_value<Object_type, std::string>(data,
			"msg");

	if (orig.type() != json_spirit::str_type || dest.type()
			!= json_spirit::str_type || type.type() != json_spirit::str_type
			|| msgID.type() != json_spirit::str_type || (msg.type()
			!= json_spirit::obj_type || msg.type() != json_spirit::bin_type))
		throw HandlerException(
				"Received a message from Robot Manager with invalid format.");

	if (dest.get_str() != _robotID)
		throw HandlerException(
				"Received a message from Robot Manager where the robot ID does not match.");

	std::string tag = orig.get_str();

	typename _Interface_ref_vector::iterator it;

	for (it = _interfaces.begin(); it < _interfaces.end(); it++)
		if (it->first == tag)
			it->second->receive(type.get_str(), msg, msgID.get_str());
}

} /* namespace rce */

#endif /* CLIENT_HXX_ */
