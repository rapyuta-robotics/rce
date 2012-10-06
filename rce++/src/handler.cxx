/*
 *    handler.cxx
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

#include "handler.hxx"
#include "types.hxx"
#include "json_spirit/json_spirit.h"

namespace rce
{

///////////////////////////////////////////////////////////////////////////////
////	RCE_Client_Handler
///////////////////////////////////////////////////////////////////////////////
template<class Handler>
void RCE_Client_Handler<Handler>::on_message(connection_ptr con,
		message_ptr msg)
{
	bool binary;

	switch (msg->get_opcode())
	{
	case websocketpp::frame::opcode::TEXT:
		binary = false;
		break;

	case websocketpp::frame::opcode::BINARY:
		binary = true;
		break;

	default:
		// TODO: Throw an error here
		return;
	}

	_handler.receive(msg->get_payload(), binary);
}

template<class Handler>
void RCE_Client_Handler<Handler>::on_fail(connection_ptr con)
{
	// TODO: Throw an error here
}

template<class Handler>
void RCE_Client_Handler<Handler>::on_open(connection_ptr con)
{
	_handler.registerConnection(con);
}

template<class Handler>
void RCE_Client_Handler<Handler>::on_close(connection_ptr con)
{
	_handler.unregisterConnection(con);
}

///////////////////////////////////////////////////////////////////////////////
////	Handler_impl
///////////////////////////////////////////////////////////////////////////////
template<class Client>
void Handler_impl<Client>::receive(const std::string &message, bool bianary)
{
	Value_type type = json_spirit::find_value<Object_type, std::string>(
			message, "type");
	Value_type data = json_spirit::find_value<Object_type, std::string>(
			message, "data");

	if (type.type() != json_spirit::str_type || data.type()
			!= json_spirit::obj_type)
		// TODO: Here an invalid message was received...
		return;

	switch (type.get_str())
	{
	case RCE_DATA_MESSAGE:
		this->processData(data.get_obj());
		break;

	case RCE_STATUS:
		this->processStatus(data.get_obj());
		break;

	case RCE_ERROR:
		this->processError(data.get_obj());
		break;

	default:
		// TODO: Throw error; Invalid message type
		return;
	}
}

template<class Client>
void Handler_impl<Client>::registerConnection(Connection_type con)
{
	if (_con != NULL)
		// TODO: Throw an error
		return;

	_con = con;
}

template<class Client>
void Handler_impl<Client>::unregisterConnection(Connection_type con)
{
	if (_con == NULL)
		// TODO: Throw an error ?
		return;

	_con = NULL;
}

template<class Client>
void Handler_impl<Client>::registerInterface(const std::string &tag,
		const Interface_type *interface)
{
	typename _Interface_ref_vector::iterator it;

	for (it = _interfaces.begin(); it < _interfaces.end(); it++)
		if (it->first == tag && it->second == interface)
			// TODO: Throw error; Interface does already exist
			return;

	_interfaces.push_back(_Interface_ref(tag, interface));
}

template<class Client>
void Handler_impl<Client>::unregisterInterface(const std::string &tag,
		const Interface_type *interface)
{
	typename _Interface_ref_vector::iterator it;

	for (it = _interfaces.begin(); it < _interfaces.end(); it++)
		if (it->first == tag && it->second == interface)
			break;

	if (it != _interfaces.end())
		_interfaces.erase(it);
	else
		// TODO: Throw error; Interface does not exist
		return;
}

template<class Client>
void Handler_impl<Client>::send(const std::string &tag,
		const std::string &type, const Value_type &msg,
		const std::string &msgID)
{
	Object_type data;
	Object_type::add(data, "dest", tag);
	Object_type::add(data, "orig", _robotID);
	Object_type::add(data, "type", type);
	Object_type::add(data, "msgID", msgID);
	Object_type::add(data, "msg", msg);

	this->send(RCE_DATA_MESSAGE, data);
}

template<class Client>
void Handler_impl<Client>::send(const std::string &type,
		const Object_type &data)
{
	Object_type message;
	Object_type::add(message, "type", type);
	Object_type::add(message, "data", data);

	_Binary_vector binaries = _Binary_vector();
	_Binary_vector::iterator it;

	this->send(json_spirit::write(message, binaries), false);

	for (it = binaries.begin(); it != binaries.end(); ++it)
		this->send(*it, true);
}

template<class Client>
void Handler_impl<Client>::send(const std::string &message, bool binary)
{
	websocketpp::message::data_ptr msg = _con->get_data_message();

	if (binary)
		msg->reset(websocketpp::frame::opcode::BINARY);
	else
		msg->reset(websocketpp::frame::opcode::TEXT);

	msg->set_payload(message);
	_con->send(msg);
}

template<class Client>
void Handler_impl<Client>::processData(const Object_type &data)
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
		// TODO: Here an invalid message was received...
		return;

	if (dest.get_str() != _robotID)
		// TODO: Here a message which was not for this robot was received...
		return;

	std::string tag = orig.get_str();

	typename _Interface_ref_vector::iterator it;

	for (it = _interfaces.begin(); it < _interfaces.end(); it++)
		if (it->first == tag)
			it->second->receive(type.get_str(), msg, msgID.get_str());
}

template<class Client>
void Handler_impl<Client>::processStatus(const Object_type &data)
{

}

template<class Client>
void Handler_impl<Client>::processError(const Object_type &data)
{

}

} /* namespace rce */
