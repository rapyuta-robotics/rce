/*
 *    client.cxx
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

#include "client.hxx"
#include "types.hxx"

namespace rce
{

///////////////////////////////////////////////////////////////////////////////
////	Service_impl
///////////////////////////////////////////////////////////////////////////////
template<class Client, class ReqMsg, class RespMsg>
inline void ServiceClient_impl<Client, ReqMsg, RespMsg>::call(const ReqMsg &msg)
{
	call(msg, _cb);
}

template<class Client, class ReqMsg, class RespMsg>
void ServiceClient_impl<Client, ReqMsg, RespMsg>::call(const ReqMsg &msg,
		const Callback_type &cb)
{
	std::string uid = generateUUID<std::string> ();

	_callbacks.push_back(_Callback_ref(uid, cb));
	_handler.send(_tag, _srvType, msg.asJSON(), uid);
}

template<class Client, class ReqMsg, class RespMsg>
void ServiceClient_impl<Client, ReqMsg, RespMsg>::receive(
		const std::string &type, const Value_type &msg,
		const std::string &msgID)
{
	if (type != _srvType)
		// TODO: Here a message with an invalid type was received...
		return;

	typename _Callback_ref_vector::iterator it;

	for (it = _callbacks.begin(); it != _callbacks.end(); ++it)
		if (it->first == msgID)
		{
			(it->second).callback(RespMsg(msg));
			break;
		}

	if (it != _callbacks.end())
		_callbacks.erase(it);
	else
		// TODO: Here a message with an invalid msgID was received...
		return;
}

///////////////////////////////////////////////////////////////////////////////
////	Publisher_impl
///////////////////////////////////////////////////////////////////////////////
template<class Client, class Message>
void Publisher_impl<Client, Message>::publish(const Message &msg)
{
	_handler.send(_tag, _msgType, msg.asJSON(), "nil");
}

///////////////////////////////////////////////////////////////////////////////
////	Subscriber_impl
///////////////////////////////////////////////////////////////////////////////
template<class Client, class Message>
void Subscriber_impl<Client, Message>::receive(const std::string &type,
		const Value_type &msg, const std::string &msgID)
{
	if (type != _msgType)
		// TODO: Here a message with an invalid type was received...
		return;

	_cb.callback(Message(msg));
}

template<class Client, class Message>
void Subscriber_impl<Client, Message>::unsubscribe()
{
	_handler.registerInterface(_tag, this);
}

///////////////////////////////////////////////////////////////////////////////
////	Client_impl
///////////////////////////////////////////////////////////////////////////////
template<class Message>
void Client_impl<Message>::connect()
{

}

template<class Message>
void Client_impl<Message>::disconnect()
{

}

template<class Message>
void Client_impl<Message>::createContainer(const std::string &cTag) const
{
	Object_type data;
	Object_type::add(data, "containerTag", cTag);

	this->send(RCE_CREATE_CONTAINER, data);
}

template<class Message>
void Client_impl<Message>::destroyContainer(const std::string &cTag) const
{
	Object_type data;
	Object_type::add(data, "containerTag", cTag);

	this->send(RCE_DESTROY_CONTAINER, data);
}

template<class Message>
void Client_impl<Message>::addNode(const std::string &cTag,
		const std::string &nTag, const std::string &pkg,
		const std::string &exe, const std::string &rosNamespace) const
{
	Object_type component;
	Object_type::add(component, "containerTag", cTag);
	Object_type::add(component, "nodeTag", nTag);
	Object_type::add(component, "pkg", pkg);
	Object_type::add(component, "exe", exe);
	Object_type::add(component, "namespace", rosNamespace);

	this->configComponent("addNodes", Client_impl::Value_type(component));
}

template<class Message>
void Client_impl<Message>::removeNode(const std::string &cTag,
		const std::string &nTag) const
{
	Object_type component;
	Object_type::add(component, "containerTag", cTag);
	Object_type::add(component, "nodeTag", nTag);

	this->configComponent("removeNodes", Client_impl::Value_type(component));
}

template<class Message>
inline void Client_impl<Message>::addParameter(const std::string &cTag,
		const std::string &name, const int value) const
{
	this->addParameter(cTag, name, "int", Value_type(value));
}

template<class Message>
inline void Client_impl<Message>::addParameter(const std::string &cTag,
		const std::string &name, const std::string &value) const
{
	this->addParameter(cTag, name, "str", Value_type(value));
}

template<class Message>
inline void Client_impl<Message>::addParameter(const std::string &cTag,
		const std::string &name, const double value) const
{
	this->addParameter(cTag, name, "float", Value_type(value));
}

template<class Message>
inline void Client_impl<Message>::addParameter(const std::string &cTag,
		const std::string &name, const bool value) const
{
	this->addParameter(cTag, name, "bool", Value_type(value));
}

template<class Message>
void Client_impl<Message>::addParameter(const std::string &cTag,
		const std::string &name, const std::string &paramType,
		const Value_type &value) const
{
	Object_type component;
	Object_type::add(component, "containerTag", cTag);
	Object_type::add(component, "name", name);
	Object_type::add(component, "value", value);
	Object_type::add(component, "paramType", paramType);

	this->configComponent("setParam", Value_type(component));
}

template<class Message>
void Client_impl<Message>::removeParameter(const std::string &cTag,
		const std::string &name) const
{
	Object_type component;
	Object_type::add(component, "containerTag", cTag);
	Object_type::add(component, "name", name);

	this->configComponent("deleteParam", Value_type(component));
}

template<class Message>
void Client_impl<Message>::addInterface(const std::string &eTag,
		const std::string &iTag, const interface_t iType,
		const std::string &iCls, const std::string &addr) const
{
	Object_type component;
	Object_type::add(component, "endpointTag", eTag);
	Object_type::add(component, "interfaceTag", iTag);

	switch (iType)
	{
	case INTERFACE_SERVICE_CLIENT:
		Object_type::add(component, "interfaceType", "ServiceInterface");
		break;

	case INTERFACE_SERVICE_PROVIDER:
		Object_type::add(component, "interfaceType", "ServiceProviderInterface");
		break;

	case INTERFACE_PUBLISHER:
		Object_type::add(component, "interfaceType", "PublisherInterface");
		break;

	case INTERFACE_SUBSCRIBER:
		Object_type::add(component, "interfaceType", "SubscriberInterface");
		break;

	case CONVERTER_SERVICE_CLIENT:
		Object_type::add(component, "interfaceType", "ServiceConverter");
		break;

	case CONVERTER_SERVICE_PROVIDER:
		Object_type::add(component, "interfaceType", "ServiceProviderConverter");
		break;

	case CONVERTER_PUBLISHER:
		Object_type::add(component, "interfaceType", "PublisherConverter");
		break;

	case CONVERTER_SUBSCRIBER:
		Object_type::add(component, "interfaceType", "SubscriberConverter");
		break;

	case FORWARDER_SERVICE_CLIENT:
		Object_type::add(component, "interfaceType", "ServiceForwarder");
		break;

	case FORWARDER_SERVICE_PROVIDER:
		Object_type::add(component, "interfaceType", "ServiceProviderForwarder");
		break;

	case FORWARDER_PUBLISHER:
		Object_type::add(component, "interfaceType", "PublisherForwarder");
		break;

	case FORWARDER_SUBSCRIBER:
		Object_type::add(component, "interfaceType", "SubscriberForwarder");
		break;
	}

	Object_type::add(component, "className", iCls);
	Object_type::add(component, "addr", addr);

	this->configComponent("addInterfaces", Value_type(component));
}

template<class Message>
void Client_impl<Message>::removeInterface(const std::string &iTag) const
{
	this->configComponent("removeInterfaces", Value_type(iTag));
}

template<class Message>
void Client_impl<Message>::configComponent(const std::string &type,
		const Object_type &component) const
{
	Array_type array;
	array.push_back(component);

	Object_type data;
	Object_type::add(data, type, array);

	this->send(RCE_CONFIGURE_COMPONENT, data);
}

template<class Message>
inline void Client_impl<Message>::addConnection(const std::string &iTag1,
		const std::string &iTag2) const
{
	this->configConnection("connect", iTag1, iTag2);
}

template<class Message>
inline void Client_impl<Message>::removeConnection(const std::string &iTag1,
		const std::string &iTag2) const
{
	this->configConnection("disconnect", iTag1, iTag2);
}

template<class Message>
void Client_impl<Message>::configConnection(const std::string &type,
		const std::string &iTag1, const std::string &iTag2) const
{
	Object_type connection;
	Object_type::add(connection, "tagA", iTag1);
	Object_type::add(connection, "tagB", iTag2);

	Client_impl::Array_type array;
	array.push_back(connection);

	Object_type data;
	Object_type::add(data, type, array);

	this->send(RCE_CONFIGURE_CONNECTION, data);
}

template<class Message> template<class ReqMsg, class RespMsg>
Client_impl<Message>::ServiceClient_type<ReqMsg, RespMsg>&
Client_impl<Message>::service(const std::string &iTag,
		const std::string &srvType, const Callback_type &cb)
{
	return ServiceClient_type<ReqMsg, RespMsg> (_handler, iTag, srvType, cb);
}

template<class Message> template<class Msg>
Client_impl<Message>::Publisher_type<Msg>&
Client_impl<Message>::publisher(const std::string &iTag,
		const std::string &msgType)
{
	return Publisher_type<Msg> (_handler, iTag, msgType);
}

template<class Message> template<class Msg>
Client_impl<Message>::Subscriber_type<Msg>&
Client_impl<Message>::subscriber(const std::string &iTag,
		const std::string &msgType, const Callback_type &cb)
{
	return Subscriber_type<Msg> (_handler, iTag, msgType, cb);
}

} /* namespace rce */
