/*
 *    client.hxx
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

#ifndef CLIENT_HXX_
#define CLIENT_HXX_

#include "types.hxx"
#include "handler.hxx"

namespace rce
{

enum interface_t
{
	INTERFACE_SERVICE_CLIENT,
	INTERFACE_SERVICE_PROVIDER,
	INTERFACE_PUBLISHER,
	INTERFACE_SUBSCRIBER,
	CONVERTER_SERVICE_CLIENT,
	CONVERTER_SERVICE_PROVIDER,
	CONVERTER_PUBLISHER,
	CONVERTER_SUBSCRIBER,
	FORWARDER_SERVICE_CLIENT,
	FORWARDER_SERVICE_PROVIDER,
	FORWARDER_PUBLISHER,
	FORWARDER_SUBSCRIBER
};

class ClientException: public std::runtime_error
{
};

template<class Config>
class Message_impl
{
	public:
		typedef Config Config_type;
		typedef typename Config::String_type String_type;
		typedef typename Config::Value_type Value;
		typedef typename Config::Object_type Object;
		typedef typename Config::Binary_type Binary;
		typedef typename Config::Array_type Array;

		Message_impl()
		{
		}

		/*
		 * Constructor which creates the message from a received JSON
		 * representation.
		 */
		Message_impl(const Value &serialized)
		{
		}

		/*
		 *Create a JSON representation of this message.
		 */
		virtual const Value asJSON() = 0;
};

template<class Client>
class Interface_impl
{
	public:
		typedef typename Client::Value_type Value_type;

		virtual void receive(const std::string &type, const Value_type &msg,
				const std::string &msgID) = 0;
};

template<class Client, class ReqMsg, class RespMsg>
class ServiceClient_impl: public Interface_impl<Client>
{
	public:
		typedef typename Client::Value_type Value_type;
		typedef typename Client::template Callback_type<RespMsg> Callback_type;
		typedef typename Client::Robot_ptr Robot_ptr;

	private:
		typedef std::pair<std::string, Callback_type> _Callback_ref;
		typedef std::vector<_Callback_ref> _Callback_ref_vector;

	public:
		ServiceClient_impl(const Robot_ptr handler, const std::string &tag,
				const std::string &srvType, const Callback_type &cb) :
			_handler(handler), _tag(tag), _srvType(srvType), _cb(cb)
		{
			_handler.registerInterface(_tag, this);
		}

		~ServiceClient_impl()
		{
			_handler.registerInterface(_tag, this);
		}

		/*
		 * Call Service.
		 *
		 * @param cb:	Overwrite default callback for one service call.
		 */
		void call(const ReqMsg &msg);
		void call(const ReqMsg &msg, const Callback_type &cb);

		void receive(const std::string &type, const Value_type &msg,
				const std::string &msgID);

	private:
		const Robot_ptr _handler;

		const Callback_type _cb;

		const std::string _tag;
		const std::string _srvType;

		_Callback_ref_vector _callbacks;
};

template<class Client, class Message>
class Publisher_impl
{
	public:
        typedef typename Client::Robot_ptr Robot_ptr;

		Publisher_impl(const Robot_ptr handler, const std::string &tag,
				const std::string &msgType) :
			_handler(handler), _tag(tag), _msgType(msgType)
		{
		}

		/*
		 * Publish message.
		 */
		void publish(const Message &msg);

	private:
		const Robot_ptr _handler;

		const std::string _tag;
		const std::string _msgType;
};

template<class Client, class Message>
class Subscriber_impl: public Interface_impl<Client>
{
	public:
		typedef typename Client::Value_type Value_type;
		typedef typename Client::template Callback_type<Message> Callback_type;
		typedef typename Client::Robot_ptr Robot_ptr;

		Subscriber_impl(const Robot_ptr handler, const std::string &tag,
				const std::string &msgType, const Callback_type &cb) :
			_handler(handler), _tag(tag), _msgType(msgType), _cb(cb)
		{
			_handler.registerInterface(_tag, this);
		}

		~Subscriber_impl()
		{
			this->unsubscribe();
		}

		void receive(const std::string &type, const Value_type &msg,
				const std::string &msgID);

		void unsubscribe();

	private:
		const Robot_ptr _handler;

		const Callback_type _cb;

		const std::string _tag;
		const std::string _msgType;
};

template<class Message>
class Client_impl
{
	public:
		typedef Message Message_type;
		typedef typename Message::Value Value_type;
		typedef typename Message::Array Array_type;
		typedef typename Message::Object Object_type;
		typedef typename Message::Config_type Config_type;
		typedef Interface_impl<Client_impl> Interface_type;
		template<class Msg> class Callback_type: public boost::function<void(
				const Msg &msg)>
		{
		};
		template<class ReqMsg, class RespMsg> class ServiceClient_type: public ServiceClient_impl<
				Client_impl, ReqMsg, RespMsg>
		{
		};
		template<class Msg> class Publisher_type: public Publisher_impl<
				Client_impl, Msg>
		{
		};
		template<class Msg> class Subscriber_type: public Subscriber_impl<
				Client_impl, Msg>
		{
		};

		typedef Robot_Handler_impl<Client_impl> Robot_type;
		typedef boost::shared_ptr<Robot_type> Robot_ptr;

	private:
		typedef Master_Handler_impl<Client_impl> _Master_type;
		typedef websocketpp::client::handler::ptr _Handler_ptr;
		typedef websocketpp::client _Client_type;
		typedef boost::shared_ptr<websocketpp::client> _Client_ptr;

	public:
		Client_impl(const std::string &userID, const std::string &robotID) :
			_userID(userID), _robotID(robotID)
		{
		}

		void connect(const std::string &url);
		void disconnect();

		void createContainer(const std::string &cTag) const;
		void destroyContainer(const std::string &cTag) const;

		void addNode(const std::string &cTag, const std::string &nTag,
				const std::string &pkg, const std::string &exe,
				const std::string &args, const std::string &name,
				const std::string &rosNamespace) const;
		void removeNode(const std::string &cTag, const std::string &nTag) const;

		void addParameter(const std::string &cTag, const std::string &name,
				const int value) const;
		void addParameter(const std::string &cTag, const std::string &name,
				const std::string &value) const;
		void addParameter(const std::string &cTag, const std::string &name,
				const double value) const;
		void addParameter(const std::string &cTag, const std::string &name,
				const bool value) const;
		void
		removeParameter(const std::string &cTag, const std::string &name) const;

		void addInterface(const std::string &eTag, const std::string &iTag,
				const interface_t iType, const std::string &iCls,
				const std::string &addr) const;
		void removeInterface(const std::string &iTag) const;

		void
		addConnection(const std::string &iTag1, const std::string &iTag2) const;
		void
				removeConnection(const std::string &iTag1,
						const std::string &iTag2) const;

		template<class ReqMsg, class RespMsg>
		ServiceClient_type<ReqMsg, RespMsg>
		service(const std::string &iTag, const std::string &srvType,
				const Callback_type<RespMsg> &cb);

		template<class Msg>
		Publisher_type<Msg> publisher(const std::string &iTag,
				const std::string &msgType);

		template<class Msg>
		Subscriber_type<Msg> subscriber(const std::string &iTag,
				const std::string &msgType, const Callback_type<Msg> &cb);

	private:
		friend void _Master_type::processInit(const Object_type &msg);

		void addParameter(const std::string &cTag, const std::string &name,
				const std::string &paramType, const Value_type &value) const;
		void
				configComponent(const std::string &type,
						const Object_type &component) const;
		void configConnection(const std::string &type,
				const std::string &iTag1, const std::string &iTag2) const;

		_Client_ptr _endpoint;
		Robot_ptr _handler;

		const std::string _userID;
		const std::string _robotID;
};

typedef Message_impl<json_spirit::Config> Message;
typedef Client_impl<Message> Client;
template<class Message> class Callback: public Client::Callback_type<Message>
{
};
template<class ReqMsg, class RespMsg> class ServiceClient: public Client::ServiceClient_type<
		ReqMsg, RespMsg>
{
};
template<class Message> class Publisher: public Client::Publisher_type<Message>
{
};
template<class Message> class Subscriber: public Client::Subscriber_type<
		Message>
{
};

typedef Message_impl<json_spirit::mConfig> mMessage;
typedef Client_impl<mMessage> mClient;
template<class Message> class mCallback: public mClient::Callback_type<Message>
{
};
template<class ReqMsg, class RespMsg> class mServiceClient: public mClient::ServiceClient_type<
		ReqMsg, RespMsg>
{
};
template<class Message> class mPublisher: public mClient::Publisher_type<
		Message>
{
};
template<class Message> class mSubscriber: public mClient::Subscriber_type<
		Message>
{
};

///////////////////////////////////////////////////////////////////////////////
/////  Implementation
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
////	Service_impl
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
	_handler->send(_tag, _srvType, msg.asJSON(), uid);
}

template<class Client, class ReqMsg, class RespMsg>
void ServiceClient_impl<Client, ReqMsg, RespMsg>::receive(
		const std::string &type, const Value_type &msg,
		const std::string &msgID)
{
	if (type != _srvType)
		throw ClientException("Received Service response with invalid type.");

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
		throw ClientException(
				"Received Service response with invalid message ID.");
}

///////////////////////////////////////////////////////////////////////////////
////	Publisher_impl
template<class Client, class Message>
void Publisher_impl<Client, Message>::publish(const Message &msg)
{
	_handler->send(_tag, _msgType, msg.asJSON(), "nil");
}

///////////////////////////////////////////////////////////////////////////////
////	Subscriber_impl
template<class Client, class Message>
void Subscriber_impl<Client, Message>::receive(const std::string &type,
		const Value_type &msg, const std::string &msgID)
{
	if (type != _msgType)
		throw ClientException("Received Message with invalid type.");

	_cb.callback(Message(msg));
}

template<class Client, class Message>
void Subscriber_impl<Client, Message>::unsubscribe()
{
	_handler->unregisterInterface(_tag, this);
}

///////////////////////////////////////////////////////////////////////////////
////	Client_impl
template<class Message>
void Client_impl<Message>::connect(const std::string &url)
{
#ifdef DEBUG
	std::cout << "Start connection with:" << std::endl;
	std::cout << "    userID: " << _userID << std::endl;
	std::cout << "    robotID: " << _userID << std::endl;
#endif

	_Handler_ptr masterHandler(new _Master_type(*this, _robotID, _userID));
	_endpoint = _Client_ptr(new _Client_type(masterHandler));

#ifdef DEBUG
    _endpoint->alog().set_level(websocketpp::log::alevel::ALL);
    _endpoint->elog().set_level(websocketpp::log::elevel::ALL);
#else
    _endpoint->alog().unset_level(websocketpp::log::alevel::ALL);
    _endpoint->elog().unset_level(websocketpp::log::elevel::ALL);
#endif

	_endpoint->connect(url);
	_endpoint->run();
}

template<class Message>
void Client_impl<Message>::disconnect()
{
	_endpoint->reset();
}

template<class Message>
void Client_impl<Message>::createContainer(const std::string &cTag) const
{
	Object_type data;
	Config_type::add(data, "containerTag", cTag);

	this->send(RCE_CREATE_CONTAINER, data);
}

template<class Message>
void Client_impl<Message>::destroyContainer(const std::string &cTag) const
{
	Object_type data;
	Config_type::add(data, "containerTag", cTag);

	this->send(RCE_DESTROY_CONTAINER, data);
}

template<class Message>
void Client_impl<Message>::addNode(const std::string &cTag,
		const std::string &nTag, const std::string &pkg,
		const std::string &exe, const std::string &args,
		const std::string &name, const std::string &rosNamespace) const
{
	Object_type component;
	Config_type::add(component, "containerTag", cTag);
	Config_type::add(component, "nodeTag", nTag);
	Config_type::add(component, "pkg", pkg);
	Config_type::add(component, "exe", exe);
	Config_type::add(component, "args", args);
	Config_type::add(component, "name", name);
	Config_type::add(component, "namespace", rosNamespace);

	this->configComponent("addNodes", Client_impl::Value_type(component));
}

template<class Message>
void Client_impl<Message>::removeNode(const std::string &cTag,
		const std::string &nTag) const
{
	Object_type component;
	Config_type::add(component, "containerTag", cTag);
	Config_type::add(component, "nodeTag", nTag);

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
	Config_type::add(component, "containerTag", cTag);
	Config_type::add(component, "name", name);
	Config_type::add(component, "value", value);
	Config_type::add(component, "paramType", paramType);

	this->configComponent("setParam", Value_type(component));
}

template<class Message>
void Client_impl<Message>::removeParameter(const std::string &cTag,
		const std::string &name) const
{
	Object_type component;
	Config_type::add(component, "containerTag", cTag);
	Config_type::add(component, "name", name);

	this->configComponent("deleteParam", Value_type(component));
}

template<class Message>
void Client_impl<Message>::addInterface(const std::string &eTag,
		const std::string &iTag, const interface_t iType,
		const std::string &iCls, const std::string &addr) const
{
	Object_type component;
	Config_type::add(component, "endpointTag", eTag);
	Config_type::add(component, "interfaceTag", iTag);

	switch (iType)
	{
	case INTERFACE_SERVICE_CLIENT:
		Config_type::add(component, "interfaceType", "ServiceInterface");
		break;

	case INTERFACE_SERVICE_PROVIDER:
		Config_type::add(component, "interfaceType", "ServiceProviderInterface");
		break;

	case INTERFACE_PUBLISHER:
		Config_type::add(component, "interfaceType", "PublisherInterface");
		break;

	case INTERFACE_SUBSCRIBER:
		Config_type::add(component, "interfaceType", "SubscriberInterface");
		break;

	case CONVERTER_SERVICE_CLIENT:
		Config_type::add(component, "interfaceType", "ServiceConverter");
		break;

	case CONVERTER_SERVICE_PROVIDER:
		Config_type::add(component, "interfaceType", "ServiceProviderConverter");
		break;

	case CONVERTER_PUBLISHER:
		Config_type::add(component, "interfaceType", "PublisherConverter");
		break;

	case CONVERTER_SUBSCRIBER:
		Config_type::add(component, "interfaceType", "SubscriberConverter");
		break;

	case FORWARDER_SERVICE_CLIENT:
		Config_type::add(component, "interfaceType", "ServiceForwarder");
		break;

	case FORWARDER_SERVICE_PROVIDER:
		Config_type::add(component, "interfaceType", "ServiceProviderForwarder");
		break;

	case FORWARDER_PUBLISHER:
		Config_type::add(component, "interfaceType", "PublisherForwarder");
		break;

	case FORWARDER_SUBSCRIBER:
		Config_type::add(component, "interfaceType", "SubscriberForwarder");
		break;
	}

	Config_type::add(component, "className", iCls);
	Config_type::add(component, "addr", addr);

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
	Config_type::add(data, type, array);

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
	Config_type::add(connection, "tagA", iTag1);
	Config_type::add(connection, "tagB", iTag2);

	Client_impl::Array_type array;
	array.push_back(connection);

	Object_type data;
	Config_type::add(data, type, array);

	this->send(RCE_CONFIGURE_CONNECTION, data);
}

template<class Message> template<class ReqMsg, class RespMsg>
Client_impl<Message>::ServiceClient_type<ReqMsg, RespMsg> Client_impl<Message>::service(
		const std::string &iTag, const std::string &srvType,
		const Callback_type<RespMsg> &cb)
{
	return ServiceClient_type<ReqMsg, RespMsg> (_handler, iTag, srvType, cb);
}

template<class Message> template<class Msg>
Client_impl<Message>::Publisher_type<Msg> Client_impl<Message>::publisher(
		const std::string &iTag, const std::string &msgType)
{
	return Publisher_type<Msg> (_handler, iTag, msgType);
}

template<class Message> template<class Msg>
Client_impl<Message>::Subscriber_type<Msg> Client_impl<Message>::subscriber(
		const std::string &iTag, const std::string &msgType,
		const Callback_type<Msg> &cb)
{
	return Subscriber_type<Msg> (_handler, iTag, msgType, cb);
}

} /* namespace rce */

#endif /* CLIENT_HXX_ */
