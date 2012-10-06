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

#include "json_spirit/json_spirit.h"
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
	virtual const Value& asJSON() = 0;
};

template<class Client>
class Callback_impl
{
public:
	typedef typename Client::Message_type Message_type;

	/*
	 * Callback method is invoked when callback is necessary.
	 */
	virtual void callback(const Message_type &msg) = 0;
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
	typedef typename Client::Callback_type Callback_type;
	typedef typename Client::Handler_type Handler_type;

private:
	typedef std::pair<std::string, Callback_type> _Callback_ref;
	typedef std::vector<_Callback_ref> _Callback_ref_vector;

public:
	ServiceClient_impl(const Handler_type &handler, const std::string &tag,
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
	const Handler_type &_handler;

	const Callback_type &_cb;

	const std::string &_tag;
	const std::string &_srvType;

	_Callback_ref_vector &_callbacks;
};

template<class Client, class Message>
class Publisher_impl
{
public:
	typedef typename Client::Handler_type Handler_type;

	Publisher_impl(const Handler_type &handler, const std::string &tag,
			const std::string &msgType) :
		_handler(handler), _tag(tag), _msgType(msgType)
	{
	}

	/*
	 * Publish message.
	 */
	void publish(const Message &msg);

private:
	const Handler_type &_handler;

	const std::string &_tag;
	const std::string &_msgType;
};

template<class Client, class Message>
class Subscriber_impl: public Interface_impl<Client>
{
public:
	typedef typename Client::Value_type Value_type;
	typedef typename Client::Callback_type Callback_type;
	typedef typename Client::Handler_type Handler_type;

	Subscriber_impl(const Handler_type &handler, const std::string &tag,
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
	const Handler_type &_handler;

	const Callback_type &_cb;

	const std::string &_tag;
	const std::string &_msgType;
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
	typedef Handler_impl<Client_impl> Handler_type;
	typedef Callback_impl<Client_impl> Callback_type;
	typedef Interface_impl<Client_impl> Interface_type;
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

	Client_impl(const std::string &userID, const std::string &robotID) :
		_userID(userID), _robotID(robotID)
	{
	}

	void connect();
	void disconnect();

	void createContainer(const std::string &cTag) const;
	void destroyContainer(const std::string &cTag) const;

	void addNode(const std::string &cTag, const std::string &nTag,
			const std::string &pkg, const std::string &exe,
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
	void removeParameter(const std::string &cTag, const std::string &name) const;

	void addInterface(const std::string &eTag, const std::string &iTag,
			const interface_t iType, const std::string &iCls,
			const std::string &addr) const;
	void removeInterface(const std::string &iTag) const;

	void addConnection(const std::string &iTag1, const std::string &iTag2) const;
	void
	removeConnection(const std::string &iTag1, const std::string &iTag2) const;

	template<class ReqMsg, class RespMsg>
	ServiceClient_type<ReqMsg, RespMsg>&
	service(const std::string &iTag, const std::string &srvType,
			const Callback_type &cb);

	template<class Msg>
	Publisher_type<Msg>& publisher(const std::string &iTag,
			const std::string &msgType);

	template<class Msg>
	Subscriber_type<Msg>& subscriber(const std::string &iTag,
			const std::string &msgType, const Callback_type &cb);

private:
	void addParameter(const std::string &cTag, const std::string &name,
			const std::string &paramType, const Value_type &value) const;
	void
	configComponent(const std::string &type, const Object_type &component) const;
	void configConnection(const std::string &type, const std::string &iTag1,
			const std::string &iTag2) const;

	const Handler_type &_handler;

	const std::string &_userID;
	const std::string &_robotID;
};

typedef Message_impl<json_spirit::Config> Message;
typedef Client_impl<Message> Client;
typedef Client::Callback_type Callback;
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
typedef mClient::Callback_type mCallback;
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

} /* namespace rce */

#endif /* CLIENT_HXX_ */
