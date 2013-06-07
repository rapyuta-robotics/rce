/*
 *    interfaces.hxx
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
 *    Copyright 2013 RoboEarth
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

#ifndef INTERFACES_HXX_
#define INTERFACES_HXX_

#include "rce/util.hxx"

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

class InterfaceException: public std::runtime_error
{
public:
	InterfaceException(const std::string &e) :
			std::runtime_error::runtime_error(e)
	{
	}
};

template<class Client>
class InterfaceBase_impl
{
public:
	InterfaceBase_impl(const typename Client::ClientPtr &client,
			const typename Client::String tag,
			const typename Client::String type) :
			client_(client), tag_(tag), type_(type)
	{
	}

protected:
	const typename Client::ClientPtr client_;
	const typename Client::String tag_;
	const typename Client::String type_;
};

template<class Client>
class ReceiverInterface_impl: public InterfaceBase_impl<Client>
{
public:
	ReceiverInterface_impl(const typename Client::ClientPtr &client,
			const typename Client::String tag,
			const typename Client::String srvType) :
			InterfaceBase_impl<Client>(client, tag, srvType)
	{
		this->client_->_registerInterface(this->tag_, this);
	}

	virtual ~ReceiverInterface_impl()
	{
		this->client_->_unregisterInterface(this->tag_, this);
	}

	virtual void receive(const typename Client::String &type,
			const typename Client::Value &msg,
			const typename Client::String &msgID) = 0;
};

template<class Client>
class ServiceClient_impl: public ReceiverInterface_impl<Client>
{
private:
	typedef std::pair<typename Client::String, typename Client::MsgCallback_t> CallbackRef_t;
	typedef std::vector<CallbackRef_t> CallbackRefVector_t;

public:
	ServiceClient_impl(const typename Client::ClientPtr client,
			const typename Client::String tag,
			const typename Client::String srvType,
			const typename Client::MsgCallback_t cb) :
			ReceiverInterface_impl<Client>(client, tag, srvType), cb_(cb)
	{
	}

	/*
	 * Call Service.
	 *
	 * @param cb:	Overwrite default callback for one service call.
	 */
	void call(const typename Client::Value &msg);
	void call(const typename Client::Value &msg,
			const typename Client::MsgCallback_t &cb);

	void receive(const typename Client::String &type,
			const typename Client::Value &msg,
			const typename Client::String &msgID);

private:
	const typename Client::MsgCallback_t cb_;
	CallbackRefVector_t callbacks_;
};

template<class Client>
class ServiceProvider_impl: public ReceiverInterface_impl<Client>
{
public:
	ServiceProvider_impl(const typename Client::ClientPtr client,
			const typename Client::String tag,
			const typename Client::String srvType,
			const typename Client::ServiceCallback_t cb) :
			ReceiverInterface_impl<Client>(client, tag, srvType), cb_(cb)
	{
	}

	void receive(const typename Client::String &type,
			const typename Client::Value &msg,
			const typename Client::String &msgID);

private:
	const typename Client::ServiceCallback_t cb_;
};

template<class Client>
class Publisher_impl: public InterfaceBase_impl<Client>
{
public:
	Publisher_impl(const typename Client::ClientPtr client,
			const typename Client::String tag,
			const typename Client::String msgType) :
			InterfaceBase_impl<Client>(client, tag, msgType)
	{
	}

	/*
	 * Publish message.
	 */
	void publish(const typename Client::Value &msg);
};

template<class Client>
class Subscriber_impl: public ReceiverInterface_impl<Client>
{
public:
	Subscriber_impl(const typename Client::ClientPtr client,
			const typename Client::String tag,
			const typename Client::String msgType,
			const typename Client::MsgCallback cb) :
			ReceiverInterface_impl<Client>(client, tag, msgType), cb_(cb)
	{
	}

	void receive(const typename Client::String &type,
			const typename Client::Value &msg,
			const typename Client::String &msgID);

	void unsubscribe()
	{
		this->client_->_unregisterInterface(this->tag_, this);
	}

private:
	const typename Client::MsgCallback cb_;
};

////////////////////////////////////////////////////////////////////////////////
// ServiceClient Implementation
////////////////////////////////////////////////////////////////////////////////
template<class Client>
inline void ServiceClient_impl<Client>::call(const typename Client::Value &msg)
{
	call(msg, cb_);
}

template<class Client>
void ServiceClient_impl<Client>::call(const typename Client::Value &msg,
		const typename Client::MsgCallback_t &cb)
{
	typename Client::String uid = generateUUID<typename Client::String>();

	callbacks_.push_back(_CallbackRef_t(uid, cb));
	this->client_->send(this->tag_, this->type_, msg, uid);
}

template<class Client>
void ServiceClient_impl<Client>::receive(const typename Client::String &type,
		const typename Client::Value &msg, const typename Client::String &msgID)
{
	if (type != this->type_)
		throw InterfaceException(
				"Received Service response with invalid type.");

	typename CallbackRefVector_t::iterator it;

	for (it = callbacks_.begin(); it != callbacks_.end(); ++it)
		if (it->first == msgID)
		{
			(it->second).callback(msg);
			break;
		}

	if (it != callbacks_.end())
		callbacks_.erase(it);
	else
		throw InterfaceException(
				"Received Service response with invalid message ID.");
}

////////////////////////////////////////////////////////////////////////////////
// ServiceProvider Implementation
////////////////////////////////////////////////////////////////////////////////
template<class Client>
void ServiceProvider_impl<Client>::receive(const typename Client::String &type,
		const typename Client::Value &msg, const typename Client::String &msgID)
{
	if (type != this->type_)
		throw InterfaceException("Received Service request with invalid type.");

	typename Client::Value response;

	// TODO: Run in separate thread
	if (!cb_(msg, response))
	{
		// TODO: Service call failed; do something about it!
		std::cout << "Service call failed." << std::endl;
		return;
	}

	this->client_->send(this->tag_, this->type_, response, msgID);
}

////////////////////////////////////////////////////////////////////////////////
// Publisher Implementation
////////////////////////////////////////////////////////////////////////////////
template<class Client>
void Publisher_impl<Client>::publish(const typename Client::Value &msg)
{
	this->client_->_send(this->tag_, this->type_, msg, "nil");
}

////////////////////////////////////////////////////////////////////////////////
// Subscriber Implementation
////////////////////////////////////////////////////////////////////////////////
template<class Client>
void Subscriber_impl<Client>::receive(const typename Client::String &type,
		const typename Client::Value &msg, const typename Client::String &msgID)
{
	if (type != this->type_)
		throw InterfaceException("Received Message with invalid type.");

	cb_(msg);
}

} /* namespace rce */

#endif /* INTERFACES_HXX_ */
