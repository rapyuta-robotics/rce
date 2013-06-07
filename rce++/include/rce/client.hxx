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

#include <curl/curl.h>
#include <openssl/sha.h>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include "rce/types.hxx"
#include "rce/json_spirit/json_spirit_value.h"
#include "rce/json_spirit/json_spirit_utils.h"
#include "rce/json_spirit/json_spirit_reader_template.h"
#include "rce/json_spirit/json_spirit_writer_template.h"
#include "rce/interfaces.hxx"

#define CLIENT_VERSION "20130415"

typedef typename websocketpp::connection_hdl WebSocketConnectionHandle;
typedef websocketpp::client<websocketpp::config::asio_client> WebSocketClient;
typedef typename WebSocketClient::connection_ptr WebSocketConnectionPointer;
typedef typename WebSocketClient::message_ptr WebSocketMessageHandle;

namespace rce
{

class ClientException: public std::runtime_error
{
public:
	ClientException(const std::string &e) :
			std::runtime_error::runtime_error(e)
	{
	}
};

template<class Config_impl>
class Client_impl: public boost::enable_shared_from_this<
		Client_impl<Config_impl> >
{
public:
	typedef boost::shared_ptr<Client_impl> ClientPtr;

	typedef Config_impl Config;
	typedef typename Config_impl::String_type String;
	typedef typename Config_impl::Value_type Value;
	typedef typename Config_impl::Object_type Object;
	typedef typename Config_impl::Binary_type Binary;
	typedef typename Config_impl::Array_type Array;

	typedef boost::function<void(ClientPtr client)> ConnectCallback;
	typedef boost::function<void(const Value &msg)> MsgCallback;
	typedef boost::function<bool(const Value &req, Value &resp)> ServiceCallback;

	typedef ServiceClient_impl<Client_impl> ServiceClient;
	typedef ServiceProvider_impl<Client_impl> ServiceProvider;
	typedef Publisher_impl<Client_impl> Publisher;
	typedef Subscriber_impl<Client_impl> Subscriber;

	typedef boost::shared_ptr<ServiceClient> ServiceClientPtr;
	typedef boost::shared_ptr<ServiceProvider> ServiceProviderPtr;
	typedef boost::shared_ptr<Publisher> PublisherPtr;
	typedef boost::shared_ptr<Subscriber> SubscriberPtr;

	typedef ReceiverInterface_impl<Client_impl> Interface;

private:
	typedef std::pair<String, Interface*> InterfaceReference;
	typedef std::vector<InterfaceReference> InterfaceReferenceVector;

	typedef std::vector<String> Binaries;
	typedef std::pair<String, Binary> BinaryBlob;
	typedef std::vector<BinaryBlob> BinaryBlobVector;
	typedef std::pair<Object, BinaryBlobVector> PendingMessage;
	typedef std::vector<PendingMessage> PendingMessageVector;

	enum ConnectionState
	{
		NOT_CONNECTED, CONNECTING, CONNECTED
	};

public:
	Client_impl(const String &userID, const String &password,
			const String &robotID);

	void connect(const std::string &url, const ConnectCallback cb);
	void disconnect(void);

	void createContainer(const String &cTag);
	void destroyContainer(const String &cTag);

	void addNode(const String &cTag, const String &nTag, const String &pkg,
			const String &exe);
	void addNode(const String &cTag, const String &nTag, const String &pkg,
			const String &exe, const String &args, const String &name,
			const String &rosNamespace);
	void removeNode(const String &cTag, const String &nTag);

	void addParameter(const String &cTag, const String &name, const int value);
	void addParameter(const String &cTag, const String &name,
			const String &value);
	void addParameter(const String &cTag, const String &name,
			const double value);
	void addParameter(const String &cTag, const String &name, const bool value);
	void removeParameter(const String &cTag, const String &name);

	void addInterface(const String &eTag, const String &iTag,
			const interface_t iType, const String &iCls);
	void addInterface(const String &eTag, const String &iTag,
			const interface_t iType, const String &iCls, const String &addr);
	void removeInterface(const String &eTag, const String &iTag);

	void addConnection(const String &iTag1, const String &iTag2);
	void addConnection(const String &eTag1, const String &iTag1,
			const String &eTag2, const String &iTag2);
	void removeConnection(const String &iTag1, const String &iTag2);
	void removeConnection(const String &eTag1, const String &iTag1,
			const String &eTag2, const String &iTag2);

	PublisherPtr createPublisher(const String &iTag, const String &msgType);
	SubscriberPtr createSubscriber(const String &iTag, const String &msgType,
			const MsgCallback &cb);
	ServiceClientPtr createServiceClient(const String &iTag,
			const String &srvType, const MsgCallback &cb);
	ServiceProviderPtr createServiceProvider(const String &iTag,
			const String &srvType, const ServiceCallback &cb);

	/* Internal Callbacks for Interfaces */
	void _send(const String &tag, const String &type, const Value &msg,
			const String &msgID);
	void _registerInterface(const String &tag, Interface* const interface);
	void _unregisterInterface(const String &tag, Interface* const interface);

private:
	void on_message(WebSocketConnectionHandle hdl, WebSocketMessageHandle msg);
	void on_open(WebSocketConnectionHandle hdl);
	void on_close(WebSocketConnectionHandle hdl);
	void on_fail(WebSocketConnectionHandle hdl);

	void addParameter(const String &cTag, const String &name,
			const Value &value);
	void configComponent(const String &type, const Value &component);
	void configConnection(const String &type, const String &iTag1,
			const String &iTag2);

	void send(const String &type, const Object &data);

	void processMessage(const std::string &msg);
	void processMessage(const Object &msg);
	void processBinary(const std::string &msg);

	void processData(const Object &data);
	void processStatus(const String &data);
	void processError(const String &data);

	ConnectionState state_;

	const String userID_;
	const String password_;
	const String robotID_;

	ConnectCallback cb_;
	WebSocketClient client_;
	WebSocketConnectionHandle handle_;
	boost::thread asio_thread_;
	boost::thread cb_thread_;

	PendingMessageVector pending_;
	boost::mutex interface_mutex_;
	InterfaceReferenceVector interfaces_;

	static bool initialized_;
};

///////////////////////////////////////////////////////////////////////////////
// Typedef's
///////////////////////////////////////////////////////////////////////////////
typedef Client_impl<json_spirit::Config> Client;
typedef Client::ClientPtr ClientPtr;

typedef Client::ConnectCallback ConnectCallback;
typedef Client::MsgCallback MsgCallback;
typedef Client::ServiceClient ServiceClient;
typedef Client::ServiceProvider ServiceProvider;
typedef Client::Publisher Publisher;
typedef Client::Subscriber Subscriber;
typedef Client::ServiceClientPtr ServiceClientPtr;
typedef Client::ServiceProviderPtr ServiceProviderPtr;
typedef Client::PublisherPtr PublisherPtr;
typedef Client::SubscriberPtr SubscriberPtr;

typedef Client_impl<json_spirit::mConfig> mClient;
typedef mClient::ClientPtr mClientPtr;

typedef mClient::ConnectCallback mConnectCallback;
typedef mClient::MsgCallback mMsgCallback;
typedef mClient::ServiceCallback mServiceCallback;

typedef mClient::ServiceClient mServiceClient;
typedef mClient::ServiceProvider mServiceProvider;
typedef mClient::Publisher mPublisher;
typedef mClient::Subscriber mSubscriber;
typedef mClient::ServiceClientPtr mServiceClientPtr;
typedef mClient::ServiceProviderPtr mServiceProviderPtr;
typedef mClient::PublisherPtr mPublisherPtr;
typedef mClient::SubscriberPtr mSubscriberPtr;

///////////////////////////////////////////////////////////////////////////////
// curl memory struct and helper function
///////////////////////////////////////////////////////////////////////////////
struct curlMemory
{
	char *memory;
	size_t size;
};

size_t curlWriteCB(void *contents, size_t size, size_t nmemb, void *userp)
{
	size_t realsize = size * nmemb;
	struct curlMemory *mem = (struct curlMemory*) userp;

	mem->memory = (char*) realloc(mem->memory, mem->size + realsize + 1);

	if (mem->memory == NULL) // out of memory!
		throw ClientException("Not enough memory.");

	memcpy(mem->memory + mem->size, contents, realsize);
	mem->size += realsize;
	mem->memory[mem->size] = 0; // Terminate string with a zero byte

	return realsize;
}

///////////////////////////////////////////////////////////////////////////////
// Client Implementation
///////////////////////////////////////////////////////////////////////////////
template<class Config> bool Client_impl<Config>::initialized_ = false;

template<class Config>
Client_impl<Config>::Client_impl(const String &userID, const String &password,
		const String &robotID) :
		state_(NOT_CONNECTED), userID_(userID), password_(password), robotID_(
				robotID)
{
	if (!initialized_)
	{
		if (curl_global_init(CURL_GLOBAL_NOTHING))
			throw ClientException("Can not initialize the curl library.");

		initialized_ = true;
	}

	client_.init_asio();

	client_.set_open_handler(
			boost::bind(&Client_impl<Config>::on_open, this, _1));
	client_.set_close_handler(
			boost::bind(&Client_impl<Config>::on_close, this, _1));
	client_.set_fail_handler(
			boost::bind(&Client_impl<Config>::on_fail, this, _1));
	client_.set_message_handler(
			boost::bind(&Client_impl<Config>::on_message, this, _1, _2));
}

template<class Config>
void Client_impl<Config>::connect(const std::string &master_url,
		const ConnectCallback cb)
{
	// Declarations
	CURL *curl = NULL;
	curlMemory memory;
	char err[CURL_ERROR_SIZE];
	int rc;
	long httprc;

	unsigned char pw_hash[SHA256_DIGEST_LENGTH];
	SHA256_CTX sha256;
	std::ostringstream pw_stream;

	std::ostringstream masterStream, robotStream;
	char *userID = NULL;
	char *robotID = NULL;
	char *password = NULL;

	websocketpp::lib::error_code ec;
	WebSocketClient::connection_ptr con;

	curl = curl_easy_init();
	memset(&memory, 0, sizeof(curlMemory));

	if (!curl)
		throw ClientException("Can not initialize the curl session.");

	SHA256_Init(&sha256);
	SHA256_Update(&sha256, password_.c_str(), password_.size());
	SHA256_Final(pw_hash, &sha256);

	pw_stream << std::hex << std::setw(2) << std::setfill('0');

	for (unsigned int i = 0; i < SHA256_DIGEST_LENGTH; ++i)
		pw_stream << static_cast<unsigned int>(pw_hash[i]);

	userID = curl_easy_escape(curl, userID_.c_str(), 0);
	robotID = curl_easy_escape(curl, robotID_.c_str(), 0);
	password = curl_easy_escape(curl, pw_stream.str().c_str(), 0);

	if (!userID || !robotID || !password)
	{
		if (userID)
			curl_free(userID);

		if (robotID)
			curl_free(robotID);

		if (password)
			curl_free(password);

		curl_easy_cleanup(curl);
		throw ClientException("Can not escape user ID/robot ID/password.");
	}

	cb_ = cb;

	// Check whether a connection attempt is valid.
	if (state_ == CONNECTING)
		throw ClientException("Already a connection attempt in progress.");

	if (state_ == CONNECTED)
		throw ClientException("Already a valid connection available.");

#ifdef DEBUG
	std::cout << "Start connection with:" << std::endl;
	std::cout << "    userID: " << userID_ << std::endl;
	std::cout << "    password: " << password_ << std::endl;
	std::cout << "    robotID: " << robotID_ << std::endl;
#endif

	state_ = CONNECTING;

	// Make the initial HTTP request to the Master Process
	masterStream << master_url;
	masterStream << "?userID=" << userID;
	masterStream << "&version=" << CLIENT_VERSION;

	// set the options for the next request
	curl_easy_setopt(curl, CURLOPT_URL, masterStream.str().c_str());
	curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, err);
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlWriteCB);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *) &memory);
	curl_easy_setopt(curl, CURLOPT_USERAGENT, "libcurl-agent/1.0");

	// Perform the request
	rc = curl_easy_perform(curl);
	curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httprc);

	if (rc != CURLE_OK)
	{
		std::string msg = "Curl reported an error: ";
		msg = msg.append(std::string(err));

		curl_easy_cleanup(curl);
		curl_free(userID);
		curl_free(robotID);
		curl_free(password);
		if (memory.memory)
			free(memory.memory);
		throw ClientException(msg);
	}

	if (httprc != 200)
	{
		std::string msg = "Received an error from Master Process: ";
		msg = msg.append(std::string(memory.memory));

		curl_easy_cleanup(curl);
		curl_free(userID);
		curl_free(robotID);
		curl_free(password);
		if (memory.memory)
			free(memory.memory);
		throw ClientException(msg);
	}

	// Process the received JSON string
	Value messageVal;
	BinaryBlobVector binaries;

	try
	{
		json_spirit::read_string_or_throw(std::string(memory.memory),
				messageVal, binaries);
	} catch (std::runtime_error &e)
	{
		curl_free(userID);
		curl_free(robotID);
		curl_free(password);
		throw ClientException(e.what());
	}

	if (memory.memory)
		free(memory.memory);
	curl_easy_cleanup(curl);

	if (!binaries.empty())
	{
		curl_free(userID);
		curl_free(robotID);
		curl_free(password);
		throw ClientException(
				"Received message from Master Process contains binary references.");
	}

	if (messageVal.type() != json_spirit::obj_type)
	{
		curl_free(userID);
		curl_free(robotID);
		curl_free(password);
		throw ClientException(
				"Received message from Master Process has invalid JSON format.");
	}

	Object message = messageVal.get_obj();

	Value urlVal = json_spirit::find_value<Object, String>(message, "url");
	Value curVal = json_spirit::find_value<Object, String>(message, "current");

	if (urlVal.type() != json_spirit::str_type)
	{
		curl_free(userID);
		curl_free(robotID);
		curl_free(password);
		throw ClientException(
				"Received message from Master Process has invalid JSON format.");
	}

	if (curVal.type() != json_spirit::null_type)
	{
		if (curVal.type() != json_spirit::str_type)
		{
			curl_free(userID);
			curl_free(robotID);
			curl_free(password);
			throw ClientException(
					"Received message from Master Process has invalid JSON format.");
		}

		std::cout << "There is a newer client version (version: '"
				<< curVal.get_str() << "') available." << std::endl;
	}

#ifdef DEBUG
	std::cout << "Received the following Robot URL from Master Process:" << std::endl;
	std::cout << "    url: " << urlVal.get_str() << std::endl;
#endif

	robotStream << urlVal.get_str();
	robotStream << "?userID=" << userID;
	robotStream << "&robotID=" << robotID;
	robotStream << "&password=" << password;

	curl_free(userID);
	curl_free(robotID);
	curl_free(password);

#ifdef DEBUG
	client_.set_access_channels(websocketpp::log::alevel::all);
	client_.set_error_channels(websocketpp::log::elevel::all);
#else
	client_.clear_access_channels(websocketpp::log::alevel::all);
	client_.clear_error_channels(websocketpp::log::elevel::all);
	client_.set_error_channels(
			websocketpp::log::elevel::rerror | websocketpp::log::elevel::fatal);
#endif

	/* Setup handlers for WebSocket connection */
	client_.set_message_handler(
			boost::bind(&Client_impl::on_message, this, _1, _2));
	client_.set_open_handler(boost::bind(&Client_impl::on_open, this, _1));
	client_.set_close_handler(boost::bind(&Client_impl::on_close, this, _1));
	client_.set_fail_handler(boost::bind(&Client_impl::on_fail, this, _1));

	/* Setup connection */
	con = client_.get_connection(robotStream.str(), ec);
	if (ec)
		throw ClientException("Connection Creation failed: " + ec.message());

	handle_ = con->get_handle();
	client_.connect(con);

	asio_thread_ = boost::thread(boost::bind(&WebSocketClient::run, &client_));
	asio_thread_.join();
	cb_thread_.join();
}

template<class Config>
void Client_impl<Config>::disconnect()
{
	state_ = NOT_CONNECTED;
	client_.stop();
}

template<class Config>
typename Client_impl<Config>::ServiceClientPtr Client_impl<Config>::createServiceClient(
		const String &iTag, const String &srvType, const MsgCallback &cb)
{
	return boost::make_shared<ServiceClient>(this->shared_from_this(), iTag, srvType, cb);
}

template<class Config>
typename Client_impl<Config>::ServiceProviderPtr Client_impl<Config>::createServiceProvider(
		const String &iTag, const String &srvType, const ServiceCallback &cb)
{
	return boost::make_shared<ServiceProvider>(this->shared_from_this(), iTag, srvType, cb);
}

template<class Config>
typename Client_impl<Config>::PublisherPtr Client_impl<Config>::createPublisher(
		const String &iTag, const String &msgType)
{
	return boost::make_shared<Publisher>(this->shared_from_this(), iTag, msgType);
}

template<class Config>
typename Client_impl<Config>::SubscriberPtr Client_impl<Config>::createSubscriber(
		const String &iTag, const String &msgType, const MsgCallback &cb)
{
	return boost::make_shared<Subscriber>(this->shared_from_this(), iTag, msgType, cb);
}

template<class Config>
void Client_impl<Config>::createContainer(const String &cTag)
{
	Object data;
	Config::add(data, "containerTag", cTag);

	send(RCE_CREATE_CONTAINER, data);
}

template<class Config>
void Client_impl<Config>::destroyContainer(const String &cTag)
{
	Object data;
	Config::add(data, "containerTag", cTag);

	send(RCE_DESTROY_CONTAINER, data);
}

template<class Config>
inline void Client_impl<Config>::addNode(const String &cTag, const String &nTag,
		const String &pkg, const String &exe)
{
	addNode(cTag, nTag, pkg, exe, "", "", "");
}

template<class Config>
void Client_impl<Config>::addNode(const String &cTag, const String &nTag,
		const String &pkg, const String &exe, const String &args,
		const String &name, const String &rosNamespace)
{
	Object component;
	Config::add(component, "containerTag", cTag);
	Config::add(component, "nodeTag", nTag);
	Config::add(component, "pkg", pkg);
	Config::add(component, "exe", exe);
	Config::add(component, "args", args);
	Config::add(component, "name", name);
	Config::add(component, "namespace", rosNamespace);

	configComponent("addNodes", Value(component));
}

template<class Config>
void Client_impl<Config>::removeNode(const String &cTag, const String &nTag)
{
	Object component;
	Config::add(component, "containerTag", cTag);
	Config::add(component, "nodeTag", nTag);

	configComponent("removeNodes", Value(component));
}

template<class Config>
inline void Client_impl<Config>::addParameter(const String &cTag,
		const String &name, const int value)
{
	addParameter(cTag, name, Value(value));
}

template<class Config>
inline void Client_impl<Config>::addParameter(const String &cTag,
		const String &name, const String &value)
{
	addParameter(cTag, name, Value(value));
}

template<class Config>
inline void Client_impl<Config>::addParameter(const String &cTag,
		const String &name, const double value)
{
	addParameter(cTag, name, Value(value));
}

template<class Config>
inline void Client_impl<Config>::addParameter(const String &cTag,
		const String &name, const bool value)
{
	addParameter(cTag, name, Value(value));
}

template<class Config>
void Client_impl<Config>::addParameter(const String &cTag, const String &name,
		const Value &value)
{
	Object component;
	Config::add(component, "containerTag", cTag);
	Config::add(component, "name", name);
	Config::add(component, "value", value);

	configComponent("setParam", Value(component));
}

template<class Config>
void Client_impl<Config>::removeParameter(const String &cTag,
		const String &name)
{
	Object component;
	Config::add(component, "containerTag", cTag);
	Config::add(component, "name", name);

	configComponent("deleteParam", Value(component));
}

template<class Config>
inline void Client_impl<Config>::addInterface(const String &eTag,
		const String &iTag, const interface_t iType, const String &iCls)
{
	addInterface(eTag, iTag, iType, iCls, "");
}

template<class Config>
void Client_impl<Config>::addInterface(const String &eTag, const String &iTag,
		const interface_t iType, const String &iCls, const String &addr)
{
	Object component;
	Config::add(component, "endpointTag", eTag);
	Config::add(component, "interfaceTag", iTag);

	switch (iType)
	{
	case INTERFACE_SERVICE_CLIENT:
		Config::add(component, "interfaceType", "ServiceInterface");
		break;

	case INTERFACE_SERVICE_PROVIDER:
		Config::add(component, "interfaceType", "ServiceProviderInterface");
		break;

	case INTERFACE_PUBLISHER:
		Config::add(component, "interfaceType", "PublisherInterface");
		break;

	case INTERFACE_SUBSCRIBER:
		Config::add(component, "interfaceType", "SubscriberInterface");
		break;

	case CONVERTER_SERVICE_CLIENT:
		Config::add(component, "interfaceType", "ServiceConverter");
		break;

	case CONVERTER_SERVICE_PROVIDER:
		Config::add(component, "interfaceType", "ServiceProviderConverter");
		break;

	case CONVERTER_PUBLISHER:
		Config::add(component, "interfaceType", "PublisherConverter");
		break;

	case CONVERTER_SUBSCRIBER:
		Config::add(component, "interfaceType", "SubscriberConverter");
		break;

	case FORWARDER_SERVICE_CLIENT:
		Config::add(component, "interfaceType", "ServiceForwarder");
		break;

	case FORWARDER_SERVICE_PROVIDER:
		Config::add(component, "interfaceType", "ServiceProviderForwarder");
		break;

	case FORWARDER_PUBLISHER:
		Config::add(component, "interfaceType", "PublisherForwarder");
		break;

	case FORWARDER_SUBSCRIBER:
		Config::add(component, "interfaceType", "SubscriberForwarder");
		break;
	}

	Config::add(component, "className", iCls);
	Config::add(component, "addr", addr);

	configComponent(String("addInterfaces"), Value(component));
}

template<class Config>
void Client_impl<Config>::removeInterface(const String &eTag,
		const String &iTag)
{
	Object component;
	Config::add(component, "endpointTag", eTag);
	Config::add(component, "interfaceTag", iTag);

	configComponent("removeInterfaces", Value(component));
}

template<class Config>
void Client_impl<Config>::configComponent(const String &type,
		const Value &component)
{
	Array array;
	array.push_back(component);

	Object data;
	Config::add(data, type, array);

	send(RCE_CONFIGURE_COMPONENT, data);
}

template<class Config>
inline void Client_impl<Config>::addConnection(const String &eTag1,
		const String &iTag1, const String &eTag2, const String &iTag2)
{
	configConnection("connect", eTag1 + "/" + iTag1, eTag2 + "/" + iTag2);
}

template<class Config>
inline void Client_impl<Config>::addConnection(const String &iTag1,
		const String &iTag2)
{
	configConnection("connect", iTag1, iTag2);
}

template<class Config>
inline void Client_impl<Config>::removeConnection(const String &eTag1,
		const String &iTag1, const String &eTag2, const String &iTag2)
{
	configConnection("disconnect", eTag1 + "/" + iTag1, eTag2 + "/" + iTag2);
}

template<class Config>
inline void Client_impl<Config>::removeConnection(const String &iTag1,
		const String &iTag2)
{
	configConnection("disconnect", iTag1, iTag2);
}

template<class Config>
void Client_impl<Config>::configConnection(const String &type,
		const String &iTag1, const String &iTag2)
{
	Object connection;
	Config::add(connection, "tagA", iTag1);
	Config::add(connection, "tagB", iTag2);

	Array array;
	array.push_back(connection);

	Object data;
	Config::add(data, type, array);

	send(RCE_CONFIGURE_CONNECTION, data);
}

template<class Config>
void Client_impl<Config>::_registerInterface(const String &tag,
		Interface* const interface)
{
	typename InterfaceReferenceVector::iterator it;

	{
		boost::mutex::scoped_lock lock(interface_mutex_);

		for (it = interfaces_.begin(); it < interfaces_.end(); it++)
			if (it->first == tag && it->second == interface)
				throw ClientException(
						"Can not register Interface. It is already registered.");

		interfaces_.push_back(InterfaceReference(tag, interface));
	}
}

template<class Config>
void Client_impl<Config>::_unregisterInterface(const String &tag,
		Interface* const interface)
{
	typename InterfaceReferenceVector::iterator it;

	{
		boost::mutex::scoped_lock lock(interface_mutex_);
		for (it = interfaces_.begin(); it < interfaces_.end(); it++)
			if (it->first == tag && it->second == interface)
				break;

		if (it != interfaces_.end())
			interfaces_.erase(it);
		else
			throw ClientException(
					"Can not unregister Interface. It is not registered.");
	}
}

template<class Config>
void Client_impl<Config>::_send(const String &tag, const String &type,
		const Value &msg, const String &msgID)
{
	Object data;
	Config::add(data, "iTag", tag);
	Config::add(data, "type", type);
	Config::add(data, "msgID", msgID);
	Config::add(data, "msg", msg);

	send(RCE_DATA_MESSAGE, data);
}

template<class Config>
void Client_impl<Config>::send(const String &type, const Object &data)
{
	Binaries binaries;
	typename Binaries::iterator it;

	Object message;

	Config::add(message, "type", type);
	Config::add(message, "data", data);

	client_.send(handle_,
			json_spirit::write_string(Value(message), binaries, false),
			websocketpp::frame::opcode::TEXT);

	for (it = binaries.begin(); it != binaries.end(); ++it)
		client_.send(handle_, *it, websocketpp::frame::opcode::BINARY);
}

template<class Config>
void Client_impl<Config>::on_message(WebSocketConnectionHandle hdl,
		WebSocketMessageHandle msg)
{
#ifdef DEBUG
	std::cout << "Message received." << std::endl;
#endif

	switch (msg->get_opcode())
	{
	case websocketpp::frame::opcode::TEXT:
		this->processMessage(msg->get_payload());
		break;

	case websocketpp::frame::opcode::BINARY:
		this->processBinary(msg->get_payload());
		break;

	default:
		throw ClientException(
				"Received message from Cloud Engine has invalid type (websocket).");
	}
}

template<class Config>
void Client_impl<Config>::on_open(WebSocketConnectionHandle hdl)
{
#ifdef DEBUG
	std::cout << "Connection established." << std::endl;
#endif

	state_ = CONNECTED;
	cb_thread_ = boost::thread(cb_, this->shared_from_this());
}

template<class Config>
void Client_impl<Config>::on_close(WebSocketConnectionHandle hdl)
{
#ifdef DEBUG
	std::cout << "Connection closed." << std::endl;
#endif

	state_ = NOT_CONNECTED;
}

template<class Config>
void Client_impl<Config>::on_fail(WebSocketConnectionHandle hdl)
{
#ifdef DEBUG
	std::cout << "Connection failed." << std::endl;
#endif

	state_ = NOT_CONNECTED;

	// TODO: client_.get_con_from_hdl(hdl)->???
}

template<class Config>
void Client_impl<Config>::processMessage(const std::string &msg)
{
	Value message;
	BinaryBlobVector binaries;

	json_spirit::read_string_or_throw(msg, message, binaries);

	if (message.type() != json_spirit::obj_type)
		throw ClientException(
				"Received a message from Cloud Engine with invalid JSON format.");

	if (!binaries.empty())
		pending_.push_back(PendingMessage(message.get_obj(), binaries));
	else
		this->processMessage(message.get_obj());
}

template<class Config>
void Client_impl<Config>::processBinary(const std::string &msg)
{
	if (msg.length() < 32)
		throw ClientException("Received a binary message with invalid header.");

	std::string uid = msg.substr(0, 32);
	std::string binary = msg.substr(32);

	typename PendingMessageVector::iterator msgIt;
	typename BinaryBlobVector::iterator binIt;

	for (msgIt = pending_.begin(); msgIt != pending_.end(); ++msgIt)
	{
		for (binIt = msgIt->second.begin(); binIt != msgIt->second.end();
				++binIt)
		{
			if (binIt->first == uid)
			{
				binIt->second.set_data(binary);
				msgIt->second.erase(binIt);

				if (msgIt->second.empty())
				{
					this->processMessage(msgIt->first);
					pending_.erase(msgIt);
					return;
				}
			}
		}
	}

	throw ClientException("Received binary message was not expected.");
}

template<class Config>
void Client_impl<Config>::processMessage(const Object &msg)
{
	Value type = json_spirit::find_value<Object, String>(msg, "type");
	Value data = json_spirit::find_value<Object, String>(msg, "data");

	if (type.type() != json_spirit::str_type)
		throw ClientException(
				"Received a message from Cloud Engine with invalid type for field 'type'.");

	String dataType = type.get_str();

	if (dataType == RCE_DATA_MESSAGE)
	{
		if (data.type() != json_spirit::obj_type)
			throw ClientException(
					"Received a message from Cloud Engine with invalid type for field 'data'.");

		this->processData(data.get_obj());
	}
	else if (dataType == RCE_STATUS)
	{
		if (data.type() != json_spirit::str_type)
			throw ClientException(
					"Received a message from Cloud Engine with invalid type for field 'data'.");

		this->processStatus(data.get_str());
	}
	else if (dataType == RCE_ERROR)
	{
		if (data.type() != json_spirit::str_type)
			throw ClientException(
					"Received a message from Cloud Engine with invalid type for field 'data'.");

		this->processError(data.get_str());
	}
	else
		throw ClientException(
				"Received a message from Cloud Engine with invalid type: "
						+ dataType);
}

template<class Config>
void Client_impl<Config>::processData(const Object &data)
{
	Value tag = json_spirit::find_value<Object, String>(data, "iTag");
	Value type = json_spirit::find_value<Object, String>(data, "type");
	Value msgID = json_spirit::find_value<Object, String>(data, "msgID");
	Value msg = json_spirit::find_value<Object, String>(data, "msg");

#ifdef DEBUG
	if (tag.type() != json_spirit::str_type)
	std::cout << "Message's interface tag is not a valid string." << std::endl;

	if (type.type() != json_spirit::str_type)
	std::cout << "Message type is not a valid string." << std::endl;

	if (msgID.type() != json_spirit::str_type)
	std::cout << "Message ID is not a valid string." << std::endl;

	if (msg.type() != json_spirit::obj_type && msg.type() != json_spirit::bin_type)
	std::cout << "Message is neither a valid binary nor a valid object." << std::endl;
#endif

	if (tag.type() != json_spirit::str_type
			|| type.type() != json_spirit::str_type
			|| msgID.type() != json_spirit::str_type
			|| (msg.type() != json_spirit::obj_type
					&& msg.type() != json_spirit::bin_type))
		throw ClientException(
				"Received a message from Robot Manager with invalid format.");

	String iTag = tag.get_str();

	{
		boost::mutex::scoped_lock(interface_mutex_);
		typename InterfaceReferenceVector::iterator it;

		for (it = interfaces_.begin(); it < interfaces_.end(); it++)
			if (it->first == iTag)
				it->second->receive(type.get_str(), msg, msgID.get_str());
	}
}

template<class Config>
void Client_impl<Config>::processStatus(const String &data)
{
	std::cout << data << std::endl;
}

template<class Config>
void Client_impl<Config>::processError(const String &data)
{
	std::cout << data << std::endl;
}

} /* namespace rce */

#endif /* CLIENT_HXX_ */
