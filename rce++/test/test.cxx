/*
 *    test.cxx
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

#define DEBUG 1

#include <iostream>
#include <boost/bind.hpp>

#include "rce/client.hxx"

class StringMsg
{
public:
	StringMsg(const std::string &data) :
			_data(data)
	{
	}

	StringMsg(const rce::Client::Value &serialized)
	{
		if (serialized.type() != json_spirit::obj_type)
			throw std::runtime_error("Received message has invalid type.");

		rce::Client::Object msg = serialized.get_obj();

		rce::Client::Value data = json_spirit::find_value<rce::Client::Object,
				rce::Client::String>(msg, "data");

		if (data.type() != json_spirit::str_type)
			throw std::runtime_error(
					"Field 'data' of received message has invalid type.");

		_data = data.get_str();
	}

	const rce::Client::Value asJSON()
	{
		rce::Client::Object msg;
		rce::Client::Config::add(msg, "data", "Hello World!");
		return rce::Client::Value(msg);
	}

	const std::string getMessage()
	{
		return _data;
	}

private:
	std::string _data;
};

class Manager
{
public:
	Manager(const std::string url, const std::string userID,
			const std::string robotID);

	void connectCB(rce::ClientPtr client);
	void msgCB(const rce::Client::Value &msg);

	void loop();

private:
	const std::string _robotID;

	rce::ClientPtr _client;
	rce::PublisherPtr _p;
	rce::SubscriberPtr _s;
};

Manager::Manager(const std::string url, const std::string userID,
		const std::string robotID) :
		_robotID(robotID)
{
	std::cout << "Version: " << CLIENT_VERSION << std::endl;
	std::cout << "Create client..." << std::endl;

	// Password of test user is equal to the user name
	_client = boost::make_shared<rce::Client>(userID, userID, robotID);

	std::cout << "Connect to Cloud..." << std::endl;
	_client->connect(url,
			rce::ConnectCallback(boost::bind(&Manager::connectCB, this, _1)));
}

void Manager::msgCB(const rce::Client::Value &msg)
{
	StringMsg str(msg);
	std::cout << "Received: " << str.getMessage() << std::endl;
}

void Manager::connectCB(rce::ClientPtr client)
{
	const std::string MSG_TYPE = "std_msgs/String";
	const std::string REQ = "stringEchoReq";
	const std::string RESP = "stringEchoResp";
	const std::string CONTAINER_ID = "cTag_01";

	client->addInterface(_robotID, REQ, rce::CONVERTER_SUBSCRIBER, MSG_TYPE);
	client->addInterface(_robotID, RESP, rce::CONVERTER_PUBLISHER, MSG_TYPE);
	client->addConnection(CONTAINER_ID, REQ, _robotID, REQ);
	client->addConnection(CONTAINER_ID, RESP, _robotID, RESP);

	_p = client->createPublisher(REQ, MSG_TYPE);
	_s = client->createSubscriber(RESP, MSG_TYPE,
			rce::MsgCallback(boost::bind(&Manager::msgCB, this, _1)));

	loop();
}

void Manager::loop()
{
	int counter = 0;
	char *buf = new char[100];

	while (true)
	{
		sleep(5);
		sprintf(buf, "rce++ Publisher/Subscriber test message (%d)", counter++);
		std::cout << "Send: " << buf << std::endl;

		_p->publish(StringMsg(std::string(buf)).asJSON());
	}

	delete buf;
}

int main(int argc, char **argv)
{
	if (argc != 2)
	{
		std::cout << "Usage: test [RCE IP]" << std::endl;
		return 0;
	}

	std::ostringstream url;
	url << "http://" << argv[1] << ":9000/";

	Manager(url.str(), "testUser", "testRobotCpp");

	std::cout << "leaving main..." << std::endl;
	return 0;
}
