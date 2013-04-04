/*
 *    util.hxx
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

#ifndef UTIL_HXX_
#define UTIL_HXX_

#include <sstream>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace rce
{

template<class String>
String generateUUID()
{
	std::basic_ostringstream<typename String::value_type> os;
	typename String::const_pointer uid_raw;
	typename String::value_type uid[32];

	os << boost::uuids::random_generator()();
	uid_raw = os.str().data();

	for (int i = 0, j = 0; i < 36; ++i)
	{
		if (i == 8 || i == 13 || i == 18 || i == 23)
			continue;

		uid[j++] = uid_raw[i];
	}

	return String(uid, 32);
}

} /* namespace rce */

#endif /* UTIL_HXX_ */
