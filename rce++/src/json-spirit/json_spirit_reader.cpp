//          Copyright John W. Wilkinson 2007 - 2009.
// Distributed under the MIT License, see accompanying file LICENSE.txt

// json spirit version 4.03

#include "json_spirit/json_spirit_reader.h"
#include "json_spirit/json_spirit_reader_template.h"

using namespace json_spirit;

bool json_spirit::read( const std::string& s, Value& value, std::vector< std::pair< std::string, Binary* > > &binaries )
{
    return read_string( s, value, binaries );
}

void json_spirit::read_or_throw( const std::string& s, Value& value, std::vector< std::pair< std::string, Binary* > > &binaries )
{
    read_string_or_throw( s, value, binaries );
}

bool json_spirit::read( std::istream& is, Value& value, std::vector< std::pair< std::string, Binary* > > &binaries )
{
    return read_stream( is, value, binaries );
}

void json_spirit::read_or_throw( std::istream& is, Value& value, std::vector< std::pair< std::string, Binary* > > &binaries )
{
    read_stream_or_throw( is, value, binaries );
}

bool json_spirit::read( std::string::const_iterator& begin, std::string::const_iterator end, Value& value, std::vector< std::pair< std::string, Binary* > > &binaries )
{
    return read_range( begin, end, value, binaries );
}

void json_spirit::read_or_throw( std::string::const_iterator& begin, std::string::const_iterator end, Value& value, std::vector< std::pair< std::string, Binary* > > &binaries )
{
    begin = read_range_or_throw( begin, end, value, binaries );
}

#ifndef BOOST_NO_STD_WSTRING

bool json_spirit::read( const std::wstring& s, wValue& value, std::vector< std::pair< std::wstring, wBinary* > > &binaries )
{
    return read_string( s, value, binaries );
}

void json_spirit::read_or_throw( const std::wstring& s, wValue& value, std::vector< std::pair< std::wstring, wBinary* > > &binaries )
{
    read_string_or_throw( s, value, binaries );
}

bool json_spirit::read( std::wistream& is, wValue& value, std::vector< std::pair< std::wstring, wBinary* > > &binaries )
{
    return read_stream( is, value, binaries );
}

void json_spirit::read_or_throw( std::wistream& is, wValue& value, std::vector< std::pair< std::wstring, wBinary* > > &binaries )
{
    read_stream_or_throw( is, value, binaries );
}

bool json_spirit::read( std::wstring::const_iterator& begin, std::wstring::const_iterator end, wValue& value, std::vector< std::pair< std::wstring, wBinary* > > &binaries )
{
    return read_range( begin, end, value, binaries );
}

void json_spirit::read_or_throw( std::wstring::const_iterator& begin, std::wstring::const_iterator end, wValue& value, std::vector< std::pair< std::wstring, wBinary* > > &binaries )
{
    begin = read_range_or_throw( begin, end, value, binaries );
}

#endif

bool json_spirit::read( const std::string& s, mValue& value, std::vector< std::pair< std::string, mBinary* > > &binaries )
{
    return read_string( s, value, binaries );
}

void json_spirit::read_or_throw( const std::string& s, mValue& value, std::vector< std::pair< std::string, mBinary* > > &binaries )
{
    read_string_or_throw( s, value, binaries );
}

bool json_spirit::read( std::istream& is, mValue& value, std::vector< std::pair< std::string, mBinary* > > &binaries )
{
    return read_stream( is, value, binaries );
}

void json_spirit::read_or_throw( std::istream& is, mValue& value, std::vector< std::pair< std::string, mBinary* > > &binaries )
{
    read_stream_or_throw( is, value, binaries );
}

bool json_spirit::read( std::string::const_iterator& begin, std::string::const_iterator end, mValue& value, std::vector< std::pair< std::string, mBinary* > > &binaries )
{
    return read_range( begin, end, value, binaries );
}

void json_spirit::read_or_throw( std::string::const_iterator& begin, std::string::const_iterator end, mValue& value, std::vector< std::pair< std::string, mBinary* > > &binaries )
{
    begin = read_range_or_throw( begin, end, value, binaries );
}

#ifndef BOOST_NO_STD_WSTRING

bool json_spirit::read( const std::wstring& s, wmValue& value, std::vector< std::pair< std::wstring, wmBinary* > > &binaries )
{
    return read_string( s, value, binaries );
}

void json_spirit::read_or_throw( const std::wstring& s, wmValue& value, std::vector< std::pair< std::wstring, wmBinary* > > &binaries )
{
    read_string_or_throw( s, value, binaries );
}

bool json_spirit::read( std::wistream& is, wmValue& value, std::vector< std::pair< std::wstring, wmBinary* > > &binaries )
{
    return read_stream( is, value, binaries );
}

void json_spirit::read_or_throw( std::wistream& is, wmValue& value, std::vector< std::pair< std::wstring, wmBinary* > > &binaries )
{
    read_stream_or_throw( is, value, binaries );
}

bool json_spirit::read( std::wstring::const_iterator& begin, std::wstring::const_iterator end, wmValue& value, std::vector< std::pair< std::wstring, wmBinary* > > &binaries )
{
    return read_range( begin, end, value, binaries );
}

void json_spirit::read_or_throw( std::wstring::const_iterator& begin, std::wstring::const_iterator end, wmValue& value, std::vector< std::pair< std::wstring, wmBinary* > > &binaries )
{
    begin = read_range_or_throw( begin, end, value, binaries );
}

#endif

