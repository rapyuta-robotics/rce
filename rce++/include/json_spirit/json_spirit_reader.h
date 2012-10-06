#ifndef JSON_SPIRIT_READER
#define JSON_SPIRIT_READER

//          Copyright John W. Wilkinson 2007 - 2009.
// Distributed under the MIT License, see accompanying file LICENSE.txt

// json spirit version 4.03

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
# pragma once
#endif

#include "json_spirit_value.h"
#include "json_spirit_error_position.h"
#include <iostream>

namespace json_spirit
{
    // functions to reads a JSON values
    bool read( const std::string& s, Value& value, std::vector< std::pair< std::string, Binary* > > &binaries );
    bool read( std::istream& is,     Value& value, std::vector< std::pair< std::string, Binary* > > &binaries );
    bool read( std::string::const_iterator& begin, std::string::const_iterator end, Value& value, std::vector< std::pair< std::string, Binary* > > &binaries );

    void read_or_throw( const std::string& s, Value& value, std::vector< std::pair< std::string, Binary* > > &binaries );
    void read_or_throw( std::istream& is,     Value& value, std::vector< std::pair< std::string, Binary* > > &binaries );
    void read_or_throw( std::string::const_iterator& begin, std::string::const_iterator end, Value& value, std::vector< std::pair< std::string, Binary* > > &binaries );

#ifndef BOOST_NO_STD_WSTRING

    bool read( const std::wstring& s, wValue& value, std::vector< std::pair< std::wstring, wBinary* > > &binaries );
    bool read( std::wistream&  is,    wValue& value, std::vector< std::pair< std::wstring, wBinary* > > &binaries );
    bool read( std::wstring::const_iterator& begin, std::wstring::const_iterator end, wValue& value, std::vector< std::pair< std::wstring, wBinary* > > &binaries );

    void read_or_throw( const std::wstring& s, wValue& value, std::vector< std::pair< std::wstring, wBinary* > > &binaries );
    void read_or_throw( std::wistream& is,     wValue& value, std::vector< std::pair< std::wstring, wBinary* > > &binaries );
    void read_or_throw( std::wstring::const_iterator& begin, std::wstring::const_iterator end, wValue& value, std::vector< std::pair< std::wstring, wBinary* > > &binaries );

#endif

    bool read( const std::string& s, mValue& value, std::vector< std::pair< std::string, mBinary* > > &binaries );
    bool read( std::istream& is,     mValue& value, std::vector< std::pair< std::string, mBinary* > > &binaries );
    bool read( std::string::const_iterator& begin, std::string::const_iterator end, mValue& value, std::vector< std::pair< std::string, mBinary* > > &binaries );

    void read_or_throw( const std::string& s, mValue& value, std::vector< std::pair< std::string, mBinary* > > &binaries );
    void read_or_throw( std::istream& is,     mValue& value, std::vector< std::pair< std::string, mBinary* > > &binaries );
    void read_or_throw( std::string::const_iterator& begin, std::string::const_iterator end, mValue& value, std::vector< std::pair< std::string, mBinary* > > &binaries );

#ifndef BOOST_NO_STD_WSTRING

    bool read( const std::wstring& s, wmValue& value, std::vector< std::pair< std::wstring, wmBinary* > > &binaries );
    bool read( std::wistream& is,     wmValue& value, std::vector< std::pair< std::wstring, wmBinary* > > &binaries );
    bool read( std::wstring::const_iterator& begin, std::wstring::const_iterator end, wmValue& value, std::vector< std::pair< std::wstring, wmBinary* > > &binaries );

    void read_or_throw( const std::wstring& s, wmValue& value, std::vector< std::pair< std::wstring, wmBinary* > > &binaries );
    void read_or_throw( std::wistream& is,     wmValue& value, std::vector< std::pair< std::wstring, wmBinary* > > &binaries );
    void read_or_throw( std::wstring::const_iterator& begin, std::wstring::const_iterator end, wmValue& value, std::vector< std::pair< std::wstring, wmBinary* > > &binaries );

#endif
}

#endif
