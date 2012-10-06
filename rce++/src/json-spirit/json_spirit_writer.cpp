//          Copyright John W. Wilkinson 2007 - 2009.
// Distributed under the MIT License, see accompanying file LICENSE.txt

// json spirit version 4.03

#include "json_spirit/json_spirit_writer.h"
#include "json_spirit/json_spirit_writer_template.h"

void json_spirit::write( const Value& value, std::vector< std::string >& binaries, std::ostream& os )
{
    write_stream( value, os, binaries, false );
}

void json_spirit::write_formatted( const Value& value, std::vector< std::string >& binaries, std::ostream& os )
{
    write_stream( value, os, binaries, true );
}

std::string json_spirit::write( const Value& value, std::vector< std::string >& binaries )
{
    return write_string( value, binaries, false );
}

std::string json_spirit::write_formatted( const Value& value, std::vector< std::string >& binaries )
{
    return write_string( value, binaries, true );
}

#ifndef BOOST_NO_STD_WSTRING

void json_spirit::write( const wValue& value, std::vector< std::wstring >& binaries, std::wostream& os )
{
    write_stream( value, os, binaries, false );
}

void json_spirit::write_formatted( const wValue& value, std::vector< std::wstring >& binaries, std::wostream& os )
{
    write_stream( value, os, binaries, true );
}

std::wstring json_spirit::write( const wValue&  value, std::vector< std::wstring >& binaries )
{
    return write_string( value, binaries, false );
}

std::wstring json_spirit::write_formatted( const wValue&  value, std::vector< std::wstring >& binaries )
{
    return write_string( value, binaries, true );
}

#endif

void json_spirit::write( const mValue& value, std::vector< std::string >& binaries, std::ostream& os )
{
    write_stream( value, os, binaries, false );
}

void json_spirit::write_formatted( const mValue& value, std::vector< std::string >& binaries, std::ostream& os )
{
    write_stream( value, os, binaries, true );
}

std::string json_spirit::write( const mValue& value, std::vector< std::string >& binaries )
{
    return write_string( value, binaries, false );
}

std::string json_spirit::write_formatted( const mValue& value, std::vector< std::string >& binaries )
{
    return write_string( value, binaries, true );
}

#ifndef BOOST_NO_STD_WSTRING

void json_spirit::write( const wmValue& value, std::vector< std::wstring >& binaries, std::wostream& os )
{
    write_stream( value, os, binaries, false );
}

void json_spirit::write_formatted( const wmValue& value, std::vector< std::wstring >& binaries, std::wostream& os )
{
    write_stream( value, os, binaries, true );
}

std::wstring json_spirit::write( const wmValue&  value, std::vector< std::wstring >& binaries )
{
    return write_string( value, binaries, false );
}

std::wstring json_spirit::write_formatted( const wmValue&  value, std::vector< std::wstring >& binaries )
{
    return write_string( value, binaries, true );
}

#endif
