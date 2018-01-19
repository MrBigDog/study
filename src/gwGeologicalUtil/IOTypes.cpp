#include "IOTypes.h"
#include "URI.h"
#include "XmlUtils.h"
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <osgDB/Registry>

using namespace gwUtil;

//------------------------------------------------------------------------

const std::string IOMetadata::CONTENT_TYPE = "Content-Type";

//------------------------------------------------------------------------

StringObject::StringObject() :osg::Object()
{ }

StringObject::~StringObject()
{ }

const std::string& StringObject::getString() const
{
	return _str;
}

void StringObject::setString(const std::string& value)
{
	_str = value;
}

//------------------------------------------------------------------------

URIReadCallback::URIReadCallback()
{ }

URIReadCallback::~URIReadCallback()
{ }

//------------------------------------------------------------------------

/**
 * Registerd "StringObject" with OSG's serialization framework. Basically
 * that means that StringObject instances can be read/written to an .osgb
 * file. We use this for caching string data (XML, JSON files for example).
 */
namespace
{
	REGISTER_OBJECT_WRAPPER(StringObject,
		new gwUtil::StringObject,
		gwUtil::StringObject,
		"gwUtil::StringObject")
	{
		ADD_STRING_SERIALIZER(String, "");  // _str
	}
}

//------------------------------------------------------------------------

/**
 * This is an OSG reader/writer template. We're using this to register
 * readers for "XML" and "JSON" format -- these are just text files, but we
 * have to do this to enable OSG to read them from inside an archive (like
 * a ZIP file). For example, this lets OSG read the tilemap.xml file stores
 * with a TMS repository in a ZIP.
 */

#define STRING_READER_WRITER_SHIM(SUFFIX, EXTENSION, DEF) \
struct osgEarthStringReaderWriter##SUFFIX : public osgDB::ReaderWriter \
{ \
    osgEarthStringReaderWriter##SUFFIX () { \
        supportsExtension( EXTENSION, DEF ); \
    } \
    osgDB::ReaderWriter::ReadResult readObject(const std::string& uri, const osgDB::Options* dbOptions) const { \
        std::string ext = osgDB::getLowerCaseFileExtension( uri ); \
        if ( !acceptsExtension(ext) ) return osgDB::ReaderWriter::ReadResult::FILE_NOT_HANDLED; \
        gwUtil::ReadResult r = URI( uri ).readString( dbOptions ); \
        if ( r.succeeded() ) return r.release<StringObject>(); \
        return osgDB::ReaderWriter::ReadResult::ERROR_IN_READING_FILE; \
    } \
    osgDB::ReaderWriter::ReadResult readObject(std::istream& in, const osgDB::Options* dbOptions ) const { \
        URIContext uriContext( dbOptions ); \
        return new StringObject( Stringify() << in.rdbuf() ); \
    } \
    osgDB::ReaderWriter::WriteResult writeObject( const osg::Object& obj, const std::string& location, const osgDB::Options* dbOptions ) const { \
        std::string ext = osgDB::getLowerCaseFileExtension(location); \
        if ( !acceptsExtension(ext) ) return osgDB::ReaderWriter::WriteResult::FILE_NOT_HANDLED; \
        const StringObject* so = dynamic_cast<const StringObject*>(&obj); \
        if ( !so ) return osgDB::ReaderWriter::WriteResult::FILE_NOT_HANDLED; \
        std::ofstream out(location.c_str()); \
        if ( out.is_open() ) { out << so->getString(); out.close(); return osgDB::ReaderWriter::WriteResult::FILE_SAVED; } \
        return osgDB::ReaderWriter::WriteResult::ERROR_IN_WRITING_FILE; \
    } \
}

STRING_READER_WRITER_SHIM(XML, "xml", "gwUtil XML shim");
REGISTER_OSGPLUGIN(xml, osgEarthStringReaderWriterXML);

STRING_READER_WRITER_SHIM(JSON, "json", "gwUtil JSON shim");
REGISTER_OSGPLUGIN(json, osgEarthStringReaderWriterJSON);
