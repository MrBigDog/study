#include "Notify.h"
#include <string>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cctype>
#include <iomanip>

using namespace gwUtil;
using namespace std;

osg::NotifySeverity gwutil_g_NotifyLevel = osg::NOTICE;

void gwUtil::setNotifyLevel(osg::NotifySeverity severity)
{
	gwUtil::initNotifyLevel();
	gwutil_g_NotifyLevel = severity;
}

osg::NotifySeverity gwUtil::getNotifyLevel()
{
	gwUtil::initNotifyLevel();
	return gwutil_g_NotifyLevel;
}

bool gwUtil::initNotifyLevel()
{
	static bool s_NotifyInit = false;

	if (s_NotifyInit) return true;

	// g_NotifyLevel
	// =============

	gwutil_g_NotifyLevel = osg::NOTICE; // Default value

	char* OSGNOTIFYLEVEL = getenv("OSGEARTH_NOTIFY_LEVEL");
	if (!OSGNOTIFYLEVEL) OSGNOTIFYLEVEL = getenv("OSGEARTHNOTIFYLEVEL");
	if (OSGNOTIFYLEVEL)
	{

		std::string stringOSGNOTIFYLEVEL(OSGNOTIFYLEVEL);

		// Convert to upper case
		for (std::string::iterator i = stringOSGNOTIFYLEVEL.begin();
			i != stringOSGNOTIFYLEVEL.end();
			++i)
		{
			*i = toupper(*i);
		}

		if (stringOSGNOTIFYLEVEL.find("ALWAYS") != std::string::npos)          gwutil_g_NotifyLevel = osg::ALWAYS;
		else if (stringOSGNOTIFYLEVEL.find("FATAL") != std::string::npos)      gwutil_g_NotifyLevel = osg::FATAL;
		else if (stringOSGNOTIFYLEVEL.find("WARN") != std::string::npos)       gwutil_g_NotifyLevel = osg::WARN;
		else if (stringOSGNOTIFYLEVEL.find("NOTICE") != std::string::npos)     gwutil_g_NotifyLevel = osg::NOTICE;
		else if (stringOSGNOTIFYLEVEL.find("DEBUG_INFO") != std::string::npos) gwutil_g_NotifyLevel = osg::DEBUG_INFO;
		else if (stringOSGNOTIFYLEVEL.find("DEBUG_FP") != std::string::npos)   gwutil_g_NotifyLevel = osg::DEBUG_FP;
		else if (stringOSGNOTIFYLEVEL.find("DEBUG") != std::string::npos)      gwutil_g_NotifyLevel = osg::DEBUG_INFO;
		else if (stringOSGNOTIFYLEVEL.find("INFO") != std::string::npos)       gwutil_g_NotifyLevel = osg::INFO;
		else std::cout << "Warning: invalid OSG_NOTIFY_LEVEL set (" << stringOSGNOTIFYLEVEL << ")" << std::endl;

	}

	s_NotifyInit = true;

	return true;

}

bool gwUtil::isNotifyEnabled(osg::NotifySeverity severity)
{
	return severity <= getNotifyLevel();
}

class NullStreamBuffer : public std::streambuf
{
private:

	virtual streamsize xsputn(const char_type*, streamsize n)
	{
		return n;
	}
};

struct NullStream : public std::ostream
{
	NullStream() :
		std::ostream(_nsb = new NullStreamBuffer) {}

	virtual ~NullStream()
	{
		delete rdbuf();
		rdbuf(0);
		//delete _nsb;
	}

	NullStreamBuffer* _nsb;
};

std::ostream& gwUtil::notify(const osg::NotifySeverity severity)
{
	// set up global notify null stream for inline notify
	static NullStream s_NotifyNulStream;

	static bool initialized = false;
	if (!initialized)
	{
		std::cerr << ""; // dummy op to force construction of cerr, before a reference is passed back to calling code.
		std::cout << ""; // dummy op to force construction of cout, before a reference is passed back to calling code.
		initialized = gwUtil::initNotifyLevel();
	}

	if (severity <= gwutil_g_NotifyLevel)
	{
		std::ostream* out = severity <= osg::WARN ? &std::cerr : &std::cout;
		(*out) << std::setprecision(8);
		return *out;
		//if (severity<=osg::WARN) return std::cerr;
		//else return std::cout;
	}
	return s_NotifyNulStream;
}
