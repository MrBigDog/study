#include "CachePolicy.h"
#include "Cache.h"
#include <limits.h>

using namespace gwUtil;

//------------------------------------------------------------------------

//statics
CachePolicy CachePolicy::DEFAULT;
CachePolicy CachePolicy::NO_CACHE(CachePolicy::USAGE_NO_CACHE);
CachePolicy CachePolicy::CACHE_ONLY(CachePolicy::USAGE_CACHE_ONLY);

//------------------------------------------------------------------------

CachePolicy::CachePolicy() :
	_usage(USAGE_READ_WRITE),
	_maxAge(INT_MAX),
	_minTime(0)
{ }

CachePolicy::CachePolicy(const Usage& usage) :
	_usage(usage),
	_maxAge(INT_MAX),
	_minTime(0)
{
	_usage = usage; // explicity set the optional<>
}

CachePolicy::CachePolicy(const Config& conf) :
	_usage(USAGE_READ_WRITE),
	_maxAge(INT_MAX),
	_minTime(0)
{
	fromConfig(conf);
}

CachePolicy::CachePolicy(const CachePolicy& rhs) :
	_usage(rhs._usage),
	_maxAge(rhs._maxAge),
	_minTime(rhs._minTime)
{
	//nop
}

//optional<CachePolicy>
//CachePolicy::get(const osgDB::Options* readOptions)
//{
//    optional<CachePolicy> policy;
//    if (readOptions)
//    {
//        CacheSettings* settings = CacheSettings::get(readOptions);
//        if (settings)
//        {
//            policy = settings->cachePolicy().get();
//        }
//    }
//    return policy;
//}

//void
//CachePolicy::store(osgDB::Options* dbOptions) const
//{
//    if ( dbOptions )
//    {
//        Config conf = getConfig();
//        dbOptions->setPluginStringData( "osgEarth::CachePolicy", conf.toJSON() );
//    }
//}

void CachePolicy::mergeAndOverride(const CachePolicy& rhs)
{
	if (rhs.usage().isSet())
		usage() = rhs.usage().get();

	if (rhs.minTime().isSet())
		minTime() = rhs.minTime().get();

	if (rhs.maxAge().isSet())
		maxAge() = rhs.maxAge().get();
}

void CachePolicy::mergeAndOverride(const optional<CachePolicy>& rhs)
{
	if (rhs.isSet())
	{
		mergeAndOverride(rhs.get());
	}
}

TimeStamp CachePolicy::getMinAcceptTime() const
{
	return
		_minTime.isSet() ? _minTime.value() :
		_maxAge.isSet() ? DateTime().asTimeStamp() - _maxAge.value() :
		0;
}

bool CachePolicy::isExpired(TimeStamp lastModified) const
{
	return lastModified < getMinAcceptTime();
}

bool CachePolicy::operator == (const CachePolicy& rhs) const
{
	return
		(_usage.get() == rhs._usage.get()) &&
		(_maxAge.get() == rhs._maxAge.get()) &&
		(_minTime.get() == rhs._minTime.get());
}

CachePolicy& CachePolicy::operator = (const CachePolicy& rhs)
{
	_usage = optional<Usage>(rhs._usage);
	_maxAge = optional<TimeSpan>(rhs._maxAge);
	_minTime = optional<TimeStamp>(rhs._minTime);

	return *this;
}

std::string CachePolicy::usageString() const
{
	if (_usage == USAGE_READ_WRITE) return "read-write";
	if (_usage == USAGE_READ_ONLY)  return "read-only";
	if (_usage == USAGE_CACHE_ONLY)  return "cache-only";
	if (_usage == USAGE_NO_CACHE)    return "no-cache";
	return "unknown";
}

bool CachePolicy::empty() const
{
	bool isSet = _usage.isSet() || _maxAge.isSet() || _minTime.isSet();
	return !isSet;
}

void CachePolicy::fromConfig(const Config& conf)
{
	conf.getIfSet("usage", "read_write", _usage, USAGE_READ_WRITE);
	conf.getIfSet("usage", "read_only", _usage, USAGE_READ_ONLY);
	conf.getIfSet("usage", "cache_only", _usage, USAGE_CACHE_ONLY);
	conf.getIfSet("usage", "no_cache", _usage, USAGE_NO_CACHE);
	conf.getIfSet("usage", "none", _usage, USAGE_NO_CACHE);
	conf.getIfSet("max_age", _maxAge);
	conf.getIfSet("min_time", _minTime);
}

Config CachePolicy::getConfig() const
{
	Config conf("cache_policy");
	conf.addIfSet("usage", "read_write", _usage, USAGE_READ_WRITE);
	conf.addIfSet("usage", "read_only", _usage, USAGE_READ_ONLY);
	conf.addIfSet("usage", "cache_only", _usage, USAGE_CACHE_ONLY);
	conf.addIfSet("usage", "no_cache", _usage, USAGE_NO_CACHE);
	conf.addIfSet("max_age", _maxAge);
	conf.addIfSet("min_time", _minTime);
	return conf;
}
