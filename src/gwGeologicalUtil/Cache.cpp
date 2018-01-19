#include "Cache.h"
#include "Registry.h"
#include "Notify.h"
#include "ThreadingUtils.h"

#include <osg/UserDataContainer>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>

using namespace gwUtil;
using namespace gwUtil::Threading;

#define LC "[Cache] "

CacheOptions::~CacheOptions()
{ }

#define CACHESETTINGS_UDC_NAME "gwUtil.CacheSettings"

CacheSettings::CacheSettings()
{
	setName(CACHESETTINGS_UDC_NAME);
}

CacheSettings::CacheSettings(const CacheSettings& rhs, const osg::CopyOp& copy)
	: osg::Object(rhs, copy)
	, _cache(rhs._cache.get())
	, _policy(rhs._policy)
	, _activeBin(rhs._activeBin.get())
{
	//nop
}

bool CacheSettings::isCacheEnabled() const
{
	return _cache.valid() && _policy->isCacheEnabled();
}

void CacheSettings::integrateCachePolicy(const optional<CachePolicy>& policy)
{
	// integrate the fields that are passed in first:
	if (policy.isSet())
		cachePolicy()->mergeAndOverride(policy);

	// then resolve with global overrides from the registry.
	Registry::instance()->resolveCachePolicy(cachePolicy());
}

void CacheSettings::store(osgDB::Options* readOptions)
{
	if (readOptions)
	{
		osg::UserDataContainer* udc = readOptions->getOrCreateUserDataContainer();
		unsigned index = udc->getUserObjectIndex(CACHESETTINGS_UDC_NAME);
		udc->removeUserObject(index);
		udc->addUserObject(this);
	}
}

CacheSettings* CacheSettings::get(const osgDB::Options* readOptions)
{
	CacheSettings* obj = 0L;
	if (readOptions)
	{
		const osg::UserDataContainer* udc = readOptions->getUserDataContainer();
		if (udc) {
			osg::Object* temp = const_cast<osg::Object*>(udc->getUserObject(CACHESETTINGS_UDC_NAME));
			obj = dynamic_cast<CacheSettings*>(temp);
		}
	}
	return obj;
}

std::string CacheSettings::toString() const
{
	return Stringify()
		<< "cache=" << (_cache.valid() ? _cache->className() : "none")
		<< "; policy=" << _policy->usageString()
		<< "; bin=" << (_activeBin.get() ? "yes" : "no");
}

//------------------------------------------------------------------------

Cache::Cache(const CacheOptions& options) :
	_ok(true),
	_options(options)
{
	//nop
}

Cache::~Cache()
{
}

Cache::Cache(const Cache& rhs, const osg::CopyOp& op) :
	osg::Object(rhs, op)
{
	_ok = rhs._ok;
}

CacheBin* Cache::getBin(const std::string& binID)
{
	osg::ref_ptr<CacheBin> _bin;
	_bin = _bins.get(binID);
	return _bin.get();
}

void Cache::removeBin(CacheBin* bin)
{
	_bins.remove(bin);
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[CacheFactory] "
#define CACHE_OPTIONS_TAG "__gwUtil::CacheOptions"

Cache* CacheFactory::create(const CacheOptions& options)
{
	osg::ref_ptr<Cache> result = 0L;
	GW_DEBUG << LC << "Initializing cache of type \"" << options.getDriver() << "\"" << std::endl;

	if (options.getDriver().empty())
	{
		GW_WARN << LC << "ILLEGAL: no driver set in cache options" << std::endl;
	}
	else if (options.getDriver() == "tms")
	{
		GW_WARN << LC << "Sorry, but TMS caching is no longer supported; try \"filesystem\" instead" << std::endl;
	}
	else // try to load from a plugin
	{
		osg::ref_ptr<osgDB::Options> rwopt = Registry::cloneOrCreateOptions();
		rwopt->setPluginData(CACHE_OPTIONS_TAG, (void*)&options);

		std::string driverExt = std::string(".osgearth_cache_") + options.getDriver();
		osgDB::ReaderWriter::ReadResult rr = osgDB::readObjectFile(driverExt, rwopt.get());
		result = dynamic_cast<Cache*>(rr.getObject());
		if (!result.valid())
		{
			GW_WARN << LC << "Failed to load cache plugin for type \"" << options.getDriver() << "\"" << std::endl;
		}
	}
	return result.release();
}

//------------------------------------------------------------------------

const CacheOptions& CacheDriver::getCacheOptions(const osgDB::ReaderWriter::Options* rwopt) const
{
	static CacheOptions s_default;
	const void* data = rwopt->getPluginData(CACHE_OPTIONS_TAG);
	return data ? *static_cast<const CacheOptions*>(data) : s_default;
}

CacheDriver::~CacheDriver()
{
}

//------------------------------------------------------------------------
