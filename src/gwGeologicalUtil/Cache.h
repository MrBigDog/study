#ifndef GWGEOLOGICALUTIL_CACHE_H
#define GWGEOLOGICALUTIL_CACHE_H 1

#include "Common.h"
#include "CacheBin.h"
#include "CachePolicy.h"
#include "Config.h"
#include "Containers.h"
#include <sys/types.h>
#include <map>

// environment variables
#define OSGEARTH_ENV_CACHE_DRIVER  "OSGEARTH_CACHE_DRIVER"
#define OSGEARTH_ENV_CACHE_PATH    "OSGEARTH_CACHE_PATH"
#define OSGEARTH_ENV_CACHE_ONLY    "OSGEARTH_CACHE_ONLY"
#define OSGEARTH_ENV_NO_CACHE      "OSGEARTH_NO_CACHE"
#define OSGEARTH_ENV_CACHE_MAX_AGE "OSGEARTH_CACHE_MAX_AGE"

namespace gwUtil
{
#define DEFAULT_BIN_ID ""

	class Cache;
	class CachePolicy;

	/**
	 * CacheSettings is an object stored in the osgDB::Options structure that
	 * propagates throughout the osgEarth data model. It represents a Cache state
	 * that includes the actual Cache, the active CacheBin, and a policy.
	 */
	class GWGEOLOGICALUTIL_EXPORT CacheSettings : public osg::Object
	{
	public:
		META_Object(gwUtil, CacheSettings);

		typedef UID ID;

		typedef std::string BinID;

		/** default constructor */
		CacheSettings();

		/** copier */
		CacheSettings(const CacheSettings& rhs, const osg::CopyOp& copy = osg::CopyOp::SHALLOW_COPY);

		/** Whether this object support an active cache. */
		bool isCacheEnabled() const;
		bool isCacheDisabled() const { return !isCacheEnabled(); }

		/** Cache used by these settings */
		Cache* getCache() const { return _cache.get(); }
		void setCache(Cache* cache) { _cache = cache; }

		/** Sets the active cache bin to use under these settings. */
		void setCacheBin(CacheBin* bin) { _activeBin = bin; }
		CacheBin* getCacheBin() { return _activeBin.get(); }

		/** The caching policy in effect for all bins; this starts out the same as
		  * defaultCachePolicy() but you can change it. */
		optional<CachePolicy>& cachePolicy() { return _policy; }
		const optional<CachePolicy>& cachePolicy() const { return _policy; }

		/** Integrates an outside cache policy with the one in these settings. This method
		  * also takes care of global (registry) override policy. */
		void integrateCachePolicy(const optional<CachePolicy>& policy);

		/** Get/Set the settings in a read-options structure. */
		static CacheSettings* get(const osgDB::Options* readOptions);
		void store(osgDB::Options* readOptions);

		/** for debugging */
		std::string toString() const;

	protected:
		virtual ~CacheSettings() { }

		osg::ref_ptr<Cache>      _cache;
		osg::ref_ptr<CacheBin>   _activeBin;
		optional<CachePolicy>    _policy;
		mutable Threading::Mutex _mutex;
	};

	//--------------------------------------------------------------------

	/**
	 * Base class for Cache options.
	 */
	class GWGEOLOGICALUTIL_EXPORT CacheOptions : public DriverConfigOptions
	{
	public:
		CacheOptions(const ConfigOptions& options = ConfigOptions())
			: DriverConfigOptions(options)
		{
			fromConfig(_conf);
		}

		virtual ~CacheOptions();

	public:
		virtual Config getConfig() const
		{
			Config conf = ConfigOptions::getConfig();
			return conf;
		}

		virtual void mergeConfig(const Config& conf)
		{
			ConfigOptions::mergeConfig(conf);
			fromConfig(conf);
		}

	private:
		void fromConfig(const Config& conf)
		{
			//future
		}
	};

	//--------------------------------------------------------------------
	typedef PerObjectRefMap<std::string, CacheBin> ThreadSafeCacheBinMap;

	/**
	 * Cache is a container for local storage of keyed data elements.
	 */
	class GWGEOLOGICALUTIL_EXPORT Cache : public osg::Object
	{
	protected:
		Cache(const CacheOptions& options = CacheOptions());
		Cache(const Cache& rhs, const osg::CopyOp& op = osg::CopyOp::DEEP_COPY_ALL);
		virtual ~Cache();

	public:
		/**
		 * Whether this cache is valid and available for use
		 */
		bool isOK() const { return _ok; }

		/**
		 * Gets a caching bin within this cache.
		 * @param name Name of the caching bin
		 * @param rw   Read/write driver for the bin (can be null)
		 */
		CacheBin* getBin(const std::string& name);

		/**
		 * Gets the default caching bin within this cache. This may or may not
		 * be supported by the implementation, so be sure to check the result
		 * before using it.
		 */
		virtual CacheBin* getOrCreateDefaultBin() { return _defaultBin.get(); }

		/**
		 * Creates (and returns a pointer to) a new Cache Bin, or returns an
		 * already-existing one with that ID.
		 * @param binID Name of the new bin
		 */
		virtual CacheBin* addBin(const std::string& binID) = 0;

		/**
		 * Removes a cache bin from the cache.
		 * @param bin Bin to remove.
		 */
		virtual void removeBin(CacheBin* bin);

		/**
		 * Gets an Options structure representing this cache's configuration.
		 */
		const CacheOptions& getCacheOptions() const { return _options; }

		/**
		 * Gets the (approximate) size of the cache on disk, or zero if
		 * the size cannot be calculated
		 */
		virtual off_t getApproximateSize() const { return 0; }

		/**
		 * Compacts the cache (if applicable).
		 */
		virtual bool compact() { return false; }

		/**
		 * Removes all records in the cache (if possible). This could take
		 * some time to complete.
		 */
		virtual bool clear() { return false; }

	protected:
		bool                   _ok;
		CacheOptions           _options;
		ThreadSafeCacheBinMap  _bins;
		osg::ref_ptr<CacheBin> _defaultBin;
	};

	//----------------------------------------------------------------------

	/**
	 * Base class for a cache driver plugin.
	 */
	class GWGEOLOGICALUTIL_EXPORT CacheDriver : public osgDB::ReaderWriter
	{
	public:
		const CacheOptions& getCacheOptions(const osgDB::ReaderWriter::Options* options) const;

		/** dtor */
		virtual ~CacheDriver();
	};

	//----------------------------------------------------------------------

	/**
	 * Factory class that can load and instantiate a Cache implementation based on the
	 * information in the CacheOptions settings.
	 */
	class GWGEOLOGICALUTIL_EXPORT CacheFactory
	{
	public:
		static Cache* create(const CacheOptions& options);
	};
}

#endif // GWGEOLOGICALUTIL_CACHE_H
