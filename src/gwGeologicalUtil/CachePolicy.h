#ifndef GWGEOLOGICAL_CACHE_POLICY_H
#define GWGEOLOGICAL_CACHE_POLICY_H 1

#include "Common.h"
#include "Config.h"
#include "DateTime.h"
#include <osgDB/Options>

namespace gwUtil
{
	/**
	 * Policy for cache usage.
	 */
	class GWGEOLOGICALUTIL_EXPORT CachePolicy
	{
	public:
		enum Usage
		{
			USAGE_READ_WRITE = 0,  // read/write to the cache if one exists.
			USAGE_CACHE_ONLY = 1,  // treat the cache as the ONLY source of data.
			USAGE_READ_ONLY = 2,  // read from the cache, but don't write new data to it.
			USAGE_NO_CACHE = 3   // neither read from or write to the cache
		};

		/** default cache policy (READ_WRITE) */
		static CachePolicy DEFAULT;

		/* policy indicating to never use a cache */
		static CachePolicy NO_CACHE;

		/** policy indicating to only use a cache */
		static CachePolicy CACHE_ONLY;

	public:
		/** constructs an invalid CachePolicy. */
		CachePolicy();

		/** copy ctor */
		CachePolicy(const CachePolicy& rhs);

		/** constructs a caching policy. */
		CachePolicy(const Usage& usage);

		/** constructs a CachePolicy from a config options */
		CachePolicy(const Config& conf);

		/** construct a cache policy be reading it from an osgDB::Options */
		//static optional<CachePolicy> get(const osgDB::Options* options);

		/** Stores this cache policy in a DB Options. */
		//void store( osgDB::Options* options ) const;

		/** Merges any set properties in another CP into this one, override existing values. */
		void mergeAndOverride(const CachePolicy& rhs);
		void mergeAndOverride(const optional<CachePolicy>& rhs);

		/** Gets the oldest timestamp for which to accept a cache record */
		TimeStamp getMinAcceptTime() const;

		/**
		 * Determine whether the given timestamp is considered to be expired based on this CachePolicy
		 */
		bool isExpired(TimeStamp lastModified) const;

		/** dtor */
		virtual ~CachePolicy() { }

		/** Gets the usage policy */
		optional<Usage>& usage() { return _usage; }
		const optional<Usage>& usage() const { return _usage; }

		/** Gets the age limit for a cache record (in seconds) */
		optional<TimeSpan>& maxAge() { return _maxAge; }
		const optional<TimeSpan>& maxAge() const { return _maxAge; }

		/** Gets the age limit for a cache record (as an absolute timestamp) */
		optional<TimeStamp>& minTime() { return _minTime; }
		const optional<TimeStamp>& minTime() const { return _minTime; }

		/** Whether any of the fields are set */
		bool empty() const;

	public: // convenience functions.

		bool isCacheEnabled() const {
			return isCacheReadable() || isCacheWriteable();
		}

		bool isCacheDisabled() const {
			return !isCacheEnabled();
		}

		bool isCacheReadable() const {
			return *_usage == USAGE_READ_WRITE || *_usage == USAGE_CACHE_ONLY || *_usage == USAGE_READ_ONLY;
		}

		bool isCacheWriteable() const {
			return *_usage == USAGE_READ_WRITE;
		}

		bool isCacheOnly() const {
			return *_usage == USAGE_CACHE_ONLY;
		}

		bool operator == (const CachePolicy& rhs) const;

		bool operator != (const CachePolicy& rhs) const {
			return !operator==(rhs);
		}

		CachePolicy& operator = (const CachePolicy& rhs);

		// returns a readable string describing usage
		std::string usageString() const;

	public: // config
		Config getConfig() const;
		void fromConfig(const Config& conf);

	private:
		optional<Usage>     _usage;
		optional<TimeSpan>  _maxAge;
		optional<TimeStamp> _minTime;
	};
}

#endif // OSGEARTH_CACHE_POLICY_H
