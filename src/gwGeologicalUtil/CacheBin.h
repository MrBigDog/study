#ifndef GWGEOLOGICAL_CACHE_BIN_H
#define GWGEOLOGICAL_CACHE_BIN_H 1

#include "Common.h"
#include "Config.h"
#include "IOTypes.h"
#include <osgDB/ReaderWriter>

namespace gwUtil
{
	/**
	 * CacheBin is a names container within a Cache. It allows different
	 * application modules to compartmentalize their data withing a single
	 * cache location.
	 */
	class GWGEOLOGICALUTIL_EXPORT CacheBin : public osg::Referenced
	{
	public:
		/** returned by the getRecordStatus() function */
		enum RecordStatus 
		{
			STATUS_NOT_FOUND,   // record is not in the cache
			STATUS_OK,          // record is in the cache and newer than the test time
			STATUS_EXPIRED      // record is in the cache and older than the test time
		};

	public:
		/**
		 * Constructs a caching bin.
		 * @param binID  Name of this caching bin (unique withing a Cache)
		 * @param driver ReaderWriter that serializes data for this caching bin.
		 */
		CacheBin(const std::string& binID)
			: _binID(binID), _hashKeys(true), _minTime(0) { }

		/** dtor */
		virtual ~CacheBin() { }

		/**
		 * The identifier (unique withing a Cache) of this bin.
		 */
		const std::string& getID() const { return _binID; }

		/**
		 * Whether the implemention should hash record keys instead of using
		 * them directly. Default = false.
		 */
		void setHashKeys(bool value) { _hashKeys = value; }
		bool getHashKeys() const { return _hashKeys; }

		/**
		 * Reads an object from the cache bin.
		 * @param key     Lookup key to read
		 */
		virtual ReadResult readObject(const std::string& key, const osgDB::Options* dbo) = 0;

		/**
		 * Reads an image from the cache bin.
		 * @param key     Lookup key to read
		 */
		virtual ReadResult readImage(const std::string& key, const osgDB::Options* dbo) = 0;

		/**
		 * Reads a string buffer from the cache bin.
		 * @param key    Lookup key to read.
		 */
		virtual ReadResult readString(const std::string& key, const osgDB::Options* dbo) = 0;

		/**
		 * Writes an object (or an image) to the cache bin.
		 * @param key    Lookup key to write to
		 * @param object Object to serialize to the cache
		 */
		virtual bool write(
			const std::string&    key,
			const osg::Object*    object,
			const Config&         metadata,
			const osgDB::Options* dbo) = 0;

		bool write(
			const std::string&    key,
			const osg::Object*    object,
			const osgDB::Options* dbo) {
			return write(key, object, Config(), dbo);
		}

		bool writeNode(
			const std::string&    key,
			osg::Node*            node,
			const Config&         metadata,
			const osgDB::Options* writeOptions);

		/**
		 * Gets the status of a key, i.e. not found, valid or expired.
		 * Pass in a minTime = 0 to simply check whether the record exists.
		 * @param key     Lookup key to check for
		 */
		virtual RecordStatus getRecordStatus(const std::string& key) = 0;

		/**
		 * Purge an entry from the cache bin
		 */
		virtual bool remove(const std::string& key) = 0;

		/**
		 * Update a record's timestamp to "now", as if it were a
		 * new entry.
		 */
		virtual bool touch(const std::string& key) = 0;

		/**
		 * Reads custom metadata from the cache.
		 */
		virtual Config readMetadata() { return Config(); }

		/**
		 * Writes custom metadata to the cache.
		 */
		virtual bool writeMetadata(const Config& meta) { return false; }

		/**
		 * Purges all entries in the cache bin.
		 */
		virtual bool clear() { return false; }

		/**
		 * Compacts the cache bin (where applicable in the implementation,
		 * no-op otherwise)
		 */
		virtual bool compact() { return false; }

		/**
		 * Returns the approximate disk space being used by this cache,
		 * or 0 if the information is unavailable.
		 */
		virtual unsigned getStorageSize() { return 0u; }

		/**
		 * Metadata associated with a cache bin.
		 */
		void setMetadata(osg::Referenced* data) { _metadata = data; }
		osg::Referenced* getMetadata() { return _metadata.get(); }

		/**
		 * If key-hashing is enabled, this method returns the hashed key
		 * corresponding to a raw input key.
		 */
		virtual std::string getHashedKey(const std::string& key) const = 0;

	public: //deprecated

		/** @deprecated - use clear */
		bool purge() { return clear(); } // backwards compatibility


	protected:
		std::string _binID;
		bool        _hashKeys;
		TimeStamp   _minTime;
		osg::ref_ptr<osg::Referenced> _metadata;
	};
}

#endif // GWGEOLOGICAL_CACHE_BIN_H
