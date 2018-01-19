#ifndef GWGEOLOGICALUTIL_REGISTRY
#define GWGEOLOGICALUTIL_REGISTRY 1

#include "Common.h"
#include "optional.h"
#include "CachePolicy.h"
#include "Containers.h"
#include "ThreadingUtils.h"

#include <OpenThreads/ReentrantMutex>
#include <OpenThreads/ScopedLock>
#include <osg/Referenced>
#include <set>

//#define GDAL_SCOPED_LOCK \
//    OpenThreads::ScopedLock<OpenThreads::ReentrantMutex> _slock( osgEarth::getGDALMutex() )\

namespace osgText
{
	class Font;
}

namespace gwUtil
{
	class Cache;
	//class Capabilities;
	//class Profile;
	//class ShaderFactory;
	//class TaskServiceManager;
	class URIReadCallback;
	//class ColorFilterRegistry;
	//class StateSetCache;
	//class ObjectIndex;
	//class Units;

	/**
	 * Application-wide global repository.
	 */
	class GWGEOLOGICALUTIL_EXPORT Registry : public osg::Referenced
	{
	public:
		/** Access the global Registry singleton. */
		static Registry* instance(bool erase = false);

		/** Gets a default cache object - based on environment settings - unless setDefaultCache
			was called to override it. */
		Cache* getDefaultCache() const;

		/** Sets a default cache that a Map will use if none other is specified. */
		void setDefaultCache(Cache* cache);

		/** The default cache policy (used when no policy is set elsewhere) */
		const optional<CachePolicy>& defaultCachePolicy() const;
		void setDefaultCachePolicy(const CachePolicy& policy);

		/** The override cache policy (overrides all others if set) */
		const optional<CachePolicy>& overrideCachePolicy() const;
		void setOverrideCachePolicy(const CachePolicy& policy);

		/** The default cache driver. */
		void setDefaultCacheDriverName(const std::string& name);
		const std::string& getDefaultCacheDriverName() const;

		/**
		 * Given a CachePolicy, composites in the default and override cache policies
		 * as necessary to create an effective CachePolicy. First it will populate
		 * any unset properties in "cp" with defaults if they are available. Then it
		 * will override any properties in "cp" with overrides that are available.
		 */
		 bool resolveCachePolicy(optional<CachePolicy>& cp) const;

		 /**
		  * Whether the given filename is blacklisted
		  */
		bool isBlacklisted(const std::string &filename);

		/**
		 * Blacklist the given filename
		 */
		void blacklist(const std::string &filename);

		/**
		 * Gets the number of blacklisted filenames
		 */
		unsigned int getNumBlacklistedFilenames();

		/**
		 * Clears the blacklist
		 */
		void clearBlacklist();

		/**
		 * Sets or gets a default system font to use
		 */
		void setDefaultFont(osgText::Font* font);
		osgText::Font* getDefaultFont();

		///**
		// * The graphics hardware capabilities for this platform.
		// */
		//bool hasCapabilities() const;
		//const Capabilities& getCapabilities() const;
		//void setCapabilities(Capabilities* caps);
		//static const Capabilities& capabilities() { return instance()->getCapabilities(); }

		/**
		 * Gets or sets the default shader factory. You can replace the default
		 * shader factory if you want to alter any of osgEarth's baseline shaders
		 * (advanced usage).
		 */
		 //const ShaderFactory* getShaderFactory() const;
		 //void setShaderFactory(ShaderFactory* lib);
		 //static const ShaderFactory* shaderFactory() { return instance()->getShaderFactory(); }

		 ///**
		 // * The default shader generator.
		 // */
		 //ShaderGeneratorProxy getShaderGenerator() const;
		 //void setShaderGenerator(ShaderGenerator* gen);
		 //static ShaderGeneratorProxy shaderGenerator() { return instance()->getShaderGenerator(); }

		 ///**
		 // * Global object index.
		 // */
		 //ObjectIndex* getObjectIndex() const;
		 //static ObjectIndex* objectIndex() { return instance()->getObjectIndex(); }

		 /**
		  * A default StateSetCache to use by any process that uses one.
		  * A StateSetCache assist in stateset sharing across multiple nodes.
		  * Note: A registry-wide SSC is only supported in OSG 3.1.4+. See
		  * the Registry.cpp comments for details.
		  */
		//StateSetCache* getStateSetCache() const;
		//void setStateSetCache(StateSetCache* cache);
		//static StateSetCache* stateSetCache() { return instance()->getStateSetCache(); }

		///**
		// * A shared cache for osg::Program objects created by the shader
		// * composition subsystem (VirtualProgram).
		// */
		//ProgramSharedRepo* getProgramSharedRepo();
		//static ProgramSharedRepo* programSharedRepo() { return instance()->getProgramSharedRepo(); }

		///**
		// * Gets a reference to the global task service manager.
		// */
		//TaskServiceManager* getTaskServiceManager() 
		//{
		//	return _taskServiceManager.get();
		//}

		/**
		 * Generates an instance-wide global unique ID.
		 */
		UID createUID();

		/**
		 * Sets a global read callback for URI objects.
		 */
		void setURIReadCallback(URIReadCallback* callback);

		/**
		 * Gets the global read callback for URI objects.
		 */
		URIReadCallback* getURIReadCallback() const;

		/**
		 * Gets the default set of osgDB::Options to use.
		 */
		const osgDB::Options* getDefaultOptions() const;

		/**
		 * Clones an options structure (fixing the archive caching), or creates
		 * a new one.
		 */
		static osgDB::Options* cloneOrCreateOptions(const osgDB::Options* options = 0L);

		///**
		// * Registers a Units definition.
		// */
		//void registerUnits(const Units* staticInstance);

		//const Units* getUnits(const std::string& name) const;

		///**
		// * The name of the default terrain engine driver
		// */
		//void setDefaultTerrainEngineDriverName(const std::string& name);
		//const std::string& getDefaultTerrainEngineDriverName() const { return _terrainEngineDriver; }

		///**
		// * For debugging - tracks activities in progress.
		// */
		//void startActivity(const std::string& name);
		//void startActivity(const std::string& name, const std::string& text);
		//void endActivity(const std::string& name);
		//void getActivities(std::set<std::string>& output);

		///**
		// * Gets the mime-type corresponding to a given extension.
		// */
		//std::string getMimeTypeForExtension(const std::string& extension);

		///**
		// * Gets the file extension corresponding to a given mime-type.
		// */
		//std::string getExtensionForMimeType(const std::string& mimeType);

		/**
		 * Sets the policy for calling osg::Texture::setUnRefImageDataAfterApply
		 * in the osgEarth terrain engine.
		 */
		optional<bool>& unRefImageDataAfterApply() { return _unRefImageDataAfterApply; }
		const optional<bool>& unRefImageDataAfterApply() const { return _unRefImageDataAfterApply; }

		///**
		// * Adds a texture image unit number that osgEarth should never use.
		// */
		//void setTextureImageUnitOffLimits(int unit);
		//const std::set<int> getOffLimitsTextureImageUnits() const;

		///** General purpose transient data store to support VisitorData in pre-osg 3.4 */
		//TransientUserDataStore& dataStore() { return _dataStore; }
		//const TransientUserDataStore& dataStore() const { return _dataStore; }

	protected:
		virtual ~Registry();
		Registry();

		void destruct();

		////bool _gdal_registered;

		//osg::ref_ptr<const Profile> _global_geodetic_profile;
		//osg::ref_ptr<const Profile> _global_mercator_profile;
		//osg::ref_ptr<const Profile> _spherical_mercator_profile;
		//osg::ref_ptr<const Profile> _cube_profile;

		mutable Threading::Mutex _regMutex;
		int _numGdalMutexGets;

		mutable osg::ref_ptr<Cache>   _defaultCache;
		mutable optional<CachePolicy> _defaultCachePolicy;
		mutable optional<CachePolicy> _overrideCachePolicy;

		mutable bool _overrideCachePolicyInitialized;

		typedef std::set<std::string> StringSet;
		StringSet _blacklistedFilenames;
		Threading::ReadWriteMutex _blacklistMutex;

		//osg::ref_ptr<ShaderFactory> _shaderLib;
		//osg::ref_ptr<ShaderGenerator> _shaderGen;
		//osg::ref_ptr<TaskServiceManager> _taskServiceManager;

		// unique ID generator:
		int                      _uidGen;
		mutable Threading::Mutex _uidGenMutex;

		// system capabilities:
		//osg::ref_ptr< Capabilities > _caps;
		//mutable Threading::Mutex     _capsMutex;
		//void initCapabilities();

		osg::ref_ptr<osgDB::Options> _defaultOptions;

		osg::ref_ptr<URIReadCallback> _uriReadCallback;

		osg::ref_ptr<osgText::Font> _defaultFont;

		//typedef std::vector<const Units*> UnitsVector;
		//UnitsVector                       _unitsVector;
		//mutable Threading::ReadWriteMutex _unitsVectorMutex;

		//osg::ref_ptr<StateSetCache> _stateSetCache;

		//std::string _terrainEngineDriver;

		mutable optional<std::string> _cacheDriver;

		typedef std::pair<std::string, std::string> Activity;
		struct ActivityLess
		{
			bool operator()(const Activity& lhs, const Activity& rhs) const
			{
				return lhs.first < rhs.first;
			}
		};
		std::set<Activity, ActivityLess> _activities;
		mutable Threading::Mutex _activityMutex;

		optional<bool> _unRefImageDataAfterApply;

		//osg::ref_ptr<ObjectIndex> _objectIndex;

		std::set<int> _offLimitsTextureImageUnits;

		//TransientUserDataStore _dataStore;
	};
}


///** Proxy class for automatic registration of Units with the Registry.*/
//struct osgEarthRegisterUnits {
//	osgEarthRegisterUnits(const osgEarth::Units* units) {
//		osgEarth::Registry::instance()->registerUnits(units);
//	}
//};
//#define OSGEARTH_REGISTER_UNITS(NAME,INSTANCE) \
//    static osgEarthRegisterUnits s_osgEarthRegistryUnitsProxy##NAME (INSTANCE)



#endif //OSGEARTH_REGISTRY
