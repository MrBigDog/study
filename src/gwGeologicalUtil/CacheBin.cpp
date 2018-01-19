#include "CacheBin.h"
#include "ImageUtils.h"
#include "ThreadingUtils.h"
#include "Registry.h"
#include "Cache.h"
#include "Notify.h"

#include <osgDB/ReaderWriter>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osg/NodeVisitor>
#include <osg/Texture>
#include <osg/Image>
#include <osg/TextureBuffer>

using namespace gwUtil;


// serializer for osg::DummyObject (not present in OSG)
// We need this because the osgDB::DatabasePager will sometimes
// add a DummyObject to textures that it finds in paged objects.
namespace osg
{
	REGISTER_OBJECT_WRAPPER(DummyObject,
		new osg::DummyObject,
		osg::DummyObject,
		"osg::DummyObject")
	{ }
}

namespace
{
#undef  LC
#define LC "[PrepareForCaching] "

	/**
	 * Visitor that preps a scene graph for writing to the cache.
	 *
	 * There are various things that need to happen:
	 *
	 * - Remove any user data containers, since these will not serialize and
	 *   will cause the OSG serializer to fail.
	 *
	 * - Replace texture image filenames to point at objects in the cache.
	 *   Before doing this, however, you need to run the WriteImagesToCache
	 *   visitor.
	 */
	struct PrepareForCaching : public osg::NodeVisitor
	{
		unsigned _textures;         // profiling 
		unsigned _userDataClears;   // profiling

		PrepareForCaching() : osg::NodeVisitor()
		{
			setTraversalMode(TRAVERSE_ALL_CHILDREN);
			setNodeMaskOverride(~0);
			_textures = 0;
			_userDataClears = 0;
		}

		void apply(osg::Node& node)
		{
			apply(node.getStateSet());
			applyUserData(node);
			traverse(node);
		}

		void apply(osg::Geode& geode)
		{
			for (unsigned i = 0; i < geode.getNumDrawables(); ++i)
			{
				apply(geode.getDrawable(i));
			}
			apply(static_cast<osg::Node&>(geode));
		}

		void apply(osg::Drawable* drawable)
		{
			if (!drawable) return;
			apply(drawable->getStateSet());
			applyUserData(*drawable);

			osg::Geometry* geom = drawable->asGeometry();
			if (geom)
				apply(geom);
		}

		void apply(osg::Geometry* geom)
		{
			// This detects any NULL vertex attribute arrays and then populates them.
			// Do this because a NULL VAA will crash the OSG serialization reader (osg 3.4.0)
			osg::Geometry::ArrayList& arrays = geom->getVertexAttribArrayList();
			for (osg::Geometry::ArrayList::iterator i = arrays.begin(); i != arrays.end(); ++i)
			{
				if (i->get() == 0L)
				{
					*i = new osg::FloatArray();
					i->get()->setBinding(osg::Array::BIND_OFF);
				}
			}
		}

		void apply(osg::StateSet* ss)
		{
			if (!ss) return;

			osg::StateSet::AttributeList& a0 = ss->getAttributeList();
			for (osg::StateSet::AttributeList::iterator i = a0.begin(); i != a0.end(); ++i)
			{
				osg::StateAttribute* sa = i->second.first.get();
				applyUserData(*sa);
			}

			// Disable the texture image-unref feature so we can share the resource 
			// across cached tiles.
			osg::StateSet::TextureAttributeList& a = ss->getTextureAttributeList();
			for (osg::StateSet::TextureAttributeList::iterator i = a.begin(); i != a.end(); ++i)
			{
				osg::StateSet::AttributeList& b = *i;
				for (osg::StateSet::AttributeList::iterator j = b.begin(); j != b.end(); ++j)
				{
					osg::StateAttribute* sa = j->second.first.get();
					if (sa)
					{
						osg::Texture* tex = dynamic_cast<osg::Texture*>(sa);
						if (tex)
						{
							tex->setUnRefImageDataAfterApply(false);

#if 0 // took this out in favor of the osg::DummyObject serializer above.

							// OSG's DatabasePager attaches "marker objects" to Textures' UserData when it runs a
							// FindCompileableGLObjectsVisitor. This operation is not thread-safe; it doesn't
							// account for the possibility that the texture may already be in use elsewhere.
							//
							// To prevent a threading violation, and the ensuing crash that reliably occurs
							// in Release mode (but not Debug for whatever reason) we are forced to make a
							// shallow clone of the Texture object and use that for serialization instead of
							// the original, since the original may change in the middle of the process.
							// We then replace the original with our close locally and serialize it safely.
							//
							// This "hack" prevents a crash in OSG 3.4.0 when trying to modify and then write
							// serialize the scene graph containing these shared texture objects.
							// Kudos to Jason B for figuring this one out.
							osg::Texture* texClone = osg::clone(tex, osg::CopyOp::SHALLOW_COPY);
							if (texClone)
							{
								for (unsigned k = 0; k < texClone->getNumImages(); ++k)
								{
									osg::Image* image = texClone->getImage(k);
									if (image)
									{
										applyUserData(*image);
									}
								}

								applyUserData(*texClone);

								j->second.first = texClone;
							}
							else
							{
								GW_WARN << LC << "Texture clone failed.\n";
							}
#endif
						}
						else
						{
							applyUserData(*sa);
						}
						}
					}
				}

			applyUserData(*ss);
			}

		void applyUserData(osg::Object& object)
		{
			if (object.getUserData())
			{
				_userDataClears++;
			}
			object.setUserDataContainer(0L);
		}
		};


#undef  LC
#define LC "[WriteImagesToCache] "

	/**
	 * Traverses a graph, located externally referenced images, and writes
	 * them to the cache using a unique cache key. Then this will change the
	 * image's FileName to point at the cached image instead of the original
	 * source. The caches image key includes the .osgearth_cachebin extension,
	 * which will invoke a pseudoloader that redirects the read to the cache bin.
	 *
	 * When you later go to read from the cache, the CacheBin must
	 * be in the osgDB::Options used to invoke the read.
	 */
	struct WriteExternalReferencesToCache : public gwUtil::TextureAndImageVisitor
	{
		CacheBin*               _bin;
		const osgDB::Options*   _writeOptions;
		static Threading::Mutex _globalMutex;

		// constructor
		WriteExternalReferencesToCache(CacheBin* bin, const osgDB::Options* writeOptions)
			: TextureAndImageVisitor(), _bin(bin), _writeOptions(writeOptions)
		{
			setTraversalMode(TRAVERSE_ALL_CHILDREN);
			setNodeMaskOverride(~0L);
		}

		void apply(osg::Texture& tex)
		{
			if (dynamic_cast<osg::TextureBuffer*>(&tex) != 0L)
			{
				// skip texture buffers, they need no prep and 
				// will be inlined as long as they have a write hint
				// set to STORE_INLINE.
			}
			else
			{
				gwUtil::TextureAndImageVisitor::apply(tex);
			}
		}

		void apply(osg::Image& image)
		{
			std::string path = image.getFileName();
			if (path.empty())
			{
				GW_WARN << LC << "ERROR image with blank filename.\n";
			}

			if (!gwUtil::endsWith(path, ".osgearth_cachebin"))
			{
				// take a plugin-global mutex to avoid two threads altering the image
				// at the same time
				Threading::ScopedMutexLock lock(_globalMutex);

				if (!gwUtil::endsWith(path, ".osgearth_cachebin"))
				{
					// get the hashed key that the cache bin will use to actually write the image,
					// and replace the image filename with it.
					std::string cacheKey = path;
					std::string hashKey = _bin->getHashedKey(cacheKey);

					// Append the pseudoloader suffix so our PL can locate the image in the cache.
					image.setFileName(cacheKey + ".osgearth_cachebin");
					image.setWriteHint(osg::Image::EXTERNAL_FILE);

					// If an object with the same key is already cached, skip it.
					CacheBin::RecordStatus rs = _bin->getRecordStatus(cacheKey);
					if (rs != CacheBin::STATUS_OK)
					{
						// The OSGB serializer won't actually write the image data without this:
						osg::ref_ptr<osgDB::Options> dbo = Registry::cloneOrCreateOptions(_writeOptions);
						dbo->setPluginStringData("WriteImageHint", "IncludeData");

						GW_INFO << LC << "Writing image \"" << image.getFileName() << "\" to the cache\n";

						if (!_bin->write(cacheKey, &image, dbo.get()))
						{
							GW_WARN << LC << "...error, write failed!\n";
						}
					}
					else
					{
						//OE_INFO << LC << "..Image \"" << path << "\" already cached\n";
					}
				}
			}
		}
	};


	Threading::Mutex WriteExternalReferencesToCache::_globalMutex;
	}


bool CacheBin::writeNode(const std::string&    key,
	osg::Node*            node,
	const Config&         metadata,
	const osgDB::Options* writeOptions)
{
	// Preparation step - removes things like UserDataContainers
	PrepareForCaching prep;
	node->accept(prep);

	// Write external refs (like texture images) to the cache bin
	WriteExternalReferencesToCache writeRefs(this, writeOptions);
	node->accept(writeRefs);

	// finally, write the graph to the bin:
	write(key, node, metadata, writeOptions);

	return true;
}


#undef  LC
#define LC "[ReadImageFromCachePseudoLoader] "

namespace
{
	/**
	 * Pseudoloader that looks for anything with an "osgearth_cachebin" extension
	 * and tries to load it from a CacheBin stored in the Options. This is useful
	 * when caching nodes that reference external texture images that are also
	 * stored in the cache bin.
	 *
	 * For this to work, you must change the image filenames in your graph so that
	 * they are in the form "cachekey.osgearth_cachebin". Then the pseudoloader will
	 * intercept the load and load them from the cache. Obviously this requires that
	 * you write both the images and the graph to the same cachebin during the
	 * same operation.
	 */
	struct gwUtilReadImageFromCachePseudoLoader : public osgDB::ReaderWriter
	{
		gwUtilReadImageFromCachePseudoLoader()
		{
			this->supportsExtension("osgearth_cachebin", "osgEarth CacheBin Pseudoloader");
		}

		ReadResult readObject(const std::string& url, const osgDB::Options* readOptions) const
		{
			if (osgDB::getLowerCaseFileExtension(url) != "osgearth_cachebin")
				return ReadResult::FILE_NOT_HANDLED;

			CacheSettings* cacheSettings = CacheSettings::get(readOptions);
			if (!cacheSettings || !cacheSettings->isCacheEnabled() || !cacheSettings->getCacheBin())
			{
				return ReadResult::FILE_NOT_FOUND;
			}

			std::string key = osgDB::getNameLessExtension(url);

			GW_DEBUG << LC << "Reading \"" << key << "\"\n";

			gwUtil::ReadResult rr = cacheSettings->getCacheBin()->readObject(key, readOptions);

			return rr.succeeded() ?
				ReadResult(rr.getObject()) :
				ReadResult::FILE_NOT_FOUND;
		}

		ReadResult readImage(const std::string& url, const osgDB::Options* readOptions) const
		{
			if (osgDB::getLowerCaseFileExtension(url) != "osgearth_cachebin")
				return ReadResult::FILE_NOT_HANDLED;

			CacheSettings* cacheSettings = CacheSettings::get(readOptions);
			if (!cacheSettings || !cacheSettings->isCacheEnabled() || !cacheSettings->getCacheBin())
			{
				GW_DEBUG << LC << "Cache not enabled...!\n";
				return ReadResult::FILE_NOT_FOUND;
			}

			std::string key = osgDB::getNameLessExtension(url);

			GW_DEBUG << LC << "Reading \"" << key << "\"\n";

			gwUtil::ReadResult rr = cacheSettings->getCacheBin()->readImage(key, readOptions);

			return rr.succeeded() ?
				ReadResult(rr.getImage()) :
				ReadResult::FILE_NOT_FOUND;
		}
	};

	REGISTER_OSGPLUGIN(osgearth_cachebin, gwUtilReadImageFromCachePseudoLoader);

}
