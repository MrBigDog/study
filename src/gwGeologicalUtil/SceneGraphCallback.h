#ifndef GWUTILSCENEGRAPHCALLBACK_H
#define GWUTILSCENEGRAPHCALLBACK_H 1

#include "ThreadingUtils.h"
#include <osg/Node>
#include <osg/PagedLOD>
#include <osg/observer_ptr>

namespace gwUtil
{
	class SceneGraphCallback : public osg::Referenced
	{
	public:
		// Called before a node is added to the live scene graph,
		// possibly from a background pager thread
		virtual void onPreMergeNode(osg::Node*) { }

		// Called after a node is added to the live scene graph
		// in the main/update thread
		virtual void onPostMergeNode(osg::Node*) { }

		// Called after a node is remoevd from the live scene graph
		// in the main/update thread
		virtual void onRemoveNode(osg::Node*) { }
	};
	typedef std::vector<osg::ref_ptr<SceneGraphCallback> > SceneGraphCallbackVector;

	/**
	 * Cotnainer for scene graph callbacks. Typically an object that
	 * supports scene graph callbacks will host an instance of this
	 * object and use it register and fire callbacks at the appropriate times.
	 */
	class GWGEOLOGICALUTIL_EXPORT SceneGraphCallbacks : public osg::Referenced
	{
	public:
		//! Add a new callback
		virtual void add(SceneGraphCallback* cb);

		//! Remove an existing callback
		virtual void remove(SceneGraphCallback* cb);

		//! Invoke all pre-merge callbacks on the provided node
		virtual void firePreMergeNode(osg::Node* node);

		//! Invoke all post-merge callbacks on the provided node
		virtual void firePostMergeNode(osg::Node* node);

		//! Invoke all remove callbacks on the provided node
		virtual void fireRemoveNode(osg::Node* node);

	private:
		SceneGraphCallbackVector _callbacks;
		Threading::Mutex _mutex;
	};

	/**
	 * Extends the osg::PagedLOD class to invoke SceneGraphCallbacks
	 * when nodes are added or removed from the live scene graph.
	 * (Note, this object does not invoke the preMergeNode method.)
	 */
	class GWGEOLOGICALUTIL_EXPORT PagedLODWithSceneGraphCallbacks : public osg::PagedLOD
	{
	public:
		PagedLODWithSceneGraphCallbacks();
		PagedLODWithSceneGraphCallbacks(osg::PagedLOD& plod, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY);
		PagedLODWithSceneGraphCallbacks(SceneGraphCallbacks* host);

		SceneGraphCallbacks* getSceneGraphCallbacks() const;
		void setSceneGraphCallbacks(SceneGraphCallbacks* host);

	public: // osg::Group
		virtual bool addChild(osg::Node* child);
		virtual bool insertChild(unsigned index, osg::Node* child);
		virtual bool replaceChild(osg::Node* origChild, osg::Node* newChild);
		virtual bool removeChild(osg::Node* child);

	private:
		osg::observer_ptr<SceneGraphCallbacks> _host;
	};
}

#endif // SceneGraphCallback_h__
