#include "SceneGraphCallback.h"

namespace gwUtil
{
	void SceneGraphCallbacks::add(SceneGraphCallback* cb)
	{
		if (!cb) return;

		Threading::ScopedMutexLock lock(_mutex);
		_callbacks.push_back(cb);
	}

	void SceneGraphCallbacks::remove(SceneGraphCallback* cb)
	{
		if (!cb) return;

		Threading::ScopedMutexLock lock(_mutex);
		for (SceneGraphCallbackVector::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i)
		{
			if (i->get() == cb)
			{
				_callbacks.erase(i);
				break;
			}
		}
	}

	void SceneGraphCallbacks::firePreMergeNode(osg::Node* node)
	{
		Threading::ScopedMutexLock lock(_mutex);
		for (SceneGraphCallbackVector::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i)
			i->get()->onPreMergeNode(node);
	}

	void SceneGraphCallbacks::firePostMergeNode(osg::Node* node)
	{
		for (SceneGraphCallbackVector::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i)
			i->get()->onPostMergeNode(node);
	}

	void SceneGraphCallbacks::fireRemoveNode(osg::Node* node)
	{
		for (SceneGraphCallbackVector::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i)
			i->get()->onPostMergeNode(node);
	}


	PagedLODWithSceneGraphCallbacks::PagedLODWithSceneGraphCallbacks() {}

	PagedLODWithSceneGraphCallbacks::PagedLODWithSceneGraphCallbacks(osg::PagedLOD& plod, const osg::CopyOp& copyop)
		:osg::PagedLOD(plod, copyop)
	{
	}

	PagedLODWithSceneGraphCallbacks::PagedLODWithSceneGraphCallbacks(SceneGraphCallbacks* host)
		: _host(host)
	{}

	SceneGraphCallbacks* PagedLODWithSceneGraphCallbacks::getSceneGraphCallbacks() const
	{
		return _host.get();
	}

	void PagedLODWithSceneGraphCallbacks::setSceneGraphCallbacks(SceneGraphCallbacks* host)
	{
		_host = host;
	}

	bool PagedLODWithSceneGraphCallbacks::addChild(osg::Node* child)
	{
		bool ok = false;
		if (child)
		{
			ok = osg::PagedLOD::addChild(child);
			osg::ref_ptr<SceneGraphCallbacks> host;
			if (_host.lock(host))
				host->firePostMergeNode(child);
		}
		return ok;
	}

	bool PagedLODWithSceneGraphCallbacks::insertChild(unsigned index, osg::Node* child)
	{
		bool ok = false;
		if (child)
		{
			ok = osg::PagedLOD::insertChild(index, child);
			osg::ref_ptr<SceneGraphCallbacks> host;
			if (_host.lock(host))
				host->firePostMergeNode(child);
		}
		return ok;
	}

	bool PagedLODWithSceneGraphCallbacks::replaceChild(osg::Node* oldChild, osg::Node* newChild)
	{
		bool ok = false;
		if (oldChild && newChild)
		{
			ok = osg::PagedLOD::replaceChild(oldChild, newChild);
			osg::ref_ptr<SceneGraphCallbacks> host;
			if (_host.lock(host))
				host->firePostMergeNode(newChild);
		}
		return ok;
	}

	bool PagedLODWithSceneGraphCallbacks::removeChild(osg::Node* child)
	{
		bool ok = false;
		if (child)
		{
			osg::ref_ptr<osg::Node> node = child;
			ok = osg::PagedLOD::removeChild(child);
			osg::ref_ptr<SceneGraphCallbacks> host;
			if (_host.lock(host))
				host->fireRemoveNode(node.get());
		}
		return ok;
	}
}