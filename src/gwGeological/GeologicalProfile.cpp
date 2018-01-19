#include "GeologicalProfile.h"

//////////////////////////////////////////////////////////////////////////
LayerProfile::LayerProfile(const std::string & name, const BaseProfileOptions & options)
	: _name(name)
	, _options(options)
{}

const std::string & LayerProfile::getName() const
{
	return _name;
}

void LayerProfile::setVisible(bool isVisible)
{
	if (!_prifileGraph.valid()) return;

	if (_isVisible == isVisible) return;
	_isVisible = isVisible;

	_prifileGraph->setNodeMask(isVisible ? ~0 : 0);
	for (CallbackVector::const_iterator it = _callbacks.begin(); it != _callbacks.end(); ++it)
	{
		(*it)->onVisibleChanged(isVisible);
	}
}

void LayerProfile::addCallback(Callback * cb)
{
	if (!cb) return;
	_callbacks.push_back(cb);
}

void LayerProfile::removeCallback(Callback * cb)
{
	if (!cb) return;
	CallbackVector::iterator it = std::find(_callbacks.begin(), _callbacks.end(), cb);
	if (it == _callbacks.end()) return;
	_callbacks.erase(it);
}
