#include "Progress.h"
#include "Notify.h"

using namespace gwUtil;

ProgressCallback::ProgressCallback() :
	osg::Referenced(true),
	_canceled(false),
	_failed(false),
	_needsRetry(false),
	_collectStats(false)
{
	//NOP
}

void ProgressCallback::reportError(const std::string& msg)
{
	_message = msg;
	_failed = true;
}

bool ProgressCallback::reportProgress(double             current,
	double             total,
	unsigned           stage,
	unsigned           numStages,
	const std::string& msg)
{
	return false;
}

double& ProgressCallback::stats(const std::string& key)
{
	Stats::iterator i = _stats.find(key);
	if (i == _stats.end())
	{
		double& value = _stats[key];
		value = 0.0;
		return value;
	}
	return i->second;
}

/******************************************************************************/
ConsoleProgressCallback::ConsoleProgressCallback() :
	ProgressCallback()
{
	//NOP
}

void ConsoleProgressCallback::reportError(const std::string& msg)
{
	ProgressCallback::reportError(msg);
	GW_NOTICE << "Error: " << msg << std::endl;
}

bool ConsoleProgressCallback::reportProgress(double current, double total,
	unsigned stage, unsigned numStages,
	const std::string& msg)
{
	if (total > 0)
	{
		double percentComplete = (current / total) * 100.0;
		GW_NOTICE
			<< "Stage " << (stage + 1) << "/" << numStages
			<< "; completed " << percentComplete << "% " << current << " of " << total
			<< std::endl;
	}
	else
	{
		GW_NOTICE << msg << std::endl;
	}
	return false;
}
