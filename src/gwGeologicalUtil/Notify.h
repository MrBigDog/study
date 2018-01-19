#ifndef GWGEOLOGICALUTIL_NOTIFY_H
#define GWGEOLOGICALUTIL_NOTIFY_H 1

#include "Export.h"
#include <osg/Notify>
#include <osg/Timer>
#include <string>

namespace gwUtil
{
	/** set the notify level, overriding the default or the value set by
	  * the environmental variable OSGNOTIFYLEVEL.
	  */
	extern GWGEOLOGICALUTIL_EXPORT void setNotifyLevel(osg::NotifySeverity severity);

	/** get the notify level. */
	extern GWGEOLOGICALUTIL_EXPORT osg::NotifySeverity getNotifyLevel();

	/** is notification enabled, given the current setNotifyLevel() setting? */
	extern GWGEOLOGICALUTIL_EXPORT bool isNotifyEnabled(osg::NotifySeverity severity);

	/** initialize notify level. */
	extern GWGEOLOGICALUTIL_EXPORT bool initNotifyLevel();

	extern GWGEOLOGICALUTIL_EXPORT std::ostream& notify(const osg::NotifySeverity severity);

	inline std::ostream& notify(void) { return gwUtil::notify(osg::INFO); }
}

#define GW_NOTIFY( X,Y ) if(gwUtil::isNotifyEnabled( X )) gwUtil::notify( X ) << Y
#define GW_FATAL GW_NOTIFY(osg::FATAL,"[gwUtil]* ")
#define GW_WARN GW_NOTIFY(osg::WARN,"[gwUtil]* ")
#define GW_NOTICE GW_NOTIFY(osg::NOTICE,"[gwUtil]  ")
#define GW_INFO GW_NOTIFY(osg::INFO,"[gwUtil]  ")
#define GW_INFO_CONTINUE GW_NOTIFY(osg::INFO, "")
#define GW_DEBUG GW_NOTIFY(osg::DEBUG_INFO,"[gwUtil]  ")
#define GW_NULL if(false) gwUtil::notify(osg::ALWAYS)

#define GW_START_TIMER(VAR) osg::Timer_t VAR##_oe_timer = osg::Timer::instance()->tick()
#define GW_STOP_TIMER(VAR) osg::Timer::instance()->delta_s( VAR##_oe_timer, osg::Timer::instance()->tick() )
#define GW_GET_TIMER(VAR) osg::Timer::instance()->delta_s( VAR##_oe_timer, osg::Timer::instance()->tick() )

#endif // OSGEARTH_NOTIFY_H
