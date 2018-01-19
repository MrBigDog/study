#ifndef GWGEOLOGICALUTIL_DATE_TIME_H
#define GWGEOLOGICALUTIL_DATE_TIME_H

#include "Common.h"
#include <ctime>
#include <cstring>
#include <string>

namespace gwUtil
{
	/** Basic timestamp (seconds from the epoch) */
	typedef ::time_t TimeStamp;

	/** Time span (in seconds) */
	typedef long TimeSpan;

	/**
	 * General-purpose UTC date/time object.
	 * One second resolution, GMT time zone.
	 */
	class GWGEOLOGICALUTIL_EXPORT DateTime
	{
	public:
		/** DateTime representing "now" */
		DateTime();

		/** DateTime copy */
		DateTime(const DateTime& rhs);

		/** DateTime from a tm (in the local time zone) */
		DateTime(const ::tm& tm);

		/** DateTime from UTC seconds since the epoch */
		DateTime(TimeStamp utc);

		/** DateTime from year, month, date, hours */
		DateTime(int year, int month, int day, double hours);

		/** DateTime from an ISO 8601 string */
		DateTime(const std::string& iso8601);

		/** As a date/time string in RFC 1123 format (e.g., HTTP) */
		const std::string asRFC1123() const;

		/** As a date/time string in ISO 8601 format (lexigraphic order). */
		const std::string asISO8601() const;

		/** As a date/time string in compact ISO 8601 format (lexigraphic
		  * order with no delimiters). */
		const std::string asCompactISO8601() const;

	public:
		int    year()  const;
		int    month() const;
		int    day()   const;
		double hours() const;

		TimeStamp   asTimeStamp() const { return _time_t; }

	protected:
		::tm     _tm;
		::time_t _time_t;

	private:
		// since timegm is not cross-platform
		::time_t timegm(const ::tm* tm) const;
	};

} // namespace osgEarth

#endif // OSGEARTH_DATE_TIME_H
