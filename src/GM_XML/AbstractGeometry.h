#ifndef AbstractGeometry_h__
#define AbstractGeometry_h__

#include "AbstractGML.h"

namespace gml
{
	class GEOXML_EXPORT AbstractGeometry : public AbstractGML
	{
	public:
		AbstractGeometry(void);
		~AbstractGeometry(void);
	};

}
#endif // AbstractGeometry_h__