#ifndef GeoSceneStyle_h__
#define GeoSceneStyle_h__

#include "AbstractGML.h"

namespace gmml
{
	class vtkLightCollection;
	class GEOXML_EXPORT  GeoSceneStyle
	{
	public:
		GeoSceneStyle(void);
		~GeoSceneStyle(void);

		double Background[3];
		vtkLightCollection*   Lights;
	};

}

#endif // GeoSceneStyle_h__
