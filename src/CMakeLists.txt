FOREACH( lib
		gwGeological
		gwGeologicalUtil
		)

    ADD_SUBDIRECTORY(${lib})
    SET_PROPERTY(TARGET ${lib} PROPERTY FOLDER "Core")
ENDFOREACH( lib )

#SDK
FOREACH( geo3dml_sdk
		 GM_XML
		 vtklibxml2
		 vtkExtending
		 GMLFeature
		 GMXMLFileIO
		 )
ADD_SUBDIRECTORY(${geo3dml_sdk})
SET_PROPERTY(TARGET ${geo3dml_sdk} PROPERTY FOLDER "geo3dml_sdk")
ENDFOREACH( geo3dml_sdk )

ADD_SUBDIRECTORY( examples )
ADD_SUBDIRECTORY( tools )
ADD_SUBDIRECTORY( gwPlugins )