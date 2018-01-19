#include "..\GM_XML\Geo3DProject.h"
#include "..\GMXMLFileIO\Geo3DProjectXMLReader.h"
#include "..\GMXMLFileIO\Geo3DProjectXMLWriter.h"

int main()
{
	Geo3DProjectXMLReader xmlreader;

	gmml::Geo3DProject* pws = NULL;
	if (xmlreader.ReadFile("D:\\Geo3DGml_GIT\\Geo3DML\\data\\geo3dml_test_models\\六面体模型\\规则六面体模型.xml", ""))
	{
		pws = ( gmml::Geo3DProject*)xmlreader.GetGeoProject();
	}
	
	Geo3DProjectXMLWriter project_writer;
	if (pws)
	{
		project_writer.WriteProject(pws, "D:\\Geo3DGml_GIT\\Geo3DML\\data\\geo3dml_test_models\\cubeMode\\aa.xml", "gb2312",1);
	}
	
	return 0;
}

