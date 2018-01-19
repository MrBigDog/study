#if 0
#   include <osg/Texture2D>
#   define SL_SAMPLER "sampler2D"
#   define SL_TEXTURE "texture2D"
#   define CLASS_TEXTURE osg::Texture2D
#   define TEXCOORD_SCALE 1.0f
#else
#   include <osg/TextureRectangle>
#   define SL_SAMPLER "samplerRect"
#   define SL_TEXTURE "textureRect"
#   define CLASS_TEXTURE osg::TextureRectangle
#   define TEXCOORD_SCALE 1024.0f
#endif

#include <osg/Quat>
#include <osg/Depth>
#include <osg/NodeVisitor>
#include <osg/StencilTwoSided>
#include <osg/MatrixTransform>
#include <osg/ComputeBoundsVisitor>
#include <osg/PositionAttitudeTransform>
#include <osg/Material>
#include <osg/CullFace>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgGA/TrackballManipulator>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>



static const char* mrtVertSource = {
	"uniform mat4 osg_ViewMatrix;\n"
	"uniform mat4 osg_ViewMatrixInverse;\n"
	"varying vec3 worldNormal;\n"
	"varying vec3 worldView;\n"
	"void main(void)\n"
	"{\n"
	"   worldNormal = vec3(gl_ModelViewMatrixInverseTranspose * vec4(gl_Normal,0.0));\n"
	"   worldNormal = normalize(worldNormal);\n"
	"   worldView = vec3(osg_ViewMatrixInverse * gl_ModelViewMatrix * gl_Vertex);\n"
	"   worldView = (osg_ViewMatrix[3].xyz / osg_ViewMatrix[3].w) - worldView;\n"
	"   worldView = normalize(worldView);\n"
	"   gl_Position = ftransform();\n"
	"   gl_TexCoord[0] = gl_MultiTexCoord0;\n"
	"}\n"
};

static const char* mrtFragSource = {
	"uniform sampler2D defaultTex;\n"
	"varying vec3 worldNormal;\n"
	"varying vec3 worldView;\n"
	"void main(void)\n"
	"{\n"
	"   gl_FragData[0] = texture2D(defaultTex, gl_TexCoord[0].xy);\n"
	"   gl_FragData[1] = vec4(worldNormal, 0.0);\n"
	"   gl_FragData[2] = vec4(worldView, 0.0);\n"
	"}\n"
};

static const char* finalVertSource = {
	"void main(void)\n"
	"{\n"
	"   gl_Position = ftransform();\n"
	"   gl_TexCoord[0] = gl_MultiTexCoord0;\n"
	"}\n"
};

static const char* finalFragSource =
{
	"uniform samplerRect colorTex;\n"
	"uniform samplerRect normalTex;\n"
	"uniform samplerRect viewTex;\n"
	"void main(void)\n"
	"{\n"
	"   vec2 uv = gl_TexCoord[0].xy;\n"
	"   vec3 color   = textureRect(colorTex, uv).xyz;\n"
	"   vec3 normal  = textureRect(normalTex, uv).xyz;\n"
	"   vec3 viewDir = textureRect(viewTex, uv).xyz;\n"
	"   vec3 lightDir = vec3(0.7, -0.7, -0.7);\n"
	"   lightDir = normalize(-lightDir);\n"

	"   vec3 halfDir = normalize(viewDir + lightDir);\n"
	"   float LdotN = dot(lightDir, normal);\n"
	"   float HdotN = dot(halfDir, normal);\n"
	"   float diffuse = LdotN<0.0 ? 0.0 : LdotN;\n"
	"   float specular = (LdotN<0.0 || HdotN<0.0) ? 0.0 : pow(HdotN, 30.0);\n"

	"   vec3 finalColor = vec3(1.0) * 0.4 * diffuse * specular;\n"
	"   finalColor += color * (diffuse + 0.07);\n"
	"   gl_FragColor = vec4(finalColor, 1.0);\n"
	"}\n"
};

osg::Geode* createScreenQuad(float width, float height, float scale = 1.0f)
{
	osg::Geometry* geom = osg::createTexturedQuadGeometry(
		osg::Vec3(), osg::Vec3(width, 0.0f, 0.0f), osg::Vec3(0.0f, height, 0.0f),
		0.0f, 0.0f, width*scale, height*scale);
	osg::ref_ptr<osg::Geode> quad = new osg::Geode;
	quad->addDrawable(geom);

	int values = osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED;
	quad->getOrCreateStateSet()->setAttribute(
		new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL), values);
	quad->getOrCreateStateSet()->setMode(GL_LIGHTING, values);
	return quad.release();
}

osg::Camera* createRTTCamera(osg::Camera::BufferComponent buffer, osg::Texture* tex, bool isAbsolute = false)
{
	osg::ref_ptr<osg::Camera> camera = new osg::Camera;
	camera->setClearColor(osg::Vec4());
	camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	camera->setRenderOrder(osg::Camera::PRE_RENDER);
	if (tex)
	{
		tex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
		tex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
		camera->setViewport(0, 0, tex->getTextureWidth(), tex->getTextureHeight());
		camera->attach(buffer, tex);
	}

	if (isAbsolute)
	{
		camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
		camera->setProjectionMatrix(osg::Matrix::ortho2D(0.0, 1.0, 0.0, 1.0));
		camera->setViewMatrix(osg::Matrix::identity());
		camera->addChild(createScreenQuad(1.0f, 1.0f));
	}
	return camera.release();
}

osg::Camera* createHUDCamera(double left, double right, double bottom, double top)
{
	osg::ref_ptr<osg::Camera> camera = new osg::Camera;
	camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	camera->setClearMask(GL_DEPTH_BUFFER_BIT);
	camera->setRenderOrder(osg::Camera::POST_RENDER);
	camera->setAllowEventFocus(false);
	camera->setProjectionMatrix(osg::Matrix::ortho2D(left, right, bottom, top));
	camera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	return camera.release();
}

class NormalVisitor :public osg::NodeVisitor
{
public:
	NormalVisitor() :osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
	{ }

	virtual void apply(osg::Geode& geode)
	{
		for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
		{
			osg::Geometry* geom = dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
			if (!geom) continue;

			osg::Material* mt = dynamic_cast<osg::Material*>(geom->getStateSet()->getAttribute(osg::StateAttribute::MATERIAL));
			if (mt)
			{
				geom->getStateSet()->removeAttribute(mt);
			}

			osg::ref_ptr <osg::Vec4Array> ca = new osg::Vec4Array;
			ca->push_back(osg::Vec4(1, 1, 1, 1));
			geom->setColorArray(ca, osg::Array::BIND_OVERALL);
			geom->setUseDisplayList(false);
			geom->setUseVertexBufferObjects(true);

			osg::ref_ptr<osg::CullFace> cullface = new osg::CullFace(osg::CullFace::FRONT);
			geom->getStateSet()->setAttribute(cullface.get());
			geom->getStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::ON);
		}
	}

private:
	osg::Vec4 _color;
};
//
//class GeologicalDirectoryVisitor
//{
//public:
//	GeologicalDirectoryVisitor::GeologicalDirectoryVisitor(osg::Group* root)
//		: _root(root)
//	{ }
//
//public:
//	void GeologicalDirectoryVisitor::traverse(const std::string& path)
//	{
//		if (osgDB::fileType(path) == osgDB::DIRECTORY)
//		{
//			if (handleDir(path))
//			{
//				osgDB::DirectoryContents files = osgDB::getDirectoryContents(path);
//				for (osgDB::DirectoryContents::const_iterator f = files.begin(); f != files.end(); ++f)
//				{
//					if (f->compare(".") == 0 || f->compare("..") == 0)
//						continue;
//
//					std::string filepath = osgDB::concatPaths(path, *f);
//					traverse(filepath);
//				}
//			}
//		}
//		else if (osgDB::fileType(path) == osgDB::REGULAR_FILE)
//		{
//			handleFile(path);
//		}
//	}
//
//private:
//	void GeologicalDirectoryVisitor::handleFile(const std::string& filename)
//	{
//		std::string ext = osgDB::getLowerCaseFileExtension(filename);
//#if 0
//		if (ext != "ive") return;
//
//		osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(filename);
//		if (!node) return;
//
//		std::string simplename = osgDB::getSimpleFileName(filename);
//
//		osg::ref_ptr<osg::MatrixTransform> mat = new osg::MatrixTransform;
//		mat->addChild(node);
//		mat->setMatrix(osg::Matrixd::translate(osg::Vec3d(412000.0, 3369000.0, 400)));
//
//		NormalVisitor vbov;
//		node->accept(vbov);
//
//
//
//		std::string sname = osgDB::getNameLessExtension(filename);
//		osgDB::writeNodeFile(*mat, sname + ".osgb", new osgDB::Options("WriteImageHint=IncludeData Compressor=zlib"));
//
//		_root->addChild(mat);
//#else
//		if (ext != "osgb") return;
//		osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(filename);
//		if (!node) return;
//		_root->addChild(node);
//#endif
//	}
//
//	bool GeologicalDirectoryVisitor::handleDir(const std::string& path)
//	{
//		return true;
//	}
//
//private:
//	osg::ref_ptr<osg::Group> _root;
//};




osg::Texture* createFloatTexture()
{
	osg::ref_ptr<CLASS_TEXTURE> texture = new CLASS_TEXTURE;
	texture->setTextureSize(1024, 1024);
	texture->setInternalFormat(GL_RGBA16F_ARB);
	texture->setSourceFormat(GL_RGBA);
	texture->setSourceType(GL_FLOAT);
	return texture.release();
}








int main()
{
	osg::ref_ptr<osg::Group> scene = new osg::Group;

	//GeologicalDirectoryVisitor gdv(scene);
	//gdv.traverse("E:/DATA/swnew");

	//osg::ref_ptr<osg::CullFace> cullface = new osg::CullFace(osg::CullFace::FRONT);
	//scene->getOrCreateStateSet()->setAttribute(cullface.get());
	//scene->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::ON);

	//osg::Image* image = osgDB::readImageFile("E:/form.png");
	//osgDB::writeNodeFile(*osg::createGeodeForImage(image),"pngTest.osgb");

	//std::string filename = "E:/DATA/swnew/j3s.ive";
	//osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(filename);

	//osg::ref_ptr<osg::MatrixTransform> mat = new osg::MatrixTransform;
	//mat->addChild(node);
	//mat->setMatrix(osg::Matrixd::translate(osg::Vec3d(412000.0, 3369000.0, 400)));

	//NormalVisitor vbov(/*ca->at(11)*/osg::Vec4(1, 1, 1, 1));
	//node->accept(vbov);

	//root->addChild(mat);

	//std::string sname = osgDB::getNameLessExtension(filename);
	//osgDB::writeNodeFile(*mat, sname + ".osgb", new osgDB::Options("WriteImageHint=IncludeData Compressor=zlib"));

	//{
	osg::Texture* colorTex = createFloatTexture();
	osg::Texture* normalTex = createFloatTexture();
	osg::Texture* viewTex = createFloatTexture();

	osg::ref_ptr<osg::Camera> rttCamera = createRTTCamera(osg::Camera::COLOR_BUFFER0, colorTex);
	rttCamera->attach(osg::Camera::COLOR_BUFFER1, normalTex);
	rttCamera->attach(osg::Camera::COLOR_BUFFER2, viewTex);
	rttCamera->addChild(scene.get());

	osg::ref_ptr<osg::Program> mrtProg = new osg::Program;
	mrtProg->addShader(new osg::Shader(osg::Shader::VERTEX, mrtVertSource));
	mrtProg->addShader(new osg::Shader(osg::Shader::FRAGMENT, mrtFragSource));
	rttCamera->getOrCreateStateSet()->setAttributeAndModes(
		mrtProg.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	rttCamera->getOrCreateStateSet()->addUniform(new osg::Uniform("defaultTex", 0));

	osg::ref_ptr<osg::Camera> hudCamera = createHUDCamera(0.0, 1.0, 0.0, 1.0);
	hudCamera->addChild(createScreenQuad(1.0f, 1.0f, TEXCOORD_SCALE));

	osg::ref_ptr<osg::Program> finalProg = new osg::Program;
	finalProg->addShader(new osg::Shader(osg::Shader::VERTEX, finalVertSource));
	finalProg->addShader(new osg::Shader(osg::Shader::FRAGMENT, finalFragSource));

	osg::StateSet* ss = hudCamera->getOrCreateStateSet();
	ss->setTextureAttributeAndModes(0, colorTex);
	ss->setTextureAttributeAndModes(1, normalTex);
	ss->setTextureAttributeAndModes(2, viewTex);
	ss->setAttributeAndModes(finalProg.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	ss->addUniform(new osg::Uniform("colorTex", 0));
	ss->addUniform(new osg::Uniform("normalTex", 1));
	ss->addUniform(new osg::Uniform("viewTex", 2));

	osg::ref_ptr<osg::Group> root = new osg::Group;
	root->addChild(rttCamera.get());
	root->addChild(hudCamera.get());
	//}
	//root->addChild(scene.get());

	osgViewer::Viewer viewer;
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
	viewer.addEventHandler(new osgViewer::StatsHandler());
	viewer.getDatabasePager()->setTargetMaximumNumberOfPageLOD(1);
	viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy(true, true);
	viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
	viewer.setCameraManipulator(new osgGA::TrackballManipulator());
	viewer.setSceneData(root);
	viewer.setUpViewInWindow(20, 20, 1000, 800);
	viewer.realize();

	while (!viewer.done())
	{
		viewer.frame();
	}
	viewer.run();

	return 0;
}