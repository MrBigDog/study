#include <osg/Geode>
#include <osg/Material>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>
#include <osg/ShapeDrawable>
#include <osgAnimation/EaseMotion>
#include <osgAnimation/UpdateMaterial>
#include <osgAnimation/BasicAnimationManager>
#include <osgDB/ReadFile>


class MaterialCallback : public osg::StateAttributeCallback
{
public:
	MaterialCallback()
	{
		_motion = new osgAnimation::InOutCubicMotion(0, 1, 1, osgAnimation::Motion::LOOP);
	}

	virtual void operator() (osg::StateAttribute* attr, osg::NodeVisitor* nv)
	{
		osg::Material* m = dynamic_cast<osg::Material*>(attr);
		if (!m) return;

		_motion->update(0.025);
		float rat = _motion->getValue();

		osg::Vec4 color = osg::Vec4(1, 1, 1, 1) + (osg::Vec4(1, 0, 0, 1) - osg::Vec4(1, 1, 1, 1))*rat;
		m->setDiffuse(m->FRONT, color);

		for (unsigned int i = 0; i < attr->getNumParents(); i++)
		{
			osg::StateSet* stateSet = attr->getParent(i);
			stateSet->setAttributeAndModes(m, 1);
		}
	}

private:
	osg::ref_ptr<osgAnimation::InOutCubicMotion> _motion;
};


void creatMaterial(osg::StateSet* ss)
{
	osg::ref_ptr<osg::Material> m = new osg::Material();
	m->setAmbient(m->FRONT, osg::Vec4(.5, .5, .5, 1));
	m->setDiffuse(m->FRONT, osg::Vec4(1, 1, 1, 1));
	m->setSpecular(m->FRONT, osg::Vec4(1, 1, 1, 1)); //0.2, 0.2, 0.2, 1));
	m->setEmission(m->FRONT, osg::Vec4(0, 0, 0, 1));
	m->setShininess(m->FRONT, 100.0);
	ss->setAttributeAndModes(m, 1); //osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	m->setUpdateCallback(new MaterialCallback());
}


int main(int argc, char* argv[])
{
	osgViewer::Viewer viewer;

	osg::Geode *boxGeode = new osg::Geode;
	boxGeode->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 0, 0), 2)));
	creatMaterial(boxGeode->getOrCreateStateSet());
	//boxGeode->getOrCreateStateSet()->setAttributeAndModes(creatMaterial, 1);

	//osgAnimation::Animation* animation = new osgAnimation::Animation;
	//osgAnimation::Vec4LinearChannel * channel0 = new osgAnimation::Vec4LinearChannel;
	//channel0->getOrCreateSampler()->getOrCreateKeyframeContainer()->push_back(osgAnimation::Vec4Keyframe(0, osg::Vec4(1.0, 0.0, 0.0, 1.0)));
	//channel0->getOrCreateSampler()->getOrCreateKeyframeContainer()->push_back(osgAnimation::Vec4Keyframe(1, osg::Vec4(1.0, 1.0, 0.0, 1.0)));
	//channel0->setTargetName("UpdateMaterialCallback");//设置作用对象名称，必须和UpdateMaterial的名称设置为一样  
	//channel0->setName("diffuseChannel");//频道的名称中必须包含"diffuse"字符  

	//animation->addChannel(channel0);
	//animation->setPlayMode(osgAnimation::Animation::PPONG);
	//osgAnimation::BasicAnimationManager* bam = new osgAnimation::BasicAnimationManager;
	//bam->registerAnimation(animation);

	//osgAnimation::UpdateMaterial* updateMaterial = new osgAnimation::UpdateMaterial("UpdateMaterialCallback");
	//osg::Material *boxMaterial = new osg::Material;
	////添加UpdateMaterial到Material对象的更新回调之中  
	//boxMaterial->setUpdateCallback(updateMaterial);
	//boxMaterial->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1));
	//boxGeode->getOrCreateStateSet()->setAttribute(boxMaterial, osg::StateAttribute::ON);

	viewer.setCameraManipulator(new osgGA::TrackballManipulator());

	osg::Group* scene = new osg::Group;
	//scene->addUpdateCallback(bam);

	scene->addChild(boxGeode);

	viewer.addEventHandler(new osgViewer::StatsHandler());
	viewer.addEventHandler(new osgViewer::WindowSizeHandler());
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

	viewer.setSceneData(scene);
	viewer.realize();

	//bam->playAnimation(animation);


	while (!viewer.done())
	{
		viewer.frame();
	}

	return 0;
}