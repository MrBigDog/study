#include "EffectUtil.h"
#include <osg/Node>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/NodeVisitor>
#include <osg/PolygonMode>

#include <osgFX/Outline>

namespace
{
	class MaterialCallback : public osg::StateAttributeCallback
	{
	public:
		MaterialCallback(const osg::Vec4& color1, const osg::Vec4& color2)
			:_motion(new osgAnimation::InOutCubicMotion(0, 1, 1, osgAnimation::Motion::LOOP))
			, _color1(color1), _color2(color2)
		{}

		virtual void operator() (osg::StateAttribute* attr, osg::NodeVisitor* nv)
		{
			osg::Material* m = dynamic_cast<osg::Material*>(attr);
			if (!m) return;

			_motion->update(0.25);
			float rat = _motion->getValue();

			osg::Vec4 color = _color1 + (_color2 - _color1)*rat;
			m->setDiffuse(m->FRONT, color);
			for (unsigned int i = 0; i < attr->getNumParents(); i++)
			{
				osg::StateSet* stateSet = attr->getParent(i);
				osg::StateSet::AttributeList& al = stateSet->getAttributeList();
				for (osg::StateSet::AttributeList::iterator it = al.begin(); it != al.end(); ++it)
				{
					osg::Material* mm = dynamic_cast<osg::Material*>((*it).second.first.get());
					if (!mm) continue;
					mm->setDiffuse(mm->FRONT, color);
				}
				//stateSet->setAttributeAndModes(m, 1);
			}
		}

	private:
		osg::Vec4 _color1, _color2;
		osg::ref_ptr<osgAnimation::InOutCubicMotion> _motion;
	};
}

void setBloomEffect3(osg::Node* node, const osg::Vec4& color1, const osg::Vec4& color2)
{
	osg::ref_ptr<osg::Material> m = new osg::Material();
	m->setAmbient(m->FRONT, osg::Vec4(0.75, 0.75, 0.75, 1));
	m->setDiffuse(m->FRONT, color1);
	m->setSpecular(m->FRONT, osg::Vec4(1, 1, 1, 1)); //0.2, 0.2, 0.2, 1));
	m->setEmission(m->FRONT, osg::Vec4(0, 0, 0, 1));
	m->setShininess(m->FRONT, 100.0);
	node->getOrCreateStateSet()->setAttributeAndModes(m, 1); //osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	m->setUpdateCallback(new MaterialCallback(color1, color2));
}

namespace
{
	static const char*color_vertex =
	{
		"void main()  					\n"
		"{								\n"
		"	gl_Position = ftransform(); \n"
		"}  							\n"
	};

	static const char* color_frag =
	{
		"uniform float osg_FrameTime;				\n"
		"											\n"
		"void main()								\n"
		"{											\n"
		"	float a = fract(osg_FrameTime);			\n"
		"	gl_FragColor = vec4(1.0, a, 0.0, 0.1);	\n"
		"}											\n"
	};
}

void setBloomEffect1(osg::Node* node)
{

	osg::ref_ptr<osg::Program> program = new osg::Program;
	program->addShader(new osg::Shader(osg::Shader::VERTEX, color_vertex));
	program->addShader(new osg::Shader(osg::Shader::FRAGMENT, color_frag));

	node->getOrCreateStateSet()->setAttributeAndModes(program);
}

void setTransparentEffect(osg::Node* node, float alf)
{
	osg::StateSet* state = node->getOrCreateStateSet();
	state->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
	state->setMode(GL_BLEND, osg::StateAttribute::ON);
	state->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
	state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	osg::ref_ptr<osg::BlendColor> bc = new osg::BlendColor(osg::Vec4(1.0, 1.0, 1.0, 0.0));
	osg::ref_ptr<osg::BlendFunc>bf = new osg::BlendFunc();
	state->setAttributeAndModes(bf, osg::StateAttribute::ON);
	state->setAttributeAndModes(bc, osg::StateAttribute::ON);
	bf->setSource(osg::BlendFunc::CONSTANT_ALPHA);
	bf->setDestination(osg::BlendFunc::ONE_MINUS_CONSTANT_ALPHA);
	bc->setConstantColor(osg::Vec4(1, 1, 1, 0.5));
}

namespace
{
	class GeomVisitor2 :public osg::NodeVisitor
	{
	public:
		GeomVisitor2(const osg::Vec4& color1, const osg::Vec4& color2, float rat)
			:osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
			, _rat(rat)
			, _color(color2)
			, _scolor(color1)
		{}

		void apply(osg::Geode& geode)
		{
			for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
			{
				osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
				if (!geom) continue;

				osg::Vec4Array* ca = dynamic_cast<osg::Vec4Array*>(geom->getColorArray());
				if (!ca) continue;

				for (unsigned int j = 0; j < ca->size(); ++j)
				{
					osg::Vec4 newColor = _scolor + (_color - _scolor)*_rat;
					ca->at(j) = newColor;
				}
				ca->dirty();
			}
		}

	private:
		float _rat;
		osg::Vec4 _color;
		osg::Vec4 _scolor;
	};

	class FGRedoutCallback : public osg::NodeCallback
	{
	public:
		FGRedoutCallback(const osg::Vec4& color1, const osg::Vec4& color2)
			:_color1(color1), _color2(color2)
		{
			_motion = new osgAnimation::InOutCubicMotion(0, 1, 1, osgAnimation::Motion::LOOP);
		}
		virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
		{
			_motion->update(0.25);
			float aa = _motion->getValue();

			GeomVisitor2 gv2(_color1, _color2, aa);
			node->accept(gv2);
		}
	private:
		osg::Vec4 _color1, _color2;
		osg::ref_ptr<osgAnimation::InOutCubicMotion> _motion;
	};
}

void setBloomEffect2(osg::Node * node, const osg::Vec4& color1, const osg::Vec4& color2)
{
	if (!node) return;
	node->setUpdateCallback(new FGRedoutCallback(color1, color2));
}


void renderNodeAsPoint(osg::Node* node)
{
	osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode;
	pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::POINT);
	node->getOrCreateStateSet()->setAttributeAndModes(pm, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
}

void renderNodeAsLine(osg::Node* node)
{
	osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode;
	pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
	node->getOrCreateStateSet()->setAttributeAndModes(pm, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
}



namespace
{
	template<typename T>
	class FindTopMostNodeOfTypeVisitor : public osg::NodeVisitor
	{
	public:
		FindTopMostNodeOfTypeVisitor() :
			osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
			_foundNode(0)
		{}

		void apply(osg::Node& node)
		{
			T* result = dynamic_cast<T*>(&node);
			if (result)
			{
				_foundNode = result;
			}
			else
			{
				traverse(node);
			}
		}

		T* _foundNode;
	};

	template<typename T>
	T* findFirstParentOfType(osg::Node* node, unsigned traversalMask = ~0)
	{
		if (!node) return 0;

		FindTopMostNodeOfTypeVisitor<T> fnotv;
		fnotv.setTraversalMode(osg::NodeVisitor::TRAVERSE_PARENTS);
		fnotv.setTraversalMask(traversalMask);
		node->accept(fnotv);

		return fnotv._foundNode;
	}
}

void addOutlineEffect(osg::Node * node, const osg::Vec4& color, float width)
{
	osg::ref_ptr<osgFX::Outline> outlinegroup = new osgFX::Outline;
	outlinegroup->setColor(color);
	outlinegroup->setWidth(width);
	outlinegroup->addChild(node);

	osg::Group* parent = findFirstParentOfType<osg::Group>(node);
	if (parent)
	{
		parent->replaceChild(node, outlinegroup);
	}
}
