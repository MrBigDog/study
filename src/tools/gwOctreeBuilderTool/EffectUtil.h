#ifndef STATESETUTIL_H
#define STATESETUTIL_H 1

#include <osg/Program>
#include <osg/Material>
#include <osg/StateSet>
#include <osg/BlendFunc>
#include <osg/BlendColor>
#include <osgAnimation/EaseMotion>

namespace gwUtil
{
	class Effect :public osg::Referenced
	{
	public:
		void install(osg::Node* node) {}
		void unstall(osg::Node* node) {}
	};

	class BloomEffect :public Effect
	{
	public:
		BloomEffect() {}

		void install(osg::Node* node) {}
		void unstall(osg::Node* node) {}

		void setBloomColor(const osg::Vec4& color1, const osg::Vec4& color2)
		{
			_color1 = color1;
			_color2 = color2;
		}

	private:
		osg::Vec4 _color1, _color2;
	};

	class TransparentEffect :public Effect
	{
	public:
		TransparentEffect() {}

		void install(osg::Node* node) {}
		void unstall(osg::Node* node) {}

		void setAlfa(float alfa) { _alfa = alfa; }

	private:
		float _alfa;
	};






	//Ч�ʸߣ�Ч����(��Ϊshader��û�ӹ���);
	void setBloomEffect1(osg::Node* node);

	//�޸�Color Array�ķ�ʽ��Ч�ʵ�, Ч����;
	void setBloomEffect2(osg::Node* node, const osg::Vec4& color1, const osg::Vec4& color2);

	//�޸�Material�ķ�ʽ��Ч�ʱ�2�ߣ���1�Եͣ�Ч��ͬ2;
	void setBloomEffect3(osg::Node* node, const osg::Vec4& color1, const osg::Vec4& color2);

	//����͸��;
	void setTransparentEffect(osg::Node* state, float alf);

	void renderNodeAsPoint(osg::Node* node);
	void renderNodeAsLine(osg::Node* node);

	void addOutlineEffect(osg::Node* node, const osg::Vec4& color, float width);
}

#endif // StateUtil_h__
