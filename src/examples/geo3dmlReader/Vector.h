#ifndef __VECTOR_H__
#define __VECTOR_H__

#include <osg/Vec2>
#include <osg/Vec3>
#include <osg/Vec4>

namespace gwUtil
{
	void add3(const osg::Vec3& first, const osg::Vec3& second, osg::Vec3& result);
	void add4(const osg::Vec4& first, const osg::Vec4& second, osg::Vec4& result);

	void sub3(const osg::Vec3& first, const osg::Vec3& second, osg::Vec3& result);
	void sub4(const osg::Vec4& first, const osg::Vec4& second, osg::Vec4& result);

	void mul3(const osg::Vec3& first, const osg::Vec3& second, osg::Vec3& result);

	void div3(const osg::Vec3& first, const osg::Vec3& second, osg::Vec3& result);

	float dot3(const osg::Vec3& first, const osg::Vec3& second);
	float dot4(const osg::Vec4& first, const osg::Vec4& second);

	void cross3(const osg::Vec3& first, const osg::Vec3& second, osg::Vec3& result);
	void cross4(const osg::Vec4& first, const osg::Vec4& second, osg::Vec4& result);

	void normalise3(const osg::Vec3& first, osg::Vec3& result);

	float length3(const osg::Vec3& first);

	void lerp3(const osg::Vec3& from, const osg::Vec3& to, float t, osg::Vec3& result);
	void lerp4(const osg::Vec4& from, const osg::Vec4& to, float t, osg::Vec4& result);

	float linePlaneCoefficient(const osg::Vec3& linePoint, const osg::Vec3& lineDir, const osg::Vec3& planeNormal, const osg::Vec3& planePoint);
}

#endif
