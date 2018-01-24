#include "Vector.h"
#include <math.h>

void add3(const osg::Vec3& first, const osg::Vec3& second, osg::Vec3& result)
{
	result.x() = first.x() + second.x();
	result.y() = first.y() + second.y();
	result.z() = first.z() + second.z();
}

void add4(const osg::Vec4& first, const osg::Vec4& second, osg::Vec4& result)
{
	result.x() = first.x() + second.x();
	result.y() = first.y() + second.y();
	result.z() = first.z() + second.z();
	result.w() = first.w() + second.w();
}

void sub3(const osg::Vec3& first, const osg::Vec3& second, osg::Vec3& result)
{
	result.x() = first.x() - second.x();
	result.y() = first.y() - second.y();
	result.z() = first.z() - second.z();
}

void sub4(const osg::Vec4& first, const osg::Vec4& second, osg::Vec4& result)
{
	result.x() = first.x() - second.x();
	result.y() = first.y() - second.y();
	result.z() = first.z() - second.z();
	result.w() = first.w() - second.w();
}

void mul3(const osg::Vec3& first, const osg::Vec3& second, osg::Vec3& result)
{
	result.x() = first.x() * second.x();
	result.y() = first.y() * second.y();
	result.z() = first.z() * second.z();
}

void div3(const osg::Vec3& first, const osg::Vec3& second, osg::Vec3& result)
{
	result.x() = first.x() / second.x();
	result.y() = first.y() / second.y();
	result.z() = first.z() / second.z();
}

float dot3(const osg::Vec3& first, const osg::Vec3& second)
{
	int i;
	float dot = 0;
	for (i = 0; i < 3; ++i)
	{
		dot += first._v[i] * second._v[i];
	}
	return dot;
}

float dot4(const osg::Vec4& first, const osg::Vec4& second)
{
	int i;
	float dot = 0;
	for (i = 0; i < 4; ++i)
	{
		dot += first._v[i] * second._v[i];
	}
	return dot;
}

void cross3(const osg::Vec3& first, const osg::Vec3& second, osg::Vec3& result)
{
	result.x() = (first.y() * second.z()) - (first.z() * second.y());
	result.y() = (first.z() * second.x()) - (first.x() * second.z());
	result.z() = (first.x() * second.y()) - (first.y() * second.x());
}

void cross4(const osg::Vec4& first, const osg::Vec4& second, osg::Vec4& result)
{
	result.x() = (first.y() * second.z()) - (first.z() * second.y());
	result.y() = (first.z() * second.x()) - (first.x() * second.z());
	result.z() = (first.x() * second.y()) - (first.y() * second.x());
}

void normalise3(const osg::Vec3& first, osg::Vec3& result)
{
	float length = length3(first);
	result.x() = first.x() / length;
	result.y() = first.y() / length;
	result.z() = first.z() / length;
}

float length3(const osg::Vec3& first)
{
	// Calculate length with pythagoras
	float lengthSquared = first.x() * first.x() + first.y() * first.y() + first.z() * first.z();
	float length = sqrt(lengthSquared);
	return length;
}

void lerp3(const osg::Vec3& from, const osg::Vec3& to, float t, osg::Vec3& result)
{
	result.x() = from.x() + t * (to.x() - from.x());
	result.y() = from.y() + t * (to.y() - from.y());
	result.z() = from.z() + t * (to.z() - from.z());
}

void lerp4(const osg::Vec4 & from, const osg::Vec4 & to, float t, osg::Vec4 & result)
{
	result.x() = from.x() + t * (to.x() - from.x());
	result.y() = from.y() + t * (to.y() - from.y());
	result.z() = from.z() + t * (to.z() - from.z());
	result.w() = from.w() + t * (to.w() - from.w());
}

float linePlaneCoefficient(const osg::Vec3& linePoint, const osg::Vec3& lineDir, const osg::Plane& plane/*const osg::Vec3& planeNormal, const osg::Vec3& planePoint*/)
{
	//osg::Plane plane(planeNormal, planePoint);
	osg::Vec3 point2 = linePoint + lineDir;
	float dis1 = abs(plane.distance(linePoint));
	float dis2 = abs(plane.distance(point2));
	float sum = dis1 + dis2;
	float result0 = sum == 0.0 ? 0.0f : dis1 / (dis1 + dis2);

	//osg::Vec3 diff;
	//float dot1;
	//float dot2;
	//float result;
	//sub3(planePoint, linePoint, diff);
	//dot1 = dot3(planeNormal, diff);
	//dot2 = dot3(planeNormal, lineDir);
	//result = dot1 / dot2;

	return result0;
}
