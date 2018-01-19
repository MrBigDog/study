#ifndef adjust_near_far_plane_hpp__
#define adjust_near_far_plane_hpp__
#include <osg/Math>
#include <osg/Camera>
#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/CoordinateSystemNode>

#include <osgUtil/CullVisitor>

#include <algorithm>


struct P3D_ProjClamper : public osg::CullSettings::ClampProjectionMatrixCallback
{
	double _minNear, _maxFar, _nearFarRatio;

	P3D_ProjClamper() : _minNear(-DBL_MAX), _maxFar(DBL_MAX), _nearFarRatio(0.0000007) { }

	template<class matrix_type, class value_type>
	bool _clampProjectionMatrix(matrix_type& projection, double& znear, double& zfar, value_type nearFarRatio) const
	{

		double epsilon = 1e-6;
		if (zfar < znear - epsilon)
		{
			OSG_INFO << "_clampProjectionMatrix not applied, invalid depth range, znear = " << znear << "  zfar = " << zfar << std::endl;
			return false;
		}

		if (zfar < znear + epsilon)
		{
			double average = (znear + zfar) * 0.5;
			znear = average - epsilon;
			zfar = average + epsilon;
		}

		if (fabs(projection(0, 3)) < epsilon  && fabs(projection(1, 3)) < epsilon  && fabs(projection(2, 3)) < epsilon)
		{
			value_type delta_span = (zfar - znear) * 0.02;
			if (delta_span < 1.0) { delta_span = 1.0; }
			value_type desired_znear = znear - delta_span;
			value_type desired_zfar = zfar + delta_span;

			znear = desired_znear;
			zfar = desired_zfar;

			projection(2, 2) = -2.0f / (desired_zfar - desired_znear);
			projection(3, 2) = -(desired_zfar + desired_znear) / (desired_zfar - desired_znear);
		}
		else
		{
			value_type zfarPushRatio = 1.02;
			value_type znearPullRatio = 0.98;

			//znearPullRatio = 0.99;
			value_type desired_znear = znear * znearPullRatio;
			value_type desired_zfar = zfar * zfarPushRatio;

			double min_near_plane = zfar * nearFarRatio;

			if (desired_znear < min_near_plane) { desired_znear = min_near_plane; }
			//if (desired_znear > min_near_plane) desired_znear=min_near_plane;

			if (desired_znear < 1.0)
			{
				desired_znear = 1.0;
			}
			znear = desired_znear;
			zfar = desired_zfar;

			value_type trans_near_plane = (-desired_znear * projection(2, 2) + projection(3, 2)) / (-desired_znear * projection(2, 3) + projection(3, 3));
			value_type trans_far_plane = (-desired_zfar * projection(2, 2) + projection(3, 2)) / (-desired_zfar * projection(2, 3) + projection(3, 3));

			value_type ratio = fabs(2.0 / (trans_near_plane - trans_far_plane));
			value_type center = -(trans_near_plane + trans_far_plane) / 2.0;

			projection.postMult(osg::Matrix(1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f, 0.0f,
				0.0f, 0.0f, ratio, 0.0f,
				0.0f, 0.0f, center * ratio, 1.0f));
		}
		return true;
	}


	bool clampProjectionMatrixImplementation(osg::Matrixf& projection, double& znear, double& zfar) const
	{
		double n = (std::max)(znear, _minNear);
		double f = (std::min)(zfar, _maxFar);
		bool r = _clampProjectionMatrix(projection, n, f, _nearFarRatio);
		if (r)
		{
			znear = n;
			zfar = f;
		}
		return r;
	}

	bool clampProjectionMatrixImplementation(osg::Matrixd& projection, double& znear, double& zfar) const
	{
		double n = (std::max)(znear, _minNear);
		double f = (std::min)(zfar, _maxFar);
		bool r = _clampProjectionMatrix(projection, n, f, _nearFarRatio);
		if (r)
		{
			znear = n;
			zfar = f;
		}
		return r;
	}
};

class AutoClipPlaneCullCallback : public osg::NodeCallback
{
private:
	bool autoFarPlaneClamping;
	double minNearFarRatio;
	double maxNearFarRatio;
	double haeThreshold;
	double rp;
	double rp2;

	osg::ref_ptr<osg::EllipsoidModel> em;
	osg::Matrix camera_matrix_pre_;

public:
	AutoClipPlaneCullCallback()
		: autoFarPlaneClamping(true)
		, minNearFarRatio(0.0000000001)
		, maxNearFarRatio(0.00005)
		, haeThreshold(250.0)
		, rp(-1)
		, rp2(-1)
		, em(new osg::EllipsoidModel(osg::WGS_84_RADIUS_EQUATOR, osg::WGS_84_RADIUS_EQUATOR)) ///WGS_84_RADIUS_POLAR
	{
		rp = (std::min)(em->getRadiusEquator(), em->getRadiusEquator());
		rp2 = rp * rp;
	}

private:
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		//OSG_INFO(node, "AutoClipPlaneCullCallback () node");
		//OSG_INFO(nv, "AutoClipPlaneCullCallback () nv");
		osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
		if (cv == NULL) { return; }

		osg::Camera* cam = cv->getCurrentCamera();
		osg::Matrix camera_matrix = cam->getViewMatrix();
		if (camera_matrix != camera_matrix_pre_)
		{
			camera_matrix_pre_ = camera_matrix;
			auto_adjust_far_near_plane(camera_matrix, cam);
		}
		traverse(node, nv);
	}

	void auto_adjust_far_near_plane(osg::Matrix& camera_matrix, osg::Camera* cam)
	{
		osg::CullSettings::ClampProjectionMatrixCallback* clamper = cam->getClampProjectionMatrixCallback();
		if (NULL == clamper)
		{
			clamper = new P3D_ProjClamper();
			cam->setClampProjectionMatrixCallback(clamper);
			return;
		}

		osg::Vec3d eye, center, up;
		camera_matrix.getLookAt(eye, center, up);

		P3D_ProjClamper* c = static_cast<P3D_ProjClamper*>(clamper);

		double d = eye.length();
		c->_maxFar = sqrt(d * d - rp2) + osg::WGS_84_RADIUS_POLAR * 0.35;

		osg::Vec3d loc;
		em->convertXYZToLatLongHeight(eye.x(), eye.y(), eye.z(), loc.y(), loc.x(), loc.z());

		double hae = loc.z();//@todo hae = ???;
		c->_nearFarRatio = reclamp(hae, 0.0, haeThreshold, minNearFarRatio, maxNearFarRatio);
	}


	double reclamp(double v, double vmin, double vmax, double r0, double r1)
	{
		float vr = (osg::clampBetween(v, vmin, vmax) - vmin) / (vmax - vmin);
		return r0 + vr * (r1 - r0);
	}

};

#endif // clip_h__