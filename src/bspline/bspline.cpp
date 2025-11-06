#include "bspline/bspline.hpp"

Bspline::Bspline(const std::vector<geometry_msgs::msg::Pose>& waypoints, bool setPointFlag)
{
	_p = Bspline::getPointfromPose(waypoints);
	k = 3;
	n = _p.size() - 1;

	if (n >= 2) // k必需<=n+1， 不能一个控制点都没有
	{
		if(setPointFlag)
    {
			Bspline::setPoint(_p);
		}

		p = _p;
		double u_tmp = 0.0;
		u.push_back(u_tmp); // 先把0.0存入

		int j = 3; // 重复度
		double dis_u = 1.0 / (k + n - (j - 1) * 2);

		for (int i = 1; i < j; i++)
		{
			u.push_back(u_tmp);
		}

		for (int i = j; i < n + k - j + 2; i++)
		{
			u_tmp += dis_u;
			u.push_back(u_tmp);
		}

		for (int i = n + k - j + 2; i < n + k + 1; i++) // n + k + 1个分段
		{
			u.push_back(u_tmp);
		}

		uBegin = u[k - 1];
		uEnd = u[n + 1]; // 计算u的区间
	}
}

Bspline::~Bspline()
{
	_p.clear();
	p.clear();
	u.clear();
	pTrack.clear();
	qTrack.clear();
	p_waypoints.clear();
}

// 计算每个u和每个i对应的B样条
double Bspline::BsplineBfunc(int i, int k, double uu)
{
	double Bfunc = 0.0;

	if (k == 1) // 递归退出的条件
	{
		if (u[i] <= uu && uu < u[i + 1])
		{
			Bfunc = 1.0;
		}
		else
		{
			Bfunc = 0.0;
		}
	}
	else if (k >= 2)
	{
		double A = 0.0;
		double B = 0.0;

		if (u[i + k - 1] - u[i] == 0.0)
		{
			A = 0.0; // 约定分母为0时，整个除式为0
		}
		else
		{
			A = (uu - u[i]) / (u[i + k - 1] - u[i]);
		}

		if (u[i + k] - u[i + 1] == 0.0)
		{
			B = 0.0; // 约定分母为0时，整个除式为0
		}
		else
		{
			B = (u[i + k] - uu) / (u[i + k] - u[i + 1]);
		}

		Bfunc = A * BsplineBfunc(i, k - 1, uu) + B * BsplineBfunc(i + 1, k - 1, uu); //递归
	}

	return Bfunc;
}

// 计算整个B样条
vector<geometry_msgs::msg::Pose> Bspline::creatBspline(double delta_u, vector<geometry_msgs::msg::Pose> waypoints)
{
	if (waypoints.size() < 3)
  {
		return waypoints;
	}

	for (double uu = uBegin; uu <= uEnd; uu += delta_u) // u的循环放外层，对应每个u，去遍历所有控制点
	{
		geometry_msgs::msg::Point P;
		P.x = 0;
		P.y = 0;
		P.z = 0; // 每轮循环初始化

		for (int i = 0; i < n + 1; i++) // i从0到n，每个控制点
		{
			double xtmp = p[i].x;
			double ytmp = p[i].y;
			double ztmp = p[i].z;
			double BfuncTmp = BsplineBfunc(i, k, uu);

			P.x += xtmp * BfuncTmp;
			P.y += ytmp * BfuncTmp;
			P.z += ztmp * BfuncTmp; // 累加
		}

		pTrack.push_back(P); // 得到轨迹点
	}
	
	int size = pTrack.size();
  geometry_msgs::msg::Quaternion Q;
	
	tf2::Quaternion quat_start;
	tf2::Quaternion quat_end;
  tf2::Quaternion quat_result;

  int ft = 0;
	int bt = 1;

	Q = Bspline::setQ(waypoints[0]);
	tf2::fromMsg(Q, quat_start);

  for (long unsigned wp = 1; wp < waypoints.size(); wp++)
  {
    while (bt < size-1)
    {
      if ((waypoints[wp].position.x - pTrack[bt].x) * (waypoints[wp].position.x - pTrack[bt+1].x) < 0 ||
          (waypoints[wp].position.y - pTrack[bt].y) * (waypoints[wp].position.y - pTrack[bt+1].y) < 0 ||
          (waypoints[wp].position.z - pTrack[bt].z) * (waypoints[wp].position.z - pTrack[bt+1].z) < 0) 
      {
        Q = Bspline::setQ(waypoints[wp]);
        tf2::fromMsg(Q, quat_end);

        double rat = 1.0;
        if (bt-ft>1) 
        {
          rat = 1.0 / (bt-ft);
        }

        for(int i = 0; i< bt-ft; i++)
        {		
          quat_result = quat_start.slerp(quat_end,rat*i);
          Q = tf2::toMsg(quat_result);
          qTrack.push_back(Q);
        }

        ft = bt; 
        quat_start = quat_end;
        break;
      }
      else 
      {
        bt++;
      }
    }	
	}

	double rat = 1;
	if (size - ft > 1) 
  {
		rat = 1.0 / (size - ft - 1);
	}

	Q = Bspline::setQ(*(waypoints.end() - 1));
	tf2::fromMsg(Q, quat_end);

	for(int i = 0; i < size - ft; i++)
  {		
		quat_result = quat_start.slerp(quat_end, rat * i);
		Q = tf2::toMsg(quat_result);
		qTrack.push_back(Q);
	}

	geometry_msgs::msg::Pose p_waypoint;

	for (int j = 0; j < size; j++)
	{
		p_waypoint.position.x = pTrack[j].x;
		p_waypoint.position.y = pTrack[j].y;
		p_waypoint.position.z = pTrack[j].z;

		p_waypoint.orientation.x = qTrack[j].x;
		p_waypoint.orientation.y = qTrack[j].y;
		p_waypoint.orientation.z = qTrack[j].z;
		p_waypoint.orientation.w = qTrack[j].w;

		p_waypoints.push_back(p_waypoint);
	}
  
	return p_waypoints;
}

// 补充点
void Bspline::setPoint(vector<geometry_msgs::msg::Point>& p)
{
	int i = 1;
	int size = p.size();
	float k = 0.2;

	geometry_msgs::msg::Point p_medium1;
	geometry_msgs::msg::Point p_medium2;

	while (i < size + 2 * (size - 1)) 
  {
		p_medium1.x = p[i - 1].x + k * (p[i].x - p[i - 1].x);
		p_medium1.y = p[i - 1].y + k * (p[i].y - p[i - 1].y);
		p_medium1.z = p[i - 1].z + k * (p[i].z - p[i - 1].z);

		p_medium2.x = p[i - 1].x + (1 - k) * (p[i].x - p[i - 1].x);
		p_medium2.y	= p[i - 1].y + (1 - k) * (p[i].y - p[i - 1].y);
		p_medium2.z = p[i - 1].z + (1 - k) * (p[i].z - p[i - 1].z);

		p.insert(p.begin() + i, p_medium2);
		p.insert(p.begin() + i, p_medium1);

		i = i + 3;
	}
}

vector<geometry_msgs::msg::Point> Bspline::getPointfromPose(vector<geometry_msgs::msg::Pose> pose)
{
	geometry_msgs::msg::Point point;
	vector<geometry_msgs::msg::Point> p_vector;
	int size = pose.size();

	for (int i = 0; i < size; i++)
	{
		point.x = pose[i].position.x;
		point.y = pose[i].position.y;
		point.z = pose[i].position.z;

		p_vector.push_back(point);
	}

	return p_vector;
}

geometry_msgs::msg::Quaternion Bspline::setQ(geometry_msgs::msg::Pose pose)
{
	geometry_msgs::msg::Quaternion Q;

	Q.x = pose.orientation.x;
	Q.y = pose.orientation.y;
	Q.z = pose.orientation.z;
	Q.w = pose.orientation.w;

	return Q;
}

