using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Security.Cryptography;
using System.Collections.Specialized;
using System.Runtime.Versioning;
using System.Diagnostics;
using System.Text;

public class Main : MonoBehaviour {

	// Use this for initialization
	void Start ()
	{
        var joint0 = new IkJoint(null);
        joint0.SetFrame(new Frame(Vector3.zero, Quaternion.identity), false);

        var joint1 = new IkJoint(joint0);
        joint0.AddChild(joint1);
        joint1.SetFrame(new Frame(Vector3.up * 0.5f, Quaternion.identity), false);

        var joint2 = new IkJoint(joint1);
        joint1.AddChild(joint2);
        joint2.SetFrame(new Frame(Vector3.up * 1.0f, Quaternion.identity), false);

        var joint3 = new IkJoint(joint2);
        joint2.AddChild(joint3);
        joint3.SetFrame(new Frame(Vector3.up * 1.5f, Quaternion.identity), false);

        var joint4 = new IkJoint(joint3);
        joint3.AddChild(joint4);
        joint4.SetFrame(new Frame(Vector3.up * 2.0f, Quaternion.identity), false);

        var joint5 = new IkJoint(joint4);
        joint4.AddChild(joint5);
        joint5.SetFrame(new Frame(Vector3.up * 2.5f, Quaternion.identity), false);

        rootJoint = joint0;
		effectorJoint = joint5;

        
        _IKJointChain = new List<IkJoint>();
		var nextJoint = rootJoint;
		while (nextJoint != null)
		{
			_IKJointChain.Add(nextJoint);

			if(nextJoint._children.Count == 0)
			{
				break;
			}

			nextJoint = nextJoint._children[0];
		}

		_Jocobian = new List<Vector3>();
		_DeltaAngles = new List<float>();
        _Axes = new List<Vector3>();
        for (int i = 0; i < _IKJointChain.Count-1; ++i)
		{
			_Jocobian.Add(Vector3.zero);
			_DeltaAngles.Add(0.0f);
			_Axes.Add(Vector3.one);
        }
    }
	
	// Update is called once per frame
	void Update () {
		int iterateCount = 30;
		while (--iterateCount >= 0)
		{
			var error = CalError();
            if (error.magnitude < 0.0001f)
			{
				break;
			}

            // 计算 Jacobian Transpose
            UpdateJacobianTranspose();

            // 计算旋转增量
            UpdateDeltaRotation(error);

            // 更新关节位置
            DoFK();
		}

		UpdateRenderable();
	}

    private Vector3 CalError()
	{
        return (target.position - effectorJoint._frame.m_position);
	}

	private void PrintJocobian()
	{
        for (int i = 0; i < _IKJointChain.Count - 1; ++i)
        {
            UnityEngine.Debug.Log("PrintJocobian: " + PrintVector3(_Jocobian[i]));
        }
    }

    private void PrintPosition()
    {
        for (int i = 0; i < _IKJointChain.Count; ++i)
        {
            UnityEngine.Debug.Log("PrintPosition: " + PrintVector3(_IKJointChain[i]._frame.m_position));
            UnityEngine.Debug.Log("PrintQuaternion: " + PrintQuaternion(_IKJointChain[i]._frame.m_rotation));
        }
    }

    private void PrintAxes()
    {
        for (int i = 0; i < _Axes.Count; ++i)
        {
            UnityEngine.Debug.Log("PrintAxes: " + PrintVector3(_Axes[i]));
        }
    }

    private void UpdateJacobianTranspose()
	{
		for(int i = 0; i < _IKJointChain.Count-1; ++i)
		{
			var joint = _IKJointChain[i];
			_Axes[i] = Vector3.Cross(effectorJoint._frame.m_position - joint._frame.m_position, target.position - joint._frame.m_position).normalized;
            _Jocobian[i] = Vector3.Cross(_Axes[i], effectorJoint._frame.m_position - joint._frame.m_position);
        }
	}

	private void UpdateForce()
	{ }

	private static string PrintVector3(Vector3 v)
	{
		StringBuilder sb = new StringBuilder();
		sb.Append(v.x);
		sb.Append(", ");

		sb.Append(v.y);
		sb.Append(", ");

		sb.Append(v.z);
		sb.Append(", ");

		return sb.ToString();
	}

    private static string PrintQuaternion(Quaternion v)
    {
        StringBuilder sb = new StringBuilder();
        sb.Append(v.x);
        sb.Append(", ");

        sb.Append(v.y);
        sb.Append(", ");

        sb.Append(v.z);
        sb.Append(", ");

        sb.Append(v.w);
        sb.Append(", ");

        return sb.ToString();
    }

    float Dot(Vector3 v1, Vector3 v2)
	{
		return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	}

    static Quaternion Normalize(Quaternion q)
    {
        var m = Mathf.Sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
        return new Quaternion(q.x / m, q.y / m, q.z / m, q.w / m);
    }

    private float CalculateAlpha(Vector3 error)
    {
        List<float> tmp = new List<float>();
        for (int i = 0; i < _IKJointChain.Count - 1; ++i)
        {
            tmp.Add( Dot(_Jocobian[i], error) );
         }

        Vector3 v = Vector3.zero;
        for (int i = 0; i < _IKJointChain.Count - 1; ++i)
        {
            v += tmp[i] * _Jocobian[i];
        }

        return Dot(error, v) / Dot(v, v);
    }

    private void UpdateDeltaRotation(Vector3 error)
	{
        float alpha = CalculateAlpha(error);
        effectorJoint.UpdateFrameToLocal();
        for (int i = _IKJointChain.Count - 2; i >= 0; --i)
        {
            var deltaTheta = alpha * Dot(_Jocobian[i], error);
			_DeltaAngles[i] = deltaTheta;

            var axis = _Axes[i];
            Quaternion deltaRotation = Normalize(Quaternion.AngleAxis(Mathf.Rad2Deg * deltaTheta, axis));
            _IKJointChain[i]._frame.m_rotation = Normalize(_IKJointChain[i]._frame.m_rotation * deltaRotation);
            _IKJointChain[i].UpdateFrameToLocal();
        }
    }

	private void PrintDeltaAngles()
	{
        for (int i = 0; i < _IKJointChain.Count - 1; ++i)
        {
            UnityEngine.Debug.LogError("PrintDeltaAngles: " + _DeltaAngles[i]);
        }
    }


    private void DoFK()
	{
        for (int i = 0; i < _IKJointChain.Count; ++i)
        {
			_IKJointChain[i].UpdateFrameToWorld();
        }
    }

    private void UpdateRenderable()
	{
		rootJoint.UpdateRenderable();
    }

    private IkJoint rootJoint;
	private IkJoint effectorJoint;
	public Transform target;
	private List<Vector3> _Jocobian;
	private List<IkJoint> _IKJointChain;
	private List<float> _DeltaAngles;
	private List<Vector3> _Axes;
}
