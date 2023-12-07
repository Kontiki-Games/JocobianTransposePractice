using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Security.Cryptography;
using System.Text;

public struct Frame
{
    //private static string PrintVector3(Vector3 v)
    //{
    //    StringBuilder sb = new StringBuilder();
    //    sb.Append(v.x);
    //    sb.Append(", ");

    //    sb.Append(v.y);
    //    sb.Append(", ");

    //    sb.Append(v.z);
    //    sb.Append(", ");

    //    return sb.ToString();
    //}


    public Vector3 m_position;
    public Quaternion m_rotation;

    public Frame(Vector3 position, Quaternion rotation)
    {
        //UnityEngine.Debug.LogError("Frame before: " + PrintVector3(position));
        m_position = position;
        m_rotation = rotation;
        //UnityEngine.Debug.LogError("Frame after: " + PrintVector3(m_position));
    }
}

public class IkJoint
{
    static Quaternion Normalize(Quaternion q)
    {
        var m = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        return new Quaternion(q.x / m, q.y / m, q.z / m, q.w / m);
    }

    //private string PrintVector3(Vector3 v)
    //{
    //    StringBuilder sb = new StringBuilder();
    //    sb.Append(v.x);
    //    sb.Append(", ");

    //    sb.Append(v.x);
    //    sb.Append(", ");

    //    sb.Append(v.x);
    //    sb.Append(", ");

    //    return sb.ToString();
    //}

    public IkJoint(IkJoint parent)
    {
        _parent = parent;
        _children = new List<IkJoint>();

        var jointShape = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        _renderableTransform = jointShape.transform;
        _renderableTransform.localScale = 0.05f * Vector3.one;
    }

    public void SetFrame(Frame frame, bool isLocal)
    {
        //UnityEngine.Debug.LogError("SetFrame before: " + PrintVector3(frame.m_position));
        _frame = frame;
        _isLocal = isLocal;

        //UnityEngine.Debug.LogError("SetFrame after: " + PrintVector3(_frame.m_position));
    }

    public void AddChild(IkJoint child)
    {
        _children.Add(child);
    }

    public void UpdateRenderable()
    {
        _renderableTransform.position = _frame.m_position;
        _renderableTransform.rotation = _frame.m_rotation;

        foreach (var child in _children)
        {
            child.UpdateRenderable();
        }

        if(_children.Count > 0)
        {
            UnityEngine.Debug.DrawLine(_frame.m_position, _children[0]._frame.m_position, Color.yellow);
        }
    }

    public void UpdateFrameToLocal()
    {
        if(_isLocal == true)
        {
            UnityEngine.Debug.LogError("Unexpected space flag.");
            return;
        }

        if(_parent == null)
        {
            _isLocal = true;
            return;
        }

        if(_parent._isLocal == true)
        {
            UnityEngine.Debug.LogError("Unexpected parent space flag.");
            return;
        }

        var inversedParentRotation = Quaternion.Inverse(_parent._frame.m_rotation);
        Vector3 translation_world = _frame.m_position - _parent._frame.m_position;
        Vector3 translation_local = inversedParentRotation * translation_world;
        _frame.m_position = translation_local;
        //_frame.m_rotation *= inversedParentRotation;
        _frame.m_rotation = Normalize(_frame.m_rotation * inversedParentRotation);

        _isLocal = true;
    }

    public void UpdateFrameToWorld()
    {
        if (_isLocal == false)
        {
            UnityEngine.Debug.LogError("Unexpected space flag.");
            return;
        }

        if (_parent == null)
        {
            _isLocal = false;
            return;
        }

        if (_parent._isLocal == true)
        {
            UnityEngine.Debug.LogError("Unexpected parent space flag.");
            return;
        }

        _frame.m_position = _parent._frame.m_rotation * _frame.m_position + _parent._frame.m_position;
        //_frame.m_rotation *= _parent._frame.m_rotation;
        _frame.m_rotation = Normalize(_frame.m_rotation * _parent._frame.m_rotation);

        _isLocal = false;
    }

    public IkJoint _parent;
    public List<IkJoint> _children;
    public Frame _frame;
    public bool _isLocal;
    private Transform _renderableTransform;
}
