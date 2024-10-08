using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FKManager : MonoBehaviour
{
    public Joint m_root;
    public Transform m_target;
    public float[] jointAngles;
    public float[] d;
    public float[] a;
    public float[] alpha;
    public float m_threshold = 0.05f;

    void Start()
    {
        if (jointAngles.Length == 0 || d.Length == 0 || a.Length == 0 || alpha.Length == 0)
        {
            Debug.LogError("No joint angles or DH parameters specified. Please set them.");
        }
    }

    void Update()
    {
        if (m_root == null || jointAngles.Length == 0 || m_target == null || d.Length == 0 || a.Length == 0 || alpha.Length == 0)
        {
            Debug.LogError("Root joint, joint angles array, target, or DH parameters are not set properly.");
            return;
        }

        ApplyForwardKinematics();
        UpdateJointAngles();
    }

    void ApplyForwardKinematics()
    {
        Joint currentJoint = m_root;
        int index = 0;

        while (currentJoint != null && index < jointAngles.Length)
        {
            currentJoint.Rotate(jointAngles[index]);
            currentJoint = currentJoint.GetChild();
            index++;
        }
    }

    Matrix4x4 GetTransformMatrix(float theta, float d, float a, float alpha)
    {
        float radTheta = theta * Mathf.Deg2Rad;
        float radAlpha = alpha * Mathf.Deg2Rad;

        return new Matrix4x4(
            new Vector4(Mathf.Cos(radTheta), -Mathf.Sin(radTheta) * Mathf.Cos(radAlpha), Mathf.Sin(radTheta) * Mathf.Sin(radAlpha), a * Mathf.Cos(radTheta)),
            new Vector4(Mathf.Sin(radTheta), Mathf.Cos(radTheta) * Mathf.Cos(radAlpha), -Mathf.Cos(radTheta) * Mathf.Sin(radAlpha), a * Mathf.Sin(radTheta)),
            new Vector4(0, Mathf.Sin(radAlpha), Mathf.Cos(radAlpha), d),
            new Vector4(0, 0, 0, 1)
        );
    }

    void UpdateJointAngles()
    {
        Vector3 endEffectorPosition = GetEndEffectorPosition();
        float distanceToTarget = Vector3.Distance(endEffectorPosition, m_target.position);

        if (distanceToTarget < m_threshold)
        {
            Debug.Log("Target reached.");
            return;
        }

        for (int i = 0; i < jointAngles.Length; i++)
        {
            jointAngles[i] += 1.0f;
        }
    }

    Vector3 GetEndEffectorPosition()
    {
        Vector3 position = m_root.transform.position;
        Joint currentJoint = m_root;
        int index = 0;

        while (currentJoint != null && index < jointAngles.Length)
        {
            currentJoint.Rotate(jointAngles[index]);
            position += currentJoint.transform.position;
            currentJoint = currentJoint.GetChild();
            index++;
        }

        return position;
    }
}
