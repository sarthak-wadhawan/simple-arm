using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class TrajectoryGenerator : MonoBehaviour
{
    public Vector3[] GenerateTrajectory(Vector3 startPose, Vector3 targetPose, int steps)
    {
        Vector3[] trajectory = new Vector3[steps];
        for (int i = 0; i < steps; i++)
        {
            float t = (float)i / (steps - 1);
            trajectory[i] = Vector3.Lerp(startPose, targetPose, t);
        }
        return trajectory;
    }

    public void MoveAlongTrajectory(Vector3[] trajectory, float duration)
    {
        StartCoroutine(FollowTrajectory(trajectory, duration));
    }

    IEnumerator FollowTrajectory(Vector3[] trajectory, float duration)
    {
        for (int i = 0; i < trajectory.Length; i++)
        {
            transform.position = trajectory[i];
            yield return new WaitForSeconds(duration / trajectory.Length);
        }
    }
}
