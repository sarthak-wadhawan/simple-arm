using UnityEngine;

public class RobotController : MonoBehaviour
{
    public RobotArm robotArm;
    public TrajectoryGenerator trajectoryGenerator;
    public PDController[] pdControllers;
    public CollisionChecker collisionChecker;
    public Transform targetSphere;

    private float[] jointAngles;

    void Start()
    {
        jointAngles = new float[robotArm.joints.Length];
    }

    void Update()
    {
        // Target position from the sphere's position
        Vector3 targetPosition = targetSphere.position;

        // Use IK to calculate the required joint angles
        float[] newJointAngles = robotArm.GradientDescentIK(targetPosition, jointAngles, 100, 0.1f);

        // Collision check
        Vector3 endEffectorPosition = robotArm.ForwardKinematics(newJointAngles);
        if (!collisionChecker.CheckCollision(endEffectorPosition, 0.5f))
        {
            // Update joints using PD controller
            for (int i = 0; i < newJointAngles.Length; i++)
            {
                float deltaTime = Time.deltaTime;
                jointAngles[i] = pdControllers[i].Compute(newJointAngles[i], jointAngles[i], deltaTime);
            }

            // Update the arm to follow the new joint angles
            UpdateJointTransforms();
        }
    }

    void UpdateJointTransforms()
    {
        for (int i = 0; i < robotArm.joints.Length; i++)
        {
            robotArm.joints[i].localRotation = Quaternion.Euler(0, 0, jointAngles[i]);
        }
    }
}
