using UnityEngine;

public class RobotArm : MonoBehaviour
{
    public Transform[] joints;  // Array of joints in the robot arm

   public Vector3 ForwardKinematics(float[] joint_angles)
{
    // Link lengths
    float L1 = 1.0f, L2 = 1.0f, L3 = 0.8f, L4 = 0.6f;

    // Joint angles in radians
    float theta1 = joint_angles[0] * Mathf.Deg2Rad;
    float theta2 = joint_angles[1] * Mathf.Deg2Rad;
    float theta3 = joint_angles[2] * Mathf.Deg2Rad;
    float theta4 = joint_angles[3] * Mathf.Deg2Rad;

    // Compute transformations for each joint-link pair
    Matrix4x4 T1 = Matrix4x4.TRS(new Vector3(0, 0, 0), Quaternion.Euler(0, 0, theta1), new Vector3(L1, 0, 0));
    Matrix4x4 T2 = Matrix4x4.TRS(new Vector3(L1, 0, 0), Quaternion.Euler(0, theta2, 0), new Vector3(L2, 0, 0));
    Matrix4x4 T3 = Matrix4x4.TRS(new Vector3(L2, 0, 0), Quaternion.Euler(0, 0, theta3), new Vector3(L3, 0, 0));
    Matrix4x4 T4 = Matrix4x4.TRS(new Vector3(L3, 0, 0), Quaternion.Euler(0, theta4, 0), new Vector3(L4, 0, 0));

    // Final end-effector position (multiply transformations)
    Matrix4x4 T_end_effector = T1 * T2 * T3 * T4;

    // Extract the position of the end-effector
    Vector3 end_effector_pos = T_end_effector.GetColumn(3); // Get the translation part of the matrix

    return end_effector_pos;
}


public float[] GradientDescentIK(Vector3 targetPosition, float[] initialJointAngles, int maxIterations, float learningRate)
{
    // Clone the initial joint angles so as not to modify the original
    float[] jointAngles = (float[])initialJointAngles.Clone();
    
    for (int iteration = 0; iteration < maxIterations; iteration++)
    {
        // Get the current position of the end-effector using forward kinematics
        Vector3 currentPosition = ForwardKinematics(jointAngles);

        // Calculate the error between the target and current positions
        Vector3 error = targetPosition - currentPosition;
        
        // If the error is small enough, stop the iteration
        if (error.magnitude < 0.01f)
            break;

        // Iterate over each joint angle to compute the gradient
        for (int i = 0; i < jointAngles.Length; i++)
        {
            // Create a perturbed copy of the joint angles for numerical gradient estimation
            float[] perturbedAngles = (float[])jointAngles.Clone();
            perturbedAngles[i] += 0.01f;  // Small perturbation for gradient estimation

            // Get the perturbed end-effector position
            Vector3 perturbedPosition = ForwardKinematics(perturbedAngles);

            // Compute the gradient as the change in position caused by the change in the joint angle
            Vector3 positionChange = perturbedPosition - currentPosition;
            float gradient = Vector3.Dot(positionChange, error.normalized) / 0.01f;  // Directional gradient

            // Update the joint angle based on the gradient
            jointAngles[i] -= learningRate * gradient;
        }
    }
    
    return jointAngles;
}
}
