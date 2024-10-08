using UnityEngine;

public class CollisionChecker : MonoBehaviour
{
    public bool CheckCollision(Vector3 endEffectorPosition, float radius)
    {
        // Perform a sphere check at the end-effector's position
        return Physics.CheckSphere(endEffectorPosition, radius);
    }
}
