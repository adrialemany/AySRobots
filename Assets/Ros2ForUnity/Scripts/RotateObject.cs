using UnityEngine;

public class RotateObject : MonoBehaviour
{
    [Header("Rotation Settings")]
    [Tooltip("Velocidad de rotación en grados por segundo.")]
    public float rotationSpeed = 50f;

    void Update()
    {
        // Calcula la rotación en función del tiempo y la velocidad
        float rotationAmount = rotationSpeed * Time.deltaTime;
        
        // Aplica la rotación al objeto en el eje Y
        transform.Rotate(0, rotationAmount, 0);
    }
}
