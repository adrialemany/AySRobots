using UnityEngine;

public class Floater : MonoBehaviour
{
    public Rigidbody rb;

    [Header("Control de Profundidad")]
    [Tooltip("0 = Superficie. Valores positivos = Sumergido")]
    public float currentDepthOffset = 0f;

    [Header("Físicas del Agua")]
    public float buoyancySpring = 15f; // Fuerza con la que busca su nivel (el muelle)
    public float waterDamping = 5f;    // Freno para evitar que tiemble o rebote de golpe

    private void Awake()
    {
        if (rb == null) rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        if (WaveManager.instance == null) return;

        // Calculamos dónde está la ola y a qué profundidad deberíamos estar
        float waveHeight = WaveManager.instance.GetWaveHeight(transform.position.x);
        float desiredY = waveHeight - currentDepthOffset;
        
        // Distancia entre donde estamos y donde deberíamos estar
        float displacement = desiredY - transform.position.y;

        // Solo aplicamos la flotabilidad si estamos en el agua o justo asomando
        if (transform.position.y < waveHeight + 1f)
        {
            // 1. Muelle que empuja hacia la profundidad deseada
            float springForce = displacement * buoyancySpring;

            // 2. Amortiguador que evita el salto de delfín y los temblores
            float damperForce = -rb.linearVelocity.y * waterDamping;

            // 3. Compensación de gravedad para tener flotabilidad neutra bajo el agua
            float gravityCompensation = Mathf.Abs(Physics.gravity.y);

            Vector3 finalForce = new Vector3(0, springForce + damperForce + gravityCompensation, 0);

            rb.AddForce(finalForce, ForceMode.Acceleration);
        }
    }
}