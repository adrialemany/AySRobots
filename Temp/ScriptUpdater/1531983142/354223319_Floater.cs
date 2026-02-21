using UnityEngine;

public class Floater : MonoBehaviour
{
    public Rigidbody rigidbody;
    public float depthBeforesubmerged = 1f;
    public float displacementAmount = 3f;

    [Header("Físicas del Agua")]
    [Tooltip("Aumenta este valor si el robot salta demasiado al emerger")]
    public float underwaterDrag = 4f; 
    public float underwaterAngularDrag = 2f;

    // Variables para guardar la fricción del aire original
    private float defaultDrag;
    private float defaultAngularDrag;

    private void Awake()
    {
        // Asignar automáticamente el Rigidbody si no se configuró en el Inspector
        if (rigidbody == null)
        {
            rigidbody = GetComponent<Rigidbody>();
        }
    }

    private void Start()
    {
        // Guardamos cómo se comporta el robot en el aire
        if (rigidbody != null)
        {
            defaultDrag = rigidbody.linearDamping;
            defaultAngularDrag = rigidbody.angularDamping;
        }
    }

    private void FixedUpdate()
    {
        // Evitamos errores si el WaveManager aún no ha cargado
        if (WaveManager.instance == null) return;

        float waveHeight = WaveManager.instance.GetWaveHeight(transform.position.x);
        
        if (transform.position.y < waveHeight)
        {
            // Calculamos la fuerza de empuje hacia arriba
            float displacementMultiplier = Mathf.Clamp01((waveHeight - transform.position.y) / depthBeforesubmerged) * displacementAmount;
            rigidbody.AddForce(new Vector3(0, Mathf.Abs(Physics.gravity.y) * displacementMultiplier, 0), ForceMode.Acceleration);

            // --- EL SECRETO PARA NO SALTAR ---
            // Aplicamos resistencia densa (Drag) mientras esté tocando el agua.
            // Esto actúa como un freno natural y evita que acumule velocidad infinita.
            rigidbody.linearDamping = underwaterDrag;
            rigidbody.angularDamping = underwaterAngularDrag;
        }
        else
        {
            // --- EN EL AIRE ---
            // Si el robot sale completamente del agua, le devolvemos su comportamiento normal
            rigidbody.linearDamping = defaultDrag;
            rigidbody.angularDamping = defaultAngularDrag;
        }
    }
}