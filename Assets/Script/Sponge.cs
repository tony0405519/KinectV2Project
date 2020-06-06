using UnityEngine;

[RequireComponent(typeof(MeshCollider), typeof(MeshFilter), typeof(Rigidbody))]
public class Sponge : MonoBehaviour
{
    private MeshCollider m_meshCollider;
    private MeshFilter m_meshFilter;
    private void Awake()
    {
        m_meshCollider = GetComponent<MeshCollider>();
        m_meshFilter= GetComponent<MeshFilter>();
        GetComponent<Rigidbody>().useGravity = false;
        GetComponent<Rigidbody>().isKinematic = true;
    }

    void Start()
    {
        //m_Collider.isTrigger = true;
    }

    private void OnTriggerEnter(Collider other)
    {
        Debug.Log(name + " hit " + other.name);
        if (other.tag == "Player")
        {
            Debug.Log("Player hit");
            GameMgr.inst.playerHit();
            Destroy(gameObject);
        }
    }
    private void OnCollisionEnter(Collision collision)
    {
        Debug.Log(name + " collision " + collision.collider.name);
    }
}
