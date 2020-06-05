using UnityEngine;

[RequireComponent(typeof(MeshCollider), typeof(MeshFilter))]
public class Sponge : MonoBehaviour
{
    private MeshCollider m_meshCollider;
    private MeshFilter m_meshFilter;
    private void Awake()
    {
        m_meshCollider = GetComponent<MeshCollider>();
        m_meshFilter= GetComponent<MeshFilter>();
        m_meshCollider.convex = true;
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
}
