using UnityEngine;

[RequireComponent(typeof(MeshCollider), typeof(MeshFilter), typeof(Rigidbody))]
public class Sponge : MonoBehaviour
{
    private MeshCollider m_meshCollider;
    private MeshFilter m_meshFilter;
    private bool firstHit = true;
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
        if (other.tag == "Player" && firstHit)
        {
            firstHit = false;
            Debug.Log("Player hit");
            GameMgr.inst.playerHit();
            //Destroy(gameObject);
        }
        else if(other.tag == "MainCamera" && firstHit)
        {
            firstHit = false;
            Debug.Log("Pass Stage");
            GameMgr.inst.passStage();
            //Destroy(gameObject);
        }
    }
    private void OnCollisionEnter(Collision collision)
    {
        Debug.Log(name + " collision " + collision.collider.name);
    }
}
