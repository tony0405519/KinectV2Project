using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class Sponge : MonoBehaviour
{
    private Rigidbody m_rigidbody;
    private bool firstHit = true;
    private void Awake()
    {
        m_rigidbody = GetComponent<Rigidbody>();
        m_rigidbody.useGravity = false;
        m_rigidbody.isKinematic = true;
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
