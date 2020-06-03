using UnityEngine;

public class Sponge : MonoBehaviour {
    
    private BoxCollider m_Collider;
    
    void Start() {
        //m_Collider.isTrigger = true;
    }

    private void OnTriggerEnter(Collider other)
    {
        Debug.Log("is Trigger----");
        if (other.tag == "Player")
        {
            Debug.Log("is Trigger");
            GameMgr.inst.playerHit();
        }
    }



}
