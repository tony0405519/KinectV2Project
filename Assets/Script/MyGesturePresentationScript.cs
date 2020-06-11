using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(MyGestureListener))]
public class MyGesturePresentationScript : MonoBehaviour
{
    public ButtonController buttonController;
    [SerializeField]
    private SceneID curScene = SceneID.None;
    private MainMgr mainMgr;
    private MyGestureListener gestureListener;
    // Start is called before the first frame update
    void Start()
    {
        curScene = (SceneID)UnityEngine.SceneManagement.SceneManager.GetActiveScene().buildIndex;
        mainMgr = MainMgr.inst;
        gestureListener = GetComponent<MyGestureListener>();   
    }

    // Update is called once per frame
    void Update()
    {
        if (mainMgr == null)
            return;

        if(gestureListener.GetCurGesture == com.rfilkov.kinect.GestureType.RaiseRightHand)
        {
            if (curScene == SceneID.Start)
                mainMgr.changeScene(SceneID.Game);

            Debug.Log("RaiseRightHand detected.");
        }

        if (gestureListener.GetCurGesture == com.rfilkov.kinect.GestureType.Wave)
        {
            if (curScene == SceneID.Game && buttonController != null)
                buttonController.pause();

            Debug.Log("Wave detected.");
        }

    }
}
