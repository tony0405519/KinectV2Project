using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(MyGestureListener))]
public class MyGesturePresentationScript : MonoBehaviour
{
    private MainMgr mainMgr;
    private MyGestureListener gestureListener;
    // Start is called before the first frame update
    void Start()
    {
        mainMgr = MainMgr.inst;
        gestureListener = GetComponent<MyGestureListener>();   
    }

    // Update is called once per frame
    void Update()
    {
        if (mainMgr == null)
            return;

        if(gestureListener.GetCurGesture == com.rfilkov.kinect.GestureType.Tpose)
        {
            var curScene = UnityEngine.SceneManagement.SceneManager.GetActiveScene();
            if (curScene.buildIndex == (int)SceneID.Start)
                mainMgr.changeScene(SceneID.Game);

            Debug.Log("Tpose detected.");
        }
    }
}
