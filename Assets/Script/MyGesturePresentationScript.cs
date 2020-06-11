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
        switch (curScene)
        {
            case SceneID.Start:
                if (gestureListener.GetCurGesture == com.rfilkov.kinect.GestureType.RaiseRightHand)
                {
                    mainMgr.changeScene(SceneID.Game);
                    Debug.Log("RaiseRightHand detected.");
                }
                break;
            case SceneID.Game:
                switch (gestureListener.GetCurGesture)
                {
                    case com.rfilkov.kinect.GestureType.Wave:
                        if (MainMgr.isPause)
                            buttonController.resume();
                        else if(GameMgr.inst.score >= 0)
                            buttonController.pause();
                        break;
                    case com.rfilkov.kinect.GestureType.RaiseLeftHand:
                        if(MainMgr.isPause)
                            buttonController.restart();
                        break;
                    case com.rfilkov.kinect.GestureType.SwipeDown:
                        if (MainMgr.isPause && !buttonController.soundButton.activeInHierarchy)
                            buttonController.soundOff();
                        break;
                    case com.rfilkov.kinect.GestureType.SwipeUp:
                        if (MainMgr.isPause && buttonController.soundButton.activeInHierarchy)
                            buttonController.soundOn();
                        break;
                }
                if (gestureListener.GetCurGesture == com.rfilkov.kinect.GestureType.Wave)
                {
                    if (curScene == SceneID.Game && buttonController != null)
                        buttonController.pause();

                    Debug.Log("Wave detected.");
                }
                if (gestureListener.GetCurGesture == com.rfilkov.kinect.GestureType.RaiseRightHand)
                {
                    mainMgr.changeScene(SceneID.Game);
                    Debug.Log("RaiseRightHand detected.");
                }
                break;

        }


    }
}
