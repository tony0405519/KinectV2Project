using System;
using System.Collections;
using System.Collections.Generic;
using System.Net.Sockets;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using Random = System.Random;

public enum SceneID : int {
    None = -1,
    Start,
    Game,
    End
};

public class MainMgr : MonoBehaviour {

    public static MainMgr inst = null;

    [HideInInspector]
    static public bool isPause = false;

    SceneID curScene = SceneID.None;

    static readonly Random rnd = new Random();

    void Awake() {
        if (inst == null) {
            inst = this;
            DontDestroyOnLoad(this);
        } else if (this != inst) {
            Destroy(gameObject);
        }
    }

    void Start() {
        
    }

    public static int randomNumber(int min, int max) {
        return rnd.Next(min, max);
    }

    public void changeScene(SceneID sceneID) {
        SceneManager.LoadScene((int)sceneID);
        MainMgr.isPause = false;
        curScene = sceneID;
    }

    SceneID getCurScenID(Scene scene) {
        int sceneID = scene.buildIndex;
        switch (sceneID) {
            case 0:
                return SceneID.Start;
            case 1:
                return SceneID.Game;
            case 2:
                return SceneID.End;
        }
        return SceneID.None;
    }

    void OnSceneLoaded(Scene scene, LoadSceneMode mode) {
        SceneID id = getCurScenID(scene);
        switch (id) {
            case SceneID.Start:
                break;
            case SceneID.Game:
                break;
            default:
                Debug.Log("[Scene] goto " + id + " scene");
                break;
        }
    }
    

}
