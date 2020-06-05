using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ButtonController : MonoBehaviour
{
    
    public void start()
    {
        MainMgr.inst.changeScene(SceneID.Game);
    }

    public GameObject pauseMenu;

    public void pause()
    {
        pauseMenu.SetActive(true);
    }

    public void resume()
    {
        pauseMenu.SetActive(false);
    }

    public void restart()
    {
        MainMgr.inst.changeScene(SceneID.Start);
    }

    public GameObject soundButton;
    public GameObject bgm;

    public void soundOff()
    {
        soundButton.SetActive(true);
        bgm.SetActive(false);
    }

    public void soundOn()
    {
        soundButton.SetActive(false);
        bgm.SetActive(true);
    }
}
