
using UnityEngine;

public class GameMgr : MonoBehaviour {

    public GameObject[] answers;
    public Sponge[] spongePrefab;
    [Tooltip("position of sponge generate")]
    public Vector3 spongeInitPos;
    [Tooltip("position of answer showing")]
    public Vector3 answerInitPos;
    [Tooltip("sponge moving speed")]
    public float movingSpeed;

    private Sponge curSponge;
    private GameObject curAnswer = null;
    private int curID = -1;

    private enum GameState {
        ready = 0,
        genSponge,
        moving,
        showAnswer
    }
    private GameState curState = GameState.ready;

    void Start() {
        
    }

    void Update() {
        //game thread
        switch (curState) {
            case GameState.ready:
                //TODO:
                break;
            case GameState.genSponge:
                generateStage();
                break;
            case GameState.moving:
                moveSponge();
                break;
            case GameState.showAnswer:
                showAnswer();
                break;
        }
    }

    private void generateStage() {
        
        //random generate sponge from prefab
        int type = MainMgr.randomNumber(0, spongePrefab.Length);
        curSponge = Instantiate(spongePrefab[type], spongeInitPos, Quaternion.identity);
        curID = type;
        //start moving after finish generate
        curState = GameState.moving;
    }

    private void moveSponge() {
        // keep moving sponge
        Vector3 movingDir = new Vector3(0, 0, 1);
        curSponge.transform.position += Vector3.Scale(movingDir, new Vector3(movingSpeed, movingSpeed, movingSpeed));
    }

    private void showAnswer() {
        if(curAnswer == null) {
            curAnswer = Instantiate(answers[curID], answerInitPos, Quaternion.identity);
        }
    }

    //called when player hit by sponge
    public void playerHit() {
        getDamage();
        //show answer
        curState = GameState.showAnswer;
    }
    
    //called when player pass sponge success
    public void passStage() {
        addPoint();
        //delete sponge
        Destroy(curSponge.gameObject);

        //next sponge
        curState = GameState.genSponge;
    }

    //close answer & goto next sponge
    public void  closeAnswer() {
        Destroy(curAnswer);
        curAnswer = null;

        curState = GameState.genSponge;
    }

    public void startGame() {
        curState = GameState.genSponge;
    }

    private void getDamage() {
        //TODO:
    }

    private void addPoint() {
        //TODO:
    }



}
