
using UnityEngine;
using System.Threading;

public class GameMgr : MonoBehaviour {

    public UnityEngine.UI.Text scoreText;
    public GameObject gameOver;
    public GameObject[] answers;
    public Sponge[] spongePrefab;

    [Tooltip("position of sponge generate")]
    public Vector3 spongeInitPos;
    [Tooltip("position of answer showing")]
    public Quaternion spongeInitRot;
    [Tooltip("position of answer showing")]
    public Vector3 answerInitPos;
    [Tooltip("sponge moving speed")]
    public Vector3 movingDir;
    public float movingSpeed;

    public static GameMgr inst = null;

    private Sponge curSponge;
    private GameObject curAnswer = null;
    private int curID = -1;
    [SerializeField]
    public int score { get; private set; }

    private float time = 0f;
    private enum GameState {
        ready = 0,
        genSponge,
        moving,
        showAnswer,
        gameover
    }
    [SerializeField, Tooltip("For Debug")]
    private GameState curState = GameState.genSponge;

    void Awake()
    {
        if (inst == null)
        {
            inst = this;
        }
        else if (this != inst)
        {
            Destroy(gameObject);
        }
    }

    void Start() {
    }

    void Update() {

        if (MainMgr.isPause)
            return;

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
            case GameState.gameover:
                gameover();
                break;
        }
    }

    private void generateStage() {
        
        //random generate sponge from prefab
        int type = MainMgr.randomNumber(0, spongePrefab.Length);
        //curSponge = Instantiate(spongePrefab[type], spongeInitPos, Quaternion.identity);
        curSponge = Instantiate(spongePrefab[type], spongeInitPos, spongeInitRot);
        curID = type;
        //start moving after finish generate
        curState = GameState.moving;
    }

    private void moveSponge() {
        // keep moving sponge
        curSponge.transform.position += Vector3.Scale(movingDir, new Vector3(movingSpeed, movingSpeed, movingSpeed));
    }

    private void showAnswer() {
        if(curAnswer == null) {
            curAnswer = Instantiate(answers[curID], answerInitPos, Quaternion.identity);
        }

        if (Time.realtimeSinceStartup - time >= 3f)
        {
            if (score >= 0)
                closeAnswer();
            else
            {
                gameOver.SetActive(true);
                time = Time.realtimeSinceStartup;
                curState = GameState.gameover;
            }

        }

    }

    private void gameover()
    {
        if (Time.realtimeSinceStartup - time >= 3f)
        {
            Debug.Log("pass");
            gameOver.SetActive(false);
            MainMgr.inst.changeScene(SceneID.Start);
            curState = GameState.ready;
        }
        
    }

    //called when player hit by sponge
    public void playerHit() {
        getDamage();
        //delete sponge
        Destroy(curSponge.gameObject);

        //show answer
        time = Time.realtimeSinceStartup;
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
        score -= 10;
        scoreText.text = "SCORE: " + score.ToString();
        //TODO:
    }

    private void addPoint() {
        score += 10;
        scoreText.text = "SCORE: " + score.ToString();
        //TODO: 
    }
    
}
