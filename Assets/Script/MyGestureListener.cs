using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using com.rfilkov.kinect;

public class MyGestureListener : MonoBehaviour, GestureListenerInterface
{
    [Tooltip("Index of the player, tracked by this component. 0 means the 1st player, 1 - the 2nd one, 2 - the 3rd one, etc.")]
    public int playerIndex = 0;

    [Tooltip("List of the gestures to detect.")]
    public List<GestureType> detectGestures = new List<GestureType>();

    [Tooltip("UI-Text to display the gesture-listener output.")]
    public UnityEngine.UI.Text gestureInfo;

    // private bool to track if progress message has been displayed
    private bool progressDisplayed;
    private float progressGestureTime;

    private GestureType curGesture;
    public GestureType GetCurGesture { get { return curGesture; } }

    private void Start()
    {
        com.rfilkov.kinect.KinectManager.Instance.gestureManager.RefreshGestureListeners();
    }
    // invoked when a new user is detected
    public void UserDetected(ulong userId, int userIndex)
    {
        if (userIndex == playerIndex)
        {
            // as an example - detect these user specific gestures
            KinectGestureManager gestureManager = KinectManager.Instance.gestureManager;

            foreach (GestureType gesture in detectGestures)
            {
                gestureManager.DetectGesture(userId, gesture);
            }
        }

        if (gestureInfo != null)
        {
            gestureInfo.text = "Please do the gestures and look for the gesture detection state.";
        }
    }


    // invoked when the user is lost
    public void UserLost(ulong userId, int userIndex)
    {
        if (userIndex != playerIndex)
            return;

        if (gestureInfo != null)
        {
            gestureInfo.text = string.Empty;
        }
    }


    // invoked to report gesture progress. important for the continuous gestures, because they never complete.
    public void GestureInProgress(ulong userId, int userIndex, GestureType gesture,
                                  float progress, KinectInterop.JointType joint, Vector3 screenPos)
    {
        if (userIndex != playerIndex)
            return;

        // check for continuous gestures
        switch (gesture)
        {
            case GestureType.ZoomOut:
            case GestureType.ZoomIn:
                if (progress > 0.5f && gestureInfo != null)
                {
                    curGesture = gesture;

                    string sGestureText = string.Format("{0} - {1:F0}%", gesture, screenPos.z * 100f);
                    gestureInfo.text = sGestureText;

                    progressDisplayed = true;
                    progressGestureTime = Time.realtimeSinceStartup;
                }
                else
                {
                    curGesture = GestureType.None;
                }
                break;

            case GestureType.Wheel:
            case GestureType.LeanLeft:
            case GestureType.LeanRight:
            case GestureType.LeanForward:
            case GestureType.LeanBack:
                if (progress > 0.5f && gestureInfo != null)
                {
                    curGesture = gesture;

                    string sGestureText = string.Format("{0} - {1:F0} degrees", gesture, screenPos.z);
                    gestureInfo.text = sGestureText;

                    progressDisplayed = true;
                    progressGestureTime = Time.realtimeSinceStartup;
                }
                else
                {
                    curGesture = GestureType.None;
                }
                break;
            case GestureType.Run:
                if (progress > 0.5f && gestureInfo != null)
                {
                    curGesture = gesture;

                    string sGestureText = string.Format("{0} - progress: {1:F0}%", gesture, progress * 100);
                    gestureInfo.text = sGestureText;

                    progressDisplayed = true;
                    progressGestureTime = Time.realtimeSinceStartup;
                }
                else
                {
                    curGesture = GestureType.None;
                }
                break;
        }
    }


    // invoked when a (discrete) gesture is complete.
    public bool GestureCompleted(ulong userId, int userIndex, GestureType gesture,
                                  KinectInterop.JointType joint, Vector3 screenPos)
    {
        if (userIndex != playerIndex)
            return false;

        if (progressDisplayed)
            return true;

        curGesture = gesture;
        string sGestureText = gesture + " detected";
        if (gestureInfo != null)
        {
            gestureInfo.text = sGestureText;
        }

        return true;
    }


    // invoked when a gesture gets cancelled by the user
    public bool GestureCancelled(ulong userId, int userIndex, GestureType gesture,
                                  KinectInterop.JointType joint)
    {
        if (userIndex != playerIndex)
            return false;

        if (curGesture == gesture)
            curGesture = GestureType.None;

        if (progressDisplayed)
        {
            progressDisplayed = false;

            if (gestureInfo != null)
            {
                gestureInfo.text = String.Empty;
            }
        }

        return true;
    }


    public void Update()
    {
        // checks for timed out progress message
        if (progressDisplayed && ((Time.realtimeSinceStartup - progressGestureTime) > 2f))
        {
            curGesture = GestureType.None;
            progressDisplayed = false;

            if (gestureInfo != null)
            {
                gestureInfo.text = String.Empty;
            }

            Debug.Log("Forced progress to end.");
        }
    }

}
