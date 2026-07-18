# Research Project Outline: Socially Acceptable Mobile Robotics

## Brief Outline of Research Project
This research project focuses on the human-robot interaction (HRI) problem of social acceptance for mobile robots in public spaces. The deliverable is a multi-feature **"demo mode"** for the Boston Dynamics Spot robot that can be switched on and run autonomously. 

The demo consists of two distinct behaviors designed to make Spot appear approachable, entertaining, and less mechanical:
*   **Auto Dog Mode:** Spot moves through a space, perceives the people around it using onboard cameras, and responds with a friendly, dog-like "look-up" gesture—orienting and tilting its body upward to acknowledge them.
*   **Autonomous Dance Mode:** Spot utilizes its onboard microphones to listen to music, processes the audio stream to detect the beat/tempo in real-time, and autonomously strings together a sequence of dance moves synced to the music.

The work proceeds in three main technical and evaluative steps: building the vision-based perception and gaze behavior, developing the audio-based beat detection and dance choreography, and evaluating human perception.

---

## Main Task
To design and implement a dual-feature autonomous “demo mode” for Spot—featuring both a responsive dog-like attention gesture and a beat-synchronized dance routine—and to evaluate whether these interactive behaviors improve human perception of the robot.

---

## Project Technical Breakdown

### Step 1: Auto Dog Mode (Perception and Gaze Behavior)
*   Implement person detection on Spot’s onboard cameras, capturing the position of nearby people in the robot's field of view.
*   Estimate each detected person’s position and approximate height from the camera data.
*   Map that estimate to a dynamic body pose command using the Spot SDK (since Spot lacks an articulated head, gaze is simulated through the body).
*   Tune the detection pipeline so the behavior runs smoothly and reliably in real-time as Spot navigates.

### Step 2: Autonomous Dance Mode (Audio and Choreography)
*   Access and route the audio feed from Spot's onboard microphones.
*   Implement a real-time beat detection algorithm to extract the tempo (BPM) and rhythmic transients from ambient music.
*   Develop a library of "dance moves" (choreographed body poses and stepping patterns) using the Spot SDK.
*   Build a controller that maps the detected beats to the choreography, allowing Spot to autonomously select and execute dance moves in time with the music.
