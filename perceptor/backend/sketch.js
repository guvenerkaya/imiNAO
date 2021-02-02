/* === 
ml5 Example PoseNet example using p5.js

Available parts are:
0   nose
1	leftEye
2	rightEye
3	leftEar
4	rightEar
5	leftShoulder
6	rightShoulder
7	leftElbow
8	rightElbow
9	leftWrist
10	rightWrist
11	leftHip
12	rightHip
13	leftKnee
14	rightKnee
15	leftAnkle
16	rightAnkle
=== */

let poseNet;
let poses = [];
let img;
const options = {
    flipHorizontal: false,
    maxDetections: 1,
    scoreThreshold: 0.8,
    nmsRadius: 30
}

function setup() {
    createCanvas(320, 240);

    // create an image using the p5 dom library
    // call modelReady() when it is loaded
    img = createImg('data/bg.png', imageReady)
    // set the image size to the size of the canvas
    img.size(width, height)

    img.hide(); // hide the image in the browser
    frameRate(5); // set the frameRate to 1 since we don't need it to be running quickly in this case
}


// when the image is ready, then load up PoseNet
function imageReady(){
    // Create a new poseNet method
    const stream = document.getElementById("stream"); 
    poseNet = ml5.poseNet(stream, options, modelReady);

    // Listen to new 'pose' events
    poseNet.on("pose", function(results) {
        poses = results
        console.log(poses)
        draw(poses)
    });
}


// when poseNet is ready, do the detection
function modelReady() {
    select('#status').html('Model Loaded')
     
    // When the model is ready, run the singlePose() function...
    // If/When a pose is detected, poseNet.on('pose', ...) will be listening for the detection results 
    // in the draw() loop, if there are any poses, then carry out the draw commands
    //poseNet.singlePose(img)
}


// draw() will not show anything until poses are found
function draw(poses) {
    if (poses.length > 0) {
        image(img, 0, 0, width, height)
        drawSkeleton(poses)
        drawKeypoints(poses)
        noLoop() // stop looping when the poses are estimated
    }

}

// The following comes from https://ml5js.org/docs/posenet-webcam
// A function to draw ellipses over the detected keypoints

function drawKeypoints(poses) {
    // Loop through all the poses detected
    for (let i = 0; i < poses.length; i++) {
        // For each pose detected, loop through all the keypoints
        let pose = poses[i].pose
        for (let j = 0; j < pose.keypoints.length; j++) {
            // A keypoint is an object describing a body part (like rightArm or leftShoulder)
            let keypoint = pose.keypoints[j]
            // Only draw an ellipse is the pose probability is bigger than 0.2
            if (keypoint.score > 0.2) {
                fill(255)
                stroke(20)
                strokeWeight(4)
                ellipse(round(keypoint.position.x), round(keypoint.position.y), 8, 8)
            }
        }
    }
}

// A function to draw the skeletons
function drawSkeleton(poses) {
    // Loop through all the skeletons detected
    for (let i = 0; i < poses.length; i++) {
        let skeleton = poses[i].skeleton
        // For every skeleton, loop through all body connections
        for (let j = 0; j < skeleton.length; j++) {
            let partA = skeleton[j][0]
            let partB = skeleton[j][1]
            stroke(255)
            strokeWeight(2)
            line(partA.position.x, partA.position.y, partB.position.x, partB.position.y)
        }
    }
}