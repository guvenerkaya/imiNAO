#!/usr/bin/env node
'use strict'

/* const rosnodejs = require('rosnodejs')
const sensorMsgs = rosnodejs.require('sensor_msgs')
rosnodejs.initNode('/posenet')
.then(() => {
    const nh = rosnodejs.nh;
    const sub = nh.subscribe('/nao_robot/camera/bottom/camera/image_raw', sensorMsgs.msg.Image, (data) => {
        if (data.encoding == 'bgr8'){
            // Change the encoding to rgb8 
            // Atm not implemented, this means red is blue and vice versa which leads to worse results
            // data.data = swapChannels(data.data)
            data.encoding = 'rgb8';
        }
        console.log('Got msg on chatter: %j %j', data.height, data.width)
    })
})
*/

global.XMLHttpRequest = require("xhr2")
const assert = require("assert")
// Main requirements
const tf = require('@tensorflow/tfjs-core')
const rosnodejs = require('rosnodejs')
const stringify = require('json-stringify')
// Requires the std_msgs message and sensor_msgs packages
const sensor_msgs = rosnodejs.require('sensor_msgs').msg;
const StringMsg = rosnodejs.require('std_msgs').msg.String;
const pose_msgs = rosnodejs.require('perceptor').msg;
const PosesMsg = rosnodejs.require('perceptor').msg.Poses;

async function run() {
    const rosNode = await rosnodejs.initNode('/perceptor')
    // ROS function for simple recieveing node param
    const getParam = async function(key, default_value){
        if(await rosNode.hasParam(key)){
            const param = await rosNode.getParam(key)
            return param;
        }
        return default_value;
    }
    // Find if GPU is enabled and start tf
    const gpu = await getParam('gpu', false)
    if (gpu)
        require('@tensorflow/tfjs-core-gpu')
    else
        require('@tensorflow/tfjs-core')
    console.log(gpu)
    const posenet = require('@tensorflow-models/posenet')
    // lowest quality first
    const options = {
        architecture: 'MobileNetV1',
        outputStride: 16,
        inputResolution: { width: 320, height: 240 },
        multiplier: await getParam('multiplier', 0.5)
      }

    // This step requires internet connection as weights are loaded from google servers...
    // TODO download them offline
    const net  = await posenet.load(options)
    // Local variables for sync with ROS
    let buffer = [];
    let newBuffer = false;
    let image_width = 0;
    let image_height = 0;
    let header = null;
    // Parameters for posenet
    const imageScaleFactor = await getParam('image_scale_factor', 0.5)
    const flipHorizontal = await getParam('flip_horizontal', false)
    const outputStride = await getParam('output_stride', 16)
    const maxPoseDetections = await getParam('max_pose', 5)
    const scoreThreshold = await getParam('score_threshold', 0.5)
    const nmsRadius = await getParam('nms_radius', 20)
    const multiPerson = await getParam('multiPerson', false)
    // topic names
    const camera_topic = await getParam('topic','/nao_robot/camera/bottom/camera/image_raw')
    const output_topic = await getParam('poses_topic','/perceptor/poses')
    // ROS topics
    let pub = rosNode.advertise(output_topic, PosesMsg)
    let sub = rosNode.subscribe(camera_topic, sensor_msgs.Image,
        (data) => {
            // TODO more encodings
            if (data.encoding == 'bgr8'){
                // Change the encoding to rgb8 
                // Atm not implemented, this means red is blue and vice versa which leads to worse results
                // data.data = swapChannels(data.data)
                data.encoding = 'rgb8';
            }
            // Currently works only with rgb8 data
            assert(data.encoding == 'rgb8')
            buffer = data.data;
            newBuffer = true;
            header = data.header;
            image_height = data.height;
            image_width = data.width;
        })

    // Loop for detecting poses
    const DetectingPoses = async function (){
    if (newBuffer == false)  return
        let tensor = tf.tensor3d(buffer, [image_height,image_width,3], 'int32')
        newBuffer = false;
        let pose_msg = new pose_msgs.Poses()
        
        if (multiPerson === true) {
            const poses = await net.estimateMultiplePoses(tensor, {
                imageScaleFactor,
                flipHorizontal,
                outputStride,
                maxPoseDetections, 
                scoreThreshold,
                nmsRadius
            })

            for (let i = 0; i < poses.length; i++){
                pose_msg.poses.push(new pose_msgs.Pose())
                pose_msg.poses[i]["score"] = poses[i]["score"];
                for (let k = 0; k < poses[i]["keypoints"].length; k++){
                    pose_msg.poses[i].keypoints.push(new pose_msgs.Keypoint())
                    pose_msg.poses[i].keypoints[k].score = poses[i]["keypoints"][k]["score"];
                    pose_msg.poses[i].keypoints[k].part = poses[i]["keypoints"][k]["part"];
                    pose_msg.poses[i].keypoints[k].position.x = poses[i]["keypoints"][k]["position"]["x"];
                    pose_msg.poses[i].keypoints[k].position.y = poses[i]["keypoints"][k]["position"]["y"];
                }
            }           
        } else {
            const poses = await net.estimateSinglePose(tensor, {
                imageScaleFactor,
                flipHorizontal,
                outputStride}
            )
            pose_msg.poses.push(new pose_msgs.Pose())
            let i = 0;
            pose_msg.poses[i]["score"] = poses["score"];
            for (let k = 0; k < poses["keypoints"].length; k++){
                pose_msg.poses[i].keypoints.push(new pose_msgs.Keypoint())
                pose_msg.poses[i].keypoints[k].score = poses["keypoints"][k]["score"];
                pose_msg.poses[i].keypoints[k].part = poses["keypoints"][k]["part"];
                pose_msg.poses[i].keypoints[k].position.x = poses["keypoints"][k]["position"]["x"];
                pose_msg.poses[i].keypoints[k].position.y = poses["keypoints"][k]["position"]["y"];
            }
        }
        
        tensor.dispose()
        pub.publish(pose_msg)
    }
    setInterval(DetectingPoses, 10)
}

run()