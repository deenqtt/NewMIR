<template>
  <div id="joystick-container">
    <div ref="joystick" id="joystick"></div>
    <div id="speed-display">
      <p>Linear Speed: {{ linearSpeed.toFixed(2) }} m/s</p>
      <p>Angular Speed: {{ angularSpeed.toFixed(2) }} rad/s</p>
    </div>
  </div>
</template>

<script setup>
import { onMounted, ref } from "vue";
import nipplejs from "nipplejs";
import * as ROSLIB from "roslib";

const joystick = ref(null);
const linearSpeed = ref(0);
const angularSpeed = ref(0);
let ros = null;
let cmdVelTopic = null;
let maxSpeed = ref(0.5);
let maxTurn = ref(1.0);

// Initialize WebSocket connection to listen for speed updates
const ws = new WebSocket("ws://localhost:3000");

ws.onopen = () => {
  console.log("WebSocket connection established");
};

ws.onmessage = (message) => {
  const msg = JSON.parse(message.data);
  if (msg.type === "set_speed") {
    maxSpeed.value = msg.maxSpeed;
    maxTurn.value = msg.maxTurn;
    console.log("Updated max speed settings:", msg);
  }
};

ws.onclose = () => {
  console.log("WebSocket connection closed");
};

onMounted(() => {
  // Initialize ROS connection
  ros = new ROSLIB.Ros({
    url: "ws://localhost:9090", // Adjust to your ROS bridge websocket URL
  });

  ros.on("connection", () => {
    console.log("Connected to websocket server.");
  });

  ros.on("error", (error) => {
    console.log("Error connecting to websocket server: ", error);
  });

  ros.on("close", () => {
    console.log("Connection to websocket server closed.");
  });

  // Define the /cmd_vel topic
  cmdVelTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/cmd_vel",
    messageType: "geometry_msgs/Twist",
  });

  // Subscribe to /odom topic to get real-time speed
  const odomTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/odom",
    messageType: "nav_msgs/Odometry",
  });

  odomTopic.subscribe((message) => {
    linearSpeed.value = message.twist.twist.linear.x;
    angularSpeed.value = message.twist.twist.angular.z;
  });

  const joystickManager = nipplejs.create({
    zone: joystick.value,
    mode: "static",
    position: { left: "50%", top: "50%" },
    color: "blue",
    size: 100,
  });

  joystickManager.on("move", (evt, data) => {
    // Calculate the velocities
    const linearVel = maxSpeed.value * data.vector.y;
    const angularVel = maxTurn.value * -data.vector.x; // Invert the x-axis for correct direction

    // Create and publish the Twist message
    const twist = new ROSLIB.Message({
      linear: {
        x: linearVel,
        y: 0.0,
        z: 0.0,
      },
      angular: {
        x: 0.0,
        y: 0.0,
        z: angularVel,
      },
    });

    cmdVelTopic.publish(twist);
  });

  joystickManager.on("end", () => {
    // Stop the robot when the joystick is released
    const twist = new ROSLIB.Message({
      linear: {
        x: 0.0,
        y: 0.0,
        z: 0.0,
      },
      angular: {
        x: 0.0,
        y: 0.0,
        z: 0.0,
      },
    });

    cmdVelTopic.publish(twist);
  });
});
</script>

<style scoped>
#joystick-container {
  width: 200px;
  height: 300px;
  position: relative;
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  background-color: #333;
}

#joystick {
  width: 200px;
  height: 200px;
  position: relative;
}

#speed-display {
  margin-top: 10px;
  color: white;
}
</style>
