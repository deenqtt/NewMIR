<template>
  <div id="speed-control-container">
    <h3>Speed Control</h3>
    <div>
      <label for="linear-speed">Linear Speed:</label>
      <input
        type="range"
        id="linear-speed"
        v-model="linearSpeed"
        min="0"
        max="2"
        step="0.1"
      />
      <span>{{ linearSpeed }}</span>
    </div>
    <div>
      <label for="angular-speed">Angular Speed:</label>
      <input
        type="range"
        id="angular-speed"
        v-model="angularSpeed"
        min="0"
        max="2"
        step="0.1"
      />
      <span>{{ angularSpeed }}</span>
    </div>
    <button @click="setSpeed">Set Speed</button>
    <div id="speed-display">
      <p>Real-time Linear Speed: {{ realTimeLinearSpeed.toFixed(2) }} m/s</p>
      <p>
        Real-time Angular Speed: {{ realTimeAngularSpeed.toFixed(2) }} rad/s
      </p>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted } from "vue";
import * as ROSLIB from "roslib";

const linearSpeed = ref(1.0);
const angularSpeed = ref(1.0);
const realTimeLinearSpeed = ref(0);
const realTimeAngularSpeed = ref(0);

const ws = new WebSocket("ws://localhost:3000");

ws.onopen = () => {
  console.log("WebSocket connection established");
};

ws.onmessage = (message) => {
  console.log("Received message from server:", message.data);
};

ws.onclose = () => {
  console.log("WebSocket connection closed");
};

const setSpeed = () => {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(
      JSON.stringify({
        type: "set_speed",
        maxSpeed: linearSpeed.value,
        maxTurn: angularSpeed.value,
      })
    );
  }
};

onMounted(() => {
  const ros = new ROSLIB.Ros({
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

  const odomTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/odom",
    messageType: "nav_msgs/Odometry",
  });

  odomTopic.subscribe((message) => {
    realTimeLinearSpeed.value = message.twist.twist.linear.x;
    realTimeAngularSpeed.value = message.twist.twist.angular.z;
  });
});
</script>

<style scoped>
#speed-control-container {
  padding: 20px;
  border: 1px solid #ccc;
  border-radius: 10px;
  max-width: 300px;
  margin: 20px auto;
  background: #f9f9f9;
}

h3 {
  text-align: center;
}

div {
  margin-bottom: 15px;
}

label {
  display: block;
  margin-bottom: 5px;
}

input[type="range"] {
  width: 100%;
}

#speed-display {
  margin-top: 10px;
  color: #333;
  text-align: center;
}
</style>
