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
  </div>
</template>

<script setup>
import { ref } from "vue";

const linearSpeed = ref(1.0);
const angularSpeed = ref(1.0);

const setSpeed = () => {
  // Send the new speed settings to the backend
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(
      JSON.stringify({
        type: "set_speed",
        maxSpeed: linearSpeed.value,
        maxTurn: angularSpeed.value,
      })
    );
    console.log("Sent speed settings to server:", {
      maxSpeed: linearSpeed.value,
      maxTurn: angularSpeed.value,
    });
  }
};

// Initialize WebSocket connection
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
</style>
