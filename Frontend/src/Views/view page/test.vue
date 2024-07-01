<template>
  <div>
    <h3>Robot Speed Settings</h3>
    <div>
      <label for="max_vel_x">Max Linear Velocity (m/s)</label>
      <input type="number" v-model="maxVelX" id="max_vel_x" step="0.1" />
    </div>
    <div>
      <label for="max_vel_theta">Max Angular Velocity (rad/s)</label>
      <input
        type="number"
        v-model="maxVelTheta"
        id="max_vel_theta"
        step="0.1"
      />
    </div>
    <button @click="setSpeed">Set Speed</button>
  </div>
</template>

<script setup>
import { ref } from "vue";
import { useWebSocket } from "./webSocket"; // Your websocket utility

const maxVelX = ref(0.3);
const maxVelTheta = ref(1.0);

const { sendMessage } = useWebSocket();

const setSpeed = () => {
  sendMessage({
    type: "set_speed",
    max_vel_x: maxVelX.value,
    max_vel_theta: maxVelTheta.value,
  });
};
</script>
