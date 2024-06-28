<template>
  <div>
    <div class="d-flex justify-content-between align-items-center mb-3">
      <div class="form-group flex-grow-1 mr-2">
        <select v-model="selectedMap" :disabled="loading || stopping">
          <option v-for="map in maps" :key="map.id" :value="map.name">
            {{ map.name }}
          </option>
        </select>
        <button @click="launchMap" :disabled="loading || stopping">
          Launch Map
        </button>
        <button @click="initializeRobot">Initialize Robot</button>
        <label for="pathSelect">Select Path</label>
        <select class="form-control" id="pathSelect" v-model="selectedPath">
          <option v-for="path in paths" :key="path.id" :value="path">
            {{ path.name }}
          </option>
        </select>
      </div>
      <button class="btn btn-primary" @click="startNavigate">
        Start Navigate
      </button>
    </div>
    <div class="card bg-light mt-3">
      <div class="card-body" id="nav"></div>
    </div>

    <!-- Select Mission -->
    <div class="mt-3">
      <h3>Select Mission</h3>
      <select v-model="selectedMission">
        <option v-for="mission in missions" :key="mission.id" :value="mission">
          {{ mission.name }}
        </option>
      </select>
      <button @click="startMission" class="btn btn-primary mb-2">
        Start Mission
      </button>
      <button
        class="btn btn-danger"
        @click="stopMission"
        :disabled="!selectedMission"
      >
        Stop Mission
      </button>
      <button @click="continueMission" class="btn btn-secondary mb-2">
        Continue Mission
      </button>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, computed } from "vue";
import { useStore } from "vuex";
import axios from "axios";
import ROSLIB from "roslib";

// Vuex Store
const store = useStore();
const robots = computed(() => store.state.robots);
const connectedRobots = computed(() => store.state.connectedRobots);

// Define navigatorInstance using ref
const navigatorInstance = ref(null);
const navClient = ref(null);
const selectedMap = ref(null);
const selectedPath = ref(null);
const selectedMission = ref(null);
const paths = ref([]);
const loading = ref(false);
const stopping = ref(false);
const connected = ref(false);
const viewer = ref(null);
const nav = ref(null);
const rosSocket = ref(null); // WebSocket for ROS
const ws = ref(null); // WebSocket for goal
const maps = ref([]);
const missions = ref([]); // List of missions
const currentPose = ref(null);
const poseListener = ref(null);
const initializePosition = ref(null);
const message = ref(""); // Added message ref
const mapLaunched = ref(false); // Added mapLaunched ref
const showContinueButton = ref(false); // Added showContinueButton ref

const fetchMaps = async () => {
  try {
    const response = await axios.get("http://localhost:5258/maps");
    maps.value = response.data;
    console.log("Fetched maps:", maps.value);
  } catch (error) {
    console.error("Error fetching maps:", error);
  }
};

const launchMap = async () => {
  if (!selectedMap.value) {
    alert("Please select a map");
    return;
  }
  try {
    loading.value = true;
    const formData = new FormData();
    formData.append("mapName", selectedMap.value);
    const response = await axios.post(
      "http://localhost:5258/maps/launch",
      formData
    );
    alert(response.data);
    const connectedRobot = robots.value.find(
      (robot) => connectedRobots.value[robot.id]
    );
    if (connectedRobot) {
      mapView(connectedRobot);
    }
  } catch (error) {
    console.error("Error launching map:", error);
  } finally {
    loading.value = false;
  }
};

const fetchPaths = async () => {
  try {
    const response = await axios.get("http://localhost:5258/paths");
    paths.value = response.data;
  } catch (error) {
    console.error("Error fetching paths:", error);
  }
};

const startNavigate = () => {
  if (!selectedPath.value) {
    alert("Please select a path");
    return;
  }
  const goal = {
    type: "send_goal",
    x: parseFloat(selectedPath.value.posX),
    y: parseFloat(selectedPath.value.posY),
    z: 0.0,
    w: parseFloat(selectedPath.value.orientation),
  };
  console.log("Sending goal to server:", goal);
  ws.value.send(JSON.stringify(goal));
};

const initConnection = () => {
  // Check if there is any connected robot
  const connectedRobot = robots.value.find(
    (robot) => connectedRobots.value[robot.id]
  );
  if (connectedRobot) {
    rosSocket.value = new WebSocket(
      `ws://${connectedRobot.ip}:${connectedRobot.port}`
    );
    rosSocket.value.onopen = () => {
      console.log("ROS2 WebSocket connection established");
      connected.value = true;
      mapView(connectedRobot);
    };
    rosSocket.value.onerror = (error) => {
      console.error("ROS2 WebSocket error: ", error);
      setTimeout(initConnection, 5000);
    };
  }
  ws.value = new WebSocket(`ws://localhost:3000`);
  ws.value.onopen = () => {
    console.log("Goal WebSocket connection established");
  };
  ws.value.onmessage = (message) => {
    const data = JSON.parse(message.data);
    if (data.type === "goal_result") {
      alert(`Navigation ${data.result}`);
    } else if (data.type === "goal_error") {
      alert(`Error: ${data.error}`);
    }
  };
  ws.value.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    if (msg.type === "position_reached") {
      message.value = `Reached ${msg.position}`;
      if (msg.position === "start" || msg.position === "goal") {
        showContinueButton = true;
      }
    } else if (msg.type === "goal_error") {
      message.value = `Error: ${msg.error}`;
    }
  };
};

const mapView = (connectedRobot) => {
  if (rosSocket.value && rosSocket.value.readyState === WebSocket.OPEN) {
    const navElement = document.getElementById("nav");
    if (!navElement) {
      console.error("Element with ID 'nav' not found in DOM.");
      return;
    }
    viewer.value = new ROS2D.Viewer({
      divID: "nav",
      width: 600,
      height: 600,
    });

    navClient.value = new NAV2D.OccupancyGridClientNav({
      ros: new ROSLIB.Ros({
        url: `ws://${connectedRobot.ip}:${connectedRobot.port}`,
      }),
      rootObject: viewer.value.scene,
      viewer: viewer.value,
      serverName: "/navigate_to_pose",
      topic: "/map",
    });

    navigatorInstance.value = new NAV2D.Navigator({
      ros: new ROSLIB.Ros({
        url: `ws://${connectedRobot.ip}:${connectedRobot.port}`,
      }),
      rootObject: viewer.value.scene,
      viewer: viewer.value,
      serverName: "/navigate_to_pose",
      withOrientation: true,
    });

    poseListener.value = new ROSLIB.Topic({
      ros: new ROSLIB.Ros({
        url: `ws://${connectedRobot.ip}:${connectedRobot.port}`,
      }),
      name: "/amcl_pose",
      messageType: "geometry_msgs/PoseWithCovarianceStamped",
    });

    poseListener.value.subscribe((message) => {
      currentPose.value = message.pose.pose;
      console.log("Current Pose:", currentPose.value);
    });
    viewer.value.scene.addEventListener("click", (event) => {
      const mousePosition = viewer.value.scene.globalToRos(
        event.stageX,
        event.stageY
      );
      console.log("Map clicked at:", mousePosition);

      if (!initializePosition.value) {
        ws.value.send(
          JSON.stringify({
            type: "initialize_robot",
            x: mousePosition.x,
            y: mousePosition.y,
            z: 0.0,
            w: 1.0,
          })
        );
      }
    });
    console.log("Nav setup complete");
  }
};

// Event handler untuk tombol Initialize Robot
const initializeRobot = () => {
  if (navigatorInstance.value && navigatorInstance.value.initializeRobotMode) {
    navigatorInstance.value.initializeRobotMode();
  } else {
    console.error("NAV2D.Navigator belum diinisialisasi");
  }
};

const fetchMissions = async () => {
  try {
    const response = await axios.get("http://localhost:5258/missions/all");
    missions.value = response.data;
  } catch (error) {
    console.error("Error fetching missions:", error);
  }
};

const startMission = () => {
  if (!selectedMission.value) {
    alert("Please select a mission");
    return;
  }

  const mission = {
    type: "start_mission",
    startX: selectedMission.value.startX,
    startY: selectedMission.value.startY,
    startOrientation: selectedMission.value.startOrientation,
    goalX: selectedMission.value.goalX,
    goalY: selectedMission.value.goalY,
    goalOrientation: selectedMission.value.goalOrientation,
  };

  if (ws.value && connected.value) {
    ws.value.send(JSON.stringify(mission));
    console.log("Mission started:", mission);
  } else {
    console.error("WebSocket not connected");
  }
};

const continueMission = () => {
  ws.value.send(
    JSON.stringify({
      type: "continue_mission",
    })
  );
};

const stopMission = () => {
  if (ws.value && connected.value) {
    ws.value.send(JSON.stringify({ type: "stop_mission" }));
    console.log("Mission stopped");
  } else {
    console.error("WebSocket not connected");
  }
};

onMounted(() => {
  fetchMaps();
  fetchPaths();
  fetchMissions();
  initConnection();
});
</script>

<style scoped>
/* Add some styling if necessary */
</style>
