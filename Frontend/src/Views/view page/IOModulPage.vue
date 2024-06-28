<template>
  <div>
    <h3>Navigation Map</h3>
    <select v-model="selectedMap" :disabled="loading || stopping">
      <option v-for="map in maps" :key="map.id" :value="map.name">
        {{ map.name }}
      </option>
    </select>
    <button @click="launchMap" :disabled="loading || stopping">
      Launch Map
    </button>
    <button @click="stopLaunch">Stop Launch</button>
    <div id="nav" style="width: 660px; height: 550px"></div>
    <button @click="initializeRobot">Initialize Robot</button>
    <button @click="activateNavigation">Navigate</button>
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

// States
const ws = ref(null);
const viewer = ref(null);
const navClient = ref(null);
const navigatorInstance = ref(null);
const maps = ref([]);
const selectedMap = ref(null);
const loading = ref(false);
const stopping = ref(false);

// Methods
const fetchMaps = async () => {
  try {
    const response = await axios.get("http://localhost:5258/maps");
    maps.value = response.data;
    console.log("Fetched maps:", maps.value);
  } catch (error) {
    console.error("Error fetching maps:", error);
  }
};

const initConnection = () => {
  // Check if there is any connected robot
  const connectedRobot = robots.value.find(
    (robot) => connectedRobots.value[robot.id]
  );
  if (connectedRobot) {
    ws.value = new WebSocket(
      `ws://${connectedRobot.ip}:${connectedRobot.port}`
    );

    ws.value.onopen = () => {
      console.log("WebSocket connection established");
      mapView(connectedRobot); // Pass the connectedRobot to the mapView function
    };

    ws.value.onclose = () => {
      console.log("Connection closed");
      setTimeout(initConnection, 5000);
    };

    ws.value.onerror = (error) => {
      console.log("WebSocket error: ", error);
      setTimeout(initConnection, 5000);
    };
  }
};

const mapView = (connectedRobot) => {
  if (ws.value && ws.value.readyState === WebSocket.OPEN) {
    const navElement = document.getElementById("nav");
    if (!navElement) {
      console.error("Element with ID 'nav' not found in DOM.");
      return;
    }

    viewer.value = new ROS2D.Viewer({
      divID: "nav",
      width: 660,
      height: 550,
    });

    // Initialize the map viewer
    navClient.value = new NAV2D.OccupancyGridClientNav({
      ros: new ROSLIB.Ros({
        url: `ws://${connectedRobot.ip}:${connectedRobot.port}`,
      }),
      rootObject: viewer.value.scene,
      viewer: viewer.value,
      serverName: "/navigate_to_pose",
      topic: "/map",
    });

    // Initialize the Navigator
    navigatorInstance.value = new NAV2D.Navigator({
      ros: new ROSLIB.Ros({
        url: `ws://${connectedRobot.ip}:${connectedRobot.port}`,
      }),
      rootObject: viewer.value.scene,
      viewer: viewer.value,
      serverName: "/navigate_to_pose",
      withOrientation: true,
    });

    console.log("Nav setup complete");
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
    // Pass the connectedRobot to the mapView function
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

const stopLaunch = async () => {
  stopping.value = true; // Set variabel stopping menjadi true untuk menandai bahwa proses penghentian sedang berlangsung
  try {
    // Kirim permintaan untuk menghentikan proses peluncuran map yang sedang berlangsung
    const response = await axios.post("http://localhost:5258/maps/stop");
    alert(response.data);
  } catch (error) {
    console.error("Error stopping map launch:", error);
  } finally {
    stopping.value = false; // Set variabel stopping kembali ke false setelah proses penghentian selesai atau gagal
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

const activateNavigation = () => {
  if (navigatorInstance.value && navigatorInstance.value.navigateMode) {
    navigatorInstance.value.navigateMode();
  } else {
    console.error("NAV2D.Navigator belum diinisialisasi");
  }
};

// Lifecycle Hooks
onMounted(() => {
  initConnection();
  fetchMaps();
});
</script>

<style scoped>
/* Add any specific styles if needed */
</style>
