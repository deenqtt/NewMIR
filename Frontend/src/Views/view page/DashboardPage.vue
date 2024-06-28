<template>
  <div class="container">
    <br />
    <div v-if="selectedRobot !== ''">
      <div class="d-flex">
        <h5 class="selectrobot" style="margin-top: 10px">
          Dashboards
          <span style="color: #000">{{ selectedRobot }}</span>
        </h5>

        <transition name="slide-fade">
          <div
            v-if="showAlert"
            class="alert alert-success d-flex align-items-center"
            role="alert"
          >
            <svg
              class="bi flex-shrink-0 me-2"
              width="24"
              height="24"
              role="img"
              aria-label="Success:"
            >
              <use xlink:href="#check-circle-fill" />
            </svg>
            <div>Success in achieving goals</div>
          </div>
        </transition>

        <transition name="slide-fade">
          <div
            v-if="showProgressAlert"
            class="alert alert-primary d-flex align-items-center"
            role="alert"
          >
            <svg
              class="bi flex-shrink-0 me-2"
              width="24"
              height="24"
              role="img"
              aria-label="Info:"
            >
              <use xlink:href="#info-fill" />
            </svg>
            <div>Robot is Carrying Out Tasks</div>
          </div>
        </transition>
      </div>
      <br />
      <div class="row">
        <div class="col-md-6">
          <label for="mapSelect">Select Map</label>
          <div class="d-flex justify-content-between align-items-center mb-3">
            <div class="form-group flex-grow-1 mr-2">
              <select class="form-select" id="mapSelect" v-model="selectedMap">
                <option v-for="map in maps" :key="map.id" :value="map.name">
                  {{ map.name }}
                </option>
              </select>
            </div>
            <button
              class="btn btn-secondary"
              type="button"
              @click="launchMap"
              style="margin-top: -17px"
            >
              Launch
            </button>
          </div>
        </div>
        <div class="col-md-6">
          <label for="pathSelect">Select Path</label>
          <div class="d-flex justify-content-between align-items-center mb-3">
            <div class="form-group flex-grow-1 mr-2">
              <select
                class="form-select"
                id="pathSelect"
                v-model="selectedPath"
              >
                <option v-for="path in paths" :key="path.id" :value="path">
                  {{ path.name }}
                </option>
              </select>
            </div>
            <button
              class="btn btn-primary"
              type="button"
              @click="startPath"
              style="margin-top: -17px"
            >
              Start Path
            </button>
          </div>
        </div>
      </div>
      <br />
    </div>

    <div v-else>
      <!-- Show default content when no robot is selected -->
      <h5>Dashboards</h5>
      <div class="default-content">
        <div class="robot-search-container">
          <img
            src="../image/select.png (1).png"
            alt="Searching for robots..."
          />
          <p>Please select a robot</p>
        </div>
      </div>
    </div>
    <div
      v-show="selectedRobot !== ''"
      class="card bg-light"
      style="box-shadow: 1px 1px 2px #000"
    >
      <div
        class="card-header d-flex justify-content-between align-items-center"
      >
        <div class="left-icons d-flex align-items-center">
          <span
            class="material-symbols-outlined"
            @click="initializeRobot"
            data-bs-toggle="tooltip"
            title="Initialize Robot"
            >location_on</span
          >
          <span class="separator"></span>
          <span
            class="material-symbols-outlined"
            @click="zoomIn"
            data-bs-toggle="tooltip"
            title="Zoom In"
            >zoom_in</span
          >
          <span
            class="material-symbols-outlined"
            @click="zoomOut"
            data-bs-toggle="tooltip"
            title="Zoom Out"
            >zoom_out</span
          >

          <span class="separator"></span>
          <span
            class="material-symbols-outlined"
            style="color: #ff0000"
            @click="stopMission"
          >
            cancel
          </span>
          <span style="font-size: 12px; margin-left: -6px; color: #000"
            >Stop Map</span
          >
        </div>

        <div class="right-icons d-flex align-items-center">
          <span
            class="material-symbols-outlined"
            @click="moveLeft"
            data-bs-toggle="tooltip"
            title="Move Left"
            >arrow_back_ios</span
          >
          <span class="separator"></span>
          <span
            class="material-symbols-outlined"
            @click="moveRight"
            data-bs-toggle="tooltip"
            title="Move Right"
            >arrow_forward_ios</span
          >
        </div>
      </div>

      <div id="nav" ref="navContainer"></div>
    </div>
  </div>
</template>
<script setup>
import axios from "axios";
import { useStore } from "vuex";
import { ref, onMounted, computed, watch, nextTick } from "vue";
const store = useStore();
const robots = computed(() => store.state.robots);
const connectedRobots = computed(() => store.state.connectedRobots);
const connected = ref(false);
const selectedRobot = ref("");
const maps = ref([]);
const showAlert = ref(false);
const showProgressAlert = ref(false);
const paths = ref([]);
const navigatorInstance = ref(null);
const navContainer = ref(null);
const navClient = ref(null);
const selectedMap = ref(null);
const selectedPath = ref(null);
const viewer = ref(null);
const initializePosition = ref(null);
const currentPose = ref(null);
const poseListener = ref(null);
const nav = ref(null);
const rosSocket = ref(null); // WebSocket for ROS
const ws = ref(null); // WebSocket for goal
const stopping = ref(false);
const showContinueButton = ref(false); // Added showContinueButton ref
const fetchMaps = async () => {
  try {
    const response = await axios.get("http://localhost:5258/maps");
    maps.value = response.data;
  } catch (error) {
    console.error("Error fetching maps:", error);
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

const launchMap = async () => {
  if (!selectedMap.value) {
    return;
  }
  try {
    const formData = new FormData();
    formData.append("mapName", selectedMap.value);
    const response = await axios.post(
      "http://localhost:5258/maps/launch",
      formData
    );
  } catch (error) {
    console.error("Error launching map:", error);
  }
};

const startPath = async () => {
  if (!selectedPath.value) {
    alert("Please select a path");
    return;
  }

  showProgressAlert.value = true; // Show the progress alert when the task starts

  const goal = {
    type: "send_goal",
    x: parseFloat(selectedPath.value.posX),
    y: parseFloat(selectedPath.value.posY),
    z: 0.0,
    w: parseFloat(selectedPath.value.orientation),
  };
  console.log("Sending goal to server:", goal);
  ws.value.send(JSON.stringify(goal));
  ws.value.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    if (msg.type === "position_reached") {
      if (msg.position === "goal") {
        showProgressAlert.value = false; // Hide the progress alert
        showAlert.value = true; // Show the success alert
        setTimeout(() => {
          showAlert.value = false; // Hide the success alert after 5 seconds
        }, 5000);
      }
    } else if (msg.type === "goal_error") {
      console.error(`Error: ${msg.error}`);
    }
  };
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
    connected.value = true; // Add this line
  };
  ws.value.onerror = (error) => {
    console.error("Goal WebSocket error: ", error); // Add error handling for goal WebSocket
    connected.value = false;
    setTimeout(initConnection, 5000); // Retry connection after 5 seconds
  };
  ws.value.onclose = () => {
    console.log("Goal WebSocket connection closed");
    connected.value = false;
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
    nextTick(() => {
      const navElement = document.getElementById("nav");
      if (!navElement) {
        console.error("Element with ID 'nav' not found in DOM.");
        return;
      }
      viewer.value = new ROS2D.Viewer({
        divID: "nav",
        width: navContainer.value.clientWidth,
        height: 550,
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
    });
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

const zoomIn = () => {
  if (viewer.value) {
    viewer.value.scene.scaleX *= 1.2;
    viewer.value.scene.scaleY *= 1.2;
    viewer.value.scene.update();
    console.log("Zoomed in");
  } else {
    console.error("Viewer is not initialized");
  }
};

const zoomOut = () => {
  if (viewer.value) {
    viewer.value.scene.scaleX /= 1.2;
    viewer.value.scene.scaleY /= 1.2;
    viewer.value.scene.update();
    console.log("Zoomed out");
  } else {
    console.error("Viewer is not initialized");
  }
};

const moveLeft = () => {
  if (viewer.value) {
    viewer.value.scene.x += 20;
    viewer.value.scene.update();
    console.log("Moved left");
  } else {
    console.error("Viewer is not initialized");
  }
};

const moveRight = () => {
  if (viewer.value) {
    viewer.value.scene.x -= 20;
    viewer.value.scene.update();
    console.log("Moved right");
  } else {
    console.error("Viewer is not initialized");
  }
};

const stopMission = async () => {
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
onMounted(() => {
  // Ensure mapView is called if a robot is connected
  const connectedRobot = robots.value.find(
    (robot) => connectedRobots.value[robot.id]
  );
  if (connectedRobot) {
    mapView(connectedRobot);
  }
  // Call updateCardWidth initially and whenever necessary
  // updateCardWidth();
  fetchMaps();
  initConnection();
  fetchPaths();
  selectedRobot.value = store.state.selectedRobot;
  store.subscribe((mutation) => {
    if (mutation.type === "setSelectedRobot") {
      selectedRobot.value = store.state.selectedRobot;
      nextTick(() => {
        initConnection();
      });
    }
  });
  // Inisialisasi tooltip Bootstrap
  const tooltipTriggerList = [].slice.call(
    document.querySelectorAll('[data-bs-toggle="tooltip"]')
  );
  tooltipTriggerList.map(function (tooltipTriggerEl) {
    return new bootstrap.Tooltip(tooltipTriggerEl);
  });
});
// Assuming nav.value is reactive and updates when the map size changes
watch(nav, () => {
  updateCardWidth();
});
</script>
<style scoped>
.robot-search-container {
  text-align: center;
}
.default-content {
  display: flex;
  justify-content: center;
  align-items: center;
}
.robot-search-container img {
  width: 300px;
  margin-bottom: 20px;
  background: none;
  -webkit-filter: drop-shadow(5px 5px 5px #666666);
  filter: drop-shadow(5px 5px 5px #666666);
}
.container {
  font-family: "Poppins", sans-serif;
}

h5 {
  font-size: 25px;
  font-weight: 700;
  color: #0800ff;
}

.card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.left-icons,
.right-icons {
  display: flex;
  align-items: center;
}

.card-header span {
  cursor: pointer;
  margin: 0 10px;
}

.separator {
  width: 1px;
  height: 24px;
  background-color: #ccc;
  margin: 0 10px;
}

.robot-search-container {
  text-align: center;
}

.default-content {
  display: flex;
  justify-content: center;
  align-items: center;
}

.robot-search-container img {
  width: 300px;
  margin-bottom: 20px;
  background: none;
  -webkit-filter: drop-shadow(5px 5px 5px #666666);
  filter: drop-shadow(5px 5px 5px #666666);
}

.card-header span {
  cursor: pointer;
  margin: 0 10px;
}

.selectrobot {
  margin-left: 20px;
  margin-top: -20px;
  font-weight: 700;
}
.slide-fade-enter-active,
.slide-fade-leave-active {
  transition: all 0.3s ease;
}

.slide-fade-enter,
.slide-fade-leave-to {
  transform: translateX(100%);
  opacity: 0;
}
</style>
