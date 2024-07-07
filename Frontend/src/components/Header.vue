<template>
  <header class="navbar navbar-expand-lg bg-light">
    <div id="navigation" class="collapse navbar-collapse">
      <ul class="navbar-nav">
        <div class="nav1">
          <li class="nav-item">
            <i class="navbar-brand" href="#">Company Name</i>
          </li>
        </div>
        <div class="form-check form-switch">
          <div id="history-list"></div>
        </div>

        <div class="collapse navbar-collapse" id="navbarSupportedContent">
          <button class="button">
            <i
              class="fa-solid fa-backward-step"
              @click="continueMission"
              data-bs-toggle="tooltip"
              title="Continue Mission"
              data-bs-placement="right"
              style="font-size: 20px"
            ></i>
          </button>
          <li class="nav-item">
            <button class="button" @click="toggleMission">
              <span
                class="fa-solid"
                data-toggle="tooltip"
                data-bs-placement="bottom"
                :title="isPlaying ? 'Pause' : 'Start'"
              >
                <i v-if="isPlaying" class="fa-solid fa-pause"></i>
                <i v-else class="fa-solid fa-play"></i>
              </span>
            </button>
          </li>
          <div class="d-flex align-items-center">
            <select
              class="form-control form-control-sm"
              v-model="selectedMission"
            >
              <option
                v-for="mission in missions"
                :key="mission.id"
                :value="mission"
              >
                {{ mission.name }}
              </option>
            </select>
            <button class="btn btn-sm ml-2" @click="stopMission">
              <i
                style="
                  position: absolute;
                  z-index: 99;
                  margin-left: -40px;
                  margin-top: -10px;
                  color: #ff000f;
                "
                v-if="isPlaying"
                class="fa-solid fa-circle-xmark"
              ></i>
            </button>
          </div>
          <div
            v-if="showAlert"
            class="alert alert-primary d-flex align-items-center"
            style="position: absolute; z-index: 99; margin-top: 200px"
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
            <div>The robot is carrying out a mission</div>
          </div>
          <div
            v-if="showTerminatedAlert"
            class="alert alert-danger d-flex align-items-center"
            style="position: absolute; z-index: 99; margin-top: 200px"
            role="alert"
          >
            <svg
              class="bi flex-shrink-0 me-2"
              width="24"
              height="24"
              role="img"
              aria-label="Danger:"
            >
              <use xlink:href="#exclamation-triangle-fill" />
            </svg>
            <div>Mission terminated</div>
          </div>

          <div class="dropdown">
            <a
              class="nav-link"
              href="#"
              id="robotDropdown"
              role="button"
              data-toggle="dropdown"
              aria-haspopup="true"
              aria-expanded="false"
            >
              <span
                class="fa-solid fa-robot"
                data-toggle="tooltip"
                data-bs-placement="right"
                title="Select Robot"
              ></span>
            </a>
            <div
              class="dropdown-menu robot-dropdown-menu"
              aria-labelledby="robotDropdown"
              style="width: 250px; margin-left: -90px; border-radius: 20px"
            >
              <div
                class="robot-item"
                v-for="robot in robots"
                :key="robot.id"
                @mouseover="hoverRobot(robot)"
                @mouseleave="leaveRobot"
                style="margin-left: 20px"
              >
                <span v-if="!isHovered(robot.id)">{{ robot.name }}</span>
                <button
                  style="margin-left: -10px"
                  v-else
                  :class="{
                    'btn btn-success': isConnected(robot.id),
                    'btn btn-danger': !isConnected(robot.id),
                  }"
                  @click.stop="toggleConnection(robot)"
                >
                  {{ isConnected(robot.id) ? "Disconnect" : "Connect" }}
                </button>
                <button
                  style="margin-right: 20px; background: none"
                  class="btn select-btn"
                  @click.stop="selectRobot(robot)"
                >
                  <span
                    data-toggle="tooltip"
                    data-bs-placement="right"
                    title="Select"
                    class="material-symbols-outlined"
                    style="color: blue"
                  >
                    done_outline
                  </span>
                </button>
              </div>
            </div>
          </div>

          <div class="dropdown">
            <a
              class="nav-link"
              href="#"
              id="joystickDropdown"
              role="button"
              :aria-expanded="dropdownOpen ? 'true' : 'false'"
              @click="toggleDropdown"
            >
              <span
                class="fa-solid fa-gamepad"
                data-toggle="tooltip"
                data-bs-placement="right"
                title="Joystick"
              ></span>
            </a>
            <div
              class="dropdown-menu"
              :class="{ show: dropdownOpen }"
              style="margin-left: -80px; margin-top: 20px"
            >
              <div class="form-check form-switch">
                <input
                  class="form-check-input"
                  type="checkbox"
                  id="flexSwitchCheckDefault"
                  v-model="joystickVisible"
                  style="margin-left: 50px; margin-top: -20px"
                />
                <label class="form-check-label" for="flexSwitchCheckDefault"
                  >Toggle Joystick</label
                >
              </div>
            </div>
          </div>
        </div>
      </ul>
    </div>
  </header>
</template>

<script setup>
import { ref, computed } from "vue";
import { useStore } from "vuex";
import axios from "axios";
import Swal from "sweetalert2";

const store = useStore();
const selectedMission = ref(null);
const missions = ref([]);
const isPlaying = ref(false);
const showAlert = ref(false);
const showTerminatedAlert = ref(false);
const dropdownOpen = ref(false);
const joystickVisible = ref(false);
const robots = computed(() => store.state.robots);
const connectedRobots = computed(() => store.state.connectedRobots);
const hoveredRobotId = ref(null);
const selectedRobot = computed(() => store.state.selectedRobot);
const ws = ref(null);
const connected = ref(false);

const toggleMission = () => {
  if (!selectedMission.value) {
    alert("Please select a mission");
    return;
  }

  if (isPlaying.value) {
    pauseMission();
  } else {
    startMission();
  }
};

const initConnection = () => {
  const connectedRobot = robots.value.find(
    (robot) => connectedRobots.value[robot.id]
  );
  if (connectedRobot) {
    ws.value = new WebSocket(
      `ws://${connectedRobot.ip}:${connectedRobot.port}`
    );
    ws.value.onopen = () => {
      console.log("WebSocket connection established");
      connected.value = true;
    };
    ws.value.onerror = (error) => {
      console.error("WebSocket error: ", error);
      setTimeout(initConnection, 5000);
    };
  }
};

const startMission = () => {
  const mission = {
    type: "start_mission",
    waypoints: selectedMission.value.waypoints,
  };

  if (ws.value && ws.value.readyState === WebSocket.OPEN && connected.value) {
    ws.value.send(JSON.stringify(mission));
    console.log("Mission started:", mission);
    isPlaying.value = true;
    showAlert.value = true;
    setTimeout(() => {
      showAlert.value = false;
    }, 5000);
  } else {
    console.error("WebSocket not connected");
  }
};

const stopMission = () => {
  if (ws.value && ws.value.readyState === WebSocket.OPEN && connected.value) {
    ws.value.send(JSON.stringify({ type: "stop_mission" }));
    console.log("Mission stopped");
    isPlaying.value = false;
    showTerminatedAlert.value = true;
    setTimeout(() => {
      showTerminatedAlert.value = false;
    }, 4000);
  } else {
    console.error("WebSocket not connected");
  }
};

const pauseMission = () => {
  if (ws.value && ws.value.readyState === WebSocket.OPEN && connected.value) {
    ws.value.send(JSON.stringify({ type: "pause_mission" }));
    console.log("Mission paused");
    isPlaying.value = false;
  } else {
    console.error("WebSocket not connected");
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

const fetchRobots = async () => {
  await store.dispatch("fetchRobots");
};

const hoverRobot = (robot) => {
  hoveredRobotId.value = robot.id;
};

const leaveRobot = () => {
  hoveredRobotId.value = null;
};

const isHovered = (robotId) => {
  return hoveredRobotId.value === robotId;
};

const selectRobot = (robot) => {
  store.commit("setSelectedRobot", robot.name);
};

const toggleConnection = async (robot) => {
  try {
    if (isConnected(robot.id)) {
      await store.dispatch("disconnectRobot", robot);
      Swal.fire({
        icon: "success",
        title: "Disconnected!",
        text: `Robot ${robot.name} has been disconnected.`,
      });
    } else {
      await store.dispatch("connectRobot", robot);
      Swal.fire({
        icon: "success",
        title: "Connected!",
        text: `Robot ${robot.name} has been connected.`,
      });
    }
  } catch (error) {
    Swal.fire({
      icon: "error",
      title: "Error!",
      text: error.message,
    });
  }
};

const isConnected = (robotId) => {
  return connectedRobots.value[robotId] || false;
};

const toggleDropdown = () => {
  dropdownOpen.value = !dropdownOpen.value;
};

const continueMission = () => {
  console.log("Continue Mission button clicked");
  if (ws.value && connected.value) {
    ws.value.send(
      JSON.stringify({
        type: "continue_mission",
      })
    );
    console.log("Sent continue_mission command to server");
  } else {
    console.error("WebSocket not connected");
  }
};

onMounted(() => {
  fetchRobots();
  fetchMissions();
  initConnection();
});
</script>

<style scoped>
.navbar .nav2 .nav-item {
  margin-left: -50px;
  margin-right: 25px;
}

.navbar .nav3 .nav-item {
  margin-right: 20px;
}

.fa-solid .nav1 .nav-item {
  font-size: 30px;
  margin-right: 20px;
}

header {
  margin-top: -10px;
  position: fixed;
  top: 0;
  left: 15;
  width: 100%;
  z-index: 1000;
  padding: 10px;
  border-bottom: 1px solid #000000;
}

.button {
  background-color: #f8f8f8;
  margin-top: 1px;
  width: 20px;
  height: 50px;
}

.navbar-brand {
  margin-left: 50px;
  margin-top: 15px;
  font-family: "Poppins", sans-serif;
  font-weight: 700;
  color: #000000;
}

.robot-dropdown-menu {
  max-height: 200px;
  overflow-y: auto;
  overflow-x: hidden;
}

.robot-item {
  display: flex;
  align-items: center;
  justify-content: space-between;
  margin-bottom: 10px;
}

.robot-item button {
  margin-left: 10px;
}

.select-btn {
  z-index: 1;
}
</style>
