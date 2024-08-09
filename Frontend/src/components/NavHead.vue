<template>
  <div>
    <header class="navbar navbar-expand-lg bg-light">
      <div id="navigation" class="collapse navbar-collapse">
        <ul class="navbar-nav">
          <div class="nav1">
            <li class="nav-item">
              <i class="navbar-brand" href="#">Company Name</i>
            </li>
          </div>
          <div class="form-check form-switch">
            <!-- <input
              class="form-check-input"
              type="checkbox"
              id="flexSwitchCheckDefault"
              v-model="mqttConnected"
              disabled
            />
            <p>MQTT ({{ mqttStatus }})</p> -->
            <div id="history-list"></div>
          </div>

          <div class="collapse navbar-collapse" id="navbarSupportedContent">
            <button class="button" style="">
              <i
                class="fa-solid fa-backward-step"
                @click="continueMission"
                data-bs-toggle="tooltip"
                title="Continue Mission"
                data-bs-placement="right"
                style="font-size: 20px"
              >
              </i>
            </button>
            <li class="nav-item">
              <!-- Continue Mission Button -->

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

          <div class="battery-status ms-3" :title="`Battery: ${batteryLevel}%`">
            <div class="battery-icon">
              <div
                class="battery-level"
                :style="{ width: batteryLevel + '%' }"
                :class="batteryLevelClass"
              ></div>
            </div>
            <!-- Tambahkan Tombol Refresh -->
            <button class="btn btn-sm ml-2" @click="refreshBatteryData">
              <i class="fa-solid fa-sync" title="Refresh Battery Data"></i>
            </button>
          </div>
        </ul>
      </div>
    </header>
    <Joystick
      v-if="joystickVisible"
      style="position: absolute; margin-top: 400px; margin-left: 200px"
    />
    <nav
      :style="{ width: is_expanded ? 'var(--sidebar-width)' : '30px' }"
      id="sidenav-5"
      data-te-sidenav-init
      data-te-sidenav-hidden="false"
      data-te-sidenav-accordion="true"
    >
      <div class="menu-toggle-wrap" style="background-color: #fff">
        <button class="button" @click="toggleMenu">
          <span
            :class="[
              'fa-solid',
              is_expanded ? 'fa-angles-right' : 'fa-angles-left',
            ]"
          ></span>
        </button>
      </div>

      <div class="sidebar-content">
        <h2>{{ pageTitle }}</h2>
        <h5 v-if="subMenu.length > 0"></h5>

        <router-link
          v-for="(item, index) in subMenu"
          :key="index"
          :to="`/${item.toLowerCase()}`"
          class="submenu-item"
          :class="{ 'active-submenu': isSubMenuActive(item) }"
        >
          <h5>{{ item }}</h5>
        </router-link>
      </div>
    </nav>
  </div>
</template>

<script setup>
import axios from "axios";
import {
  ref,
  watch,
  onMounted,
  toRefs,
  onUnmounted,
  computed,
  reactive,
  toRaw,
} from "vue";
import { useRouter } from "vue-router";
import { useStore } from "vuex";
import Joystick from "./Joystick.vue";
const batteryLevel = ref(50); // Initibatteryeve

watch(batteryLevel, (newLevel) => {
  console.log(`Battery level changed to: ${newLevel}%`);
});

const batteryLevelClass = computed(() => {
  return batteryLevel.value < 20 ? "low" : "";
});

import Swal from "sweetalert2";
const showAlert = ref(false);
const showTerminatedAlert = ref(false);
const router = useRouter();
const robots = computed(() => store.state.robots);
const connectedRobots = computed(() => store.state.connectedRobots);
const hoveredRobotId = ref(null);
const missionOptions = ref([]);
const activeSubMenu = ref("");
const message = ref(""); // Added message ref
const store = useStore();
const ws = ref(null); // WebSocket for goal
const selectedMission = ref(null);
const selectedRobot = computed(() => store.state.selectedRobot);
const hoveredNotif = ref(false);
const dropdownOpen = ref(false);
const joystickVisible = ref(false);
const missions = ref([]); // List of missions
const dropdownOpen1 = ref(false);
const connected = ref(false);
const rosSocket = ref(null); // WebSocket for RO
const isPlaying = ref(false);
const wsBattery = ref(null);
const connectBatteryWebSocket = () => {
  const connectWebSocket = () => {
    wsBattery.value = new WebSocket(`ws://localhost:3000`);

    wsBattery.value.onopen = () => {
      console.log("Battery WebSocket connection established");
    };

    wsBattery.value.onerror = (error) => {
      console.error("Battery WebSocket error: ", error);
    };

    wsBattery.value.onmessage = (event) => {
      const msg = JSON.parse(event.data);
      console.log("Received battery message:", msg);
      if (msg.type === "battery_update") {
        batteryLevel.value = msg.level;
        console.log(`Battery level updated: ${msg.level}%`);
      }
    };

    wsBattery.value.onclose = () => {
      console.log("Battery WebSocket connection closed");
      setTimeout(connectWebSocket, 5000); // Retry connection after 5 seconds
    };
  };

  connectWebSocket();
};

// Tambahkan metode ini
const refreshBatteryData = () => {
  if (ws.value) {
    ws.value.close();
  }
  connectBatteryWebSocket();
  console.log("Battery data refresh requested");
};

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
    };
    rosSocket.value.onerror = (error) => {
      console.error("ROS2 WebSocket error: ", error);
      setTimeout(initConnection, 5000);
    };
  }

  ws.value = new WebSocket(`ws://localhost:3000`);
  ws.value.onopen = () => {
    console.log("Goal WebSocket connection established");
    connected.value = true;
  };
  ws.value.onerror = (error) => {
    console.error("Goal WebSocket error: ", error);
    connected.value = false;
    setTimeout(initConnection, 5000); // Retry connection after 5 seconds
  };
  ws.value.onclose = () => {
    console.log("Goal WebSocket connection closed");
    connected.value = false;
  };
  ws.value.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    console.log("Received message:", msg); // Log received message for debugging
    if (msg.type === "position_reached") {
      message.value = `Reached ${msg.position}`;
      console.log(`Reached position: ${msg.position}`); // Log position reached
      if (msg.position === "start" || msg.position === "goal") {
        showContinueButton = true;
      }
    } else if (msg.type === "goal_error") {
      message.value = `Error: ${msg.error}`;
      console.error(`Goal error: ${msg.error}`); // Log goal error
    } else if (msg.type === "battery_update") {
      batteryLevel.value = msg.level;
      console.log(`Battery level updated: ${msg.level}%`); // Log battery update
    }
  };
};

const startMission = () => {
  if (!selectedMission.value) {
    alert("Please select a mission");
    return;
  }

  const mission = {
    type: "start_mission",
    waypoints: selectedMission.value.waypoints.map((wp) => ({
      x: wp.x,
      y: wp.y,
      orientation: wp.orientation,
    })),
  };

  if (ws.value && ws.value.readyState === WebSocket.OPEN && connected.value) {
    ws.value.send(JSON.stringify(mission));
    console.log("Mission started:", mission);
    isPlaying.value = true;

    // Tampilkan waypoint di console log
    selectedMission.value.waypoints.forEach((waypoint, index) => {
      console.log(
        `Waypoint ${index + 1}: X=${waypoint.x}, Y=${waypoint.y}, Orientation=${
          waypoint.orientation
        }`
      );
    });

    // Show alert for mission started
    showAlert.value = true;
    setTimeout(() => {
      showAlert.value = false;
    }, 5000); // Hide alert after 5 seconds
  } else {
    console.error("WebSocket not connected");
  }
};

const stopMission = () => {
  if (ws.value && ws.value.readyState === WebSocket.OPEN && connected.value) {
    ws.value.send(JSON.stringify({ type: "stop_mission" }));
    console.log("Mission stopped");
    isPlaying.value = false;

    // Show alert for mission terminated
    showTerminatedAlert.value = true;
    setTimeout(() => {
      showTerminatedAlert.value = false;
    }, 4000); // Hide alert after 4 seconds
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

const pauseMission = () => {
  if (ws.value && ws.value.readyState === WebSocket.OPEN && connected.value) {
    ws.value.send(JSON.stringify({ type: "pause_mission" }));
    console.log("Mission paused");
    isPlaying.value = false;
  } else {
    console.error("WebSocket not connected");
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

const removeNotifications = (notificationIndex) => {
  // Remove the clicked notification from the list
  store.dispatch("removeNotification", notificationIndex);
};

// Dapatkan nilai boolean apakah ada notifikasi yang belum dibaca
const hasUnreadNotifications = computed(() => {
  return store.state.unreadNotifications > 0;
});

const markNotificationsAsReadAndRemoveLocalStorage = () => {
  // Mark all notifications as read in Vuex store
  store.state.notifications.forEach((notification) => {
    notification.read = true;
  });

  // Remove the flag indicating unread notifications from local storage
  localStorage.removeItem("hasUnreadNotifications");
};
const addNotification = (notification) => {
  // Menambahkan notifikasi ke dalam array notifications
  store.commit("addNotification", notification);

  // Set a flag in local storage to indicate that there are unread notifications
  localStorage.setItem("hasUnreadNotifications", true);
};

const notifications = computed(() => {
  return store.state.notifications;
});

const toggleDropdown = () => {
  dropdownOpen.value = !dropdownOpen.value;
};

const toggleDropdownJosytick = () => {
  dropdownOpen1.value = !dropdownOpen1.value;
};

watch(selectedRobot, () => {
  console.log("Selected Robot changed, fetching filtered missions...");
  // You can fetch the filtered missions here or update as needed
});

const isSubMenuActive = (item) => {
  const routeName = router.currentRoute.value.name;
  const isActive = routeName && routeName.toLowerCase() === item.toLowerCase();
  if (isActive) {
    activeSubMenu.value = item;
  }
  console.log(`Route: ${routeName}, Active Submenu: ${activeSubMenu.value}`);
  return isActive;
};

// Deklarasi state
const state = reactive({
  is_expanded: localStorage.getItem("is_expanded") === "true",
  pageTitle: "",
  subMenu: [],
});

// Mendapatkan referensi ke variabel dari state
const { is_expanded, pageTitle, subMenu } = toRefs(state);
const toggleMenu = () => {
  is_expanded.value = !is_expanded.value;
  localStorage.setItem("is_expanded", is_expanded.value);
};

const updatePageTitle = () => {
  const routeName = router.currentRoute.value.name;
  const commonSubMenu = [
    "Add Robot",
    "Maps",
    "Path",
    "Mission",
    "Footprint",
    "Modul",
    "User",
  ];

  if (routeName === "Dashboard") {
    pageTitle.value = "Dashboard";
    subMenu.value = ["Dashboard"];
  } else if (routeName === "Setup" || commonSubMenu.includes(routeName)) {
    pageTitle.value = "Setup";
    subMenu.value = commonSubMenu;
  } else if (routeName === "Activity") {
    pageTitle.value = "Monitoring";
    subMenu.value = ["Activity", "Error Log"];
  } else if (routeName === "Settings") {
    pageTitle.value = "System";
    subMenu.value = ["Settings", "Error"];
  } else if (routeName === "Robot") {
    pageTitle.value = "Robot";
    subMenu.value = ["Robot"];
  }
};

watch(
  () => router.currentRoute.value.name,
  () => {
    updatePageTitle();
  }
);
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
  $(document).ready(function () {
    $('[data-toggle="tooltip"]').tooltip();
  });
  console.log("Component mounted");
  connectBatteryWebSocket();
  updatePageTitle();
  fetchRobots();
  fetchMissions();
  initConnection();
});
onUnmounted(() => {
  if (ws.value) {
    ws.value.close();
  }
});
</script>

<style lang="scss" scoped>
.dropdown-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.form-check-input:checked {
  background-color: #28a745; /* Warna hijau saat tombol dicentang (checked) */
  border-color: #28a745; /* Warna garis pinggir hijau */
}

.form-check-input:checked + .form-check-label {
  color: #28a745; /* Warna teks hijau saat tombol dicentang (checked) */
}

.form-check p {
  width: 200px;
}

.form-check {
  margin-left: 20px;
  margin-top: 20px;
  font-family: "Poppins", sans-serif;
  font-weight: 600;
  width: 200px;
}

.submenu-item .fa-submenu {
  margin-right: -10px;
}

.robot-dropdown-menu {
  max-height: 200px; /* Atur ketinggian maksimum dropdown di sini */
  overflow-y: auto; /* Aktifkan scrolling vertikal */
  overflow-x: hidden; /* Sembunyikan scrolling horizontal */
}

/* Styling for dropdown menu items */
/* Keyframes untuk animasi dropdown */
@keyframes dropdownAnimation {
  from {
    transform: translateY(-50%);
    opacity: 0;
  }
  to {
    transform: translateY(0);
    opacity: 1;
  }
}

/* CSS untuk menampilkan titik biru di atas ikon lonceng (bell) */
.notification-container {
  position: relative;
}

.notification-dot {
  position: absolute;
  width: 10px;
  height: 10px;
  background-color: rgb(0, 255, 51);
  border-radius: 50%;
  top: -5px;
  right: -5px;
}

#notif .dropdown-menu {
  margin-left: -70px;
}

.form-control {
  width: 180px;
  height: 20px;
}

button {
  border: hidden;
}

input {
  height: 20px;
  background-color: #d2d2d2;
}

#navbarSupportedContent {
  gap: 60px;
  margin-left: 100px;
  margin-top: 10px;
}

.submenu-item.active-submenu {
  h5 {
    position: relative;
    margin-top: 0;
    padding: 5px 20px;
    background-color: #f0f0f0;
    color: #000000;

    &:before,
    &:after {
      content: "";
      position: absolute;
      left: 0;
      right: 0;
      height: 1px;
      background-color: #000000;
    }

    &:before {
      top: -1px;
    }

    &:after {
      bottom: -1px;
    }
  }
}

.fa-solid {
  font-size: 18px;
  color: #000000;
}

.submenu-item.active-submenu:first-child h5:before,
.submenu-item.active-submenu:last-child h5:after {
  content: "";
  position: absolute;
  left: 0;
  right: 0;
  height: 1px;
  background-color: #000000;
}

.submenu-item.active-submenu:first-child h5:before {
  top: -1px;
}

.submenu-item.active-submenu:last-child h5:after {
  bottom: -1px;
}

.submenu-item {
  text-decoration: none;
  color: #000000;
  position: relative;

  h5 {
    position: relative;
    margin-top: 0;
    padding: 5px 20px;

    &:hover {
      &:before,
      &:after {
        content: "";
        position: absolute;
        left: 0;
        right: 0;
        height: 1px;
        background-color: #000000;
      }

      &:before {
        top: -1px;
      }

      &:after {
        bottom: -1px;
      }
    }
  }
}

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

nav {
  margin-top: 50px;
  display: flex;
  flex-direction: column;
  background-color: #ffffff;
  overflow: hidden;
  padding: 1rem;
  width: var(--sidebar-width);
  z-index: 899;

  .menu-toggle-wrap {
    margin-left: -50px;
    display: flex;
    justify-content: flex-end;
    margin-bottom: 1rem;
    position: relative;
    top: 0;
    transition: 0.8s ease-in-out;

    .menu-toggle {
      transition: 0.8s ease-in-out;
    }
  }
}

.menu-toggle-wrap {
  display: flex;
  justify-content: center;
}

.is-expanded nav {
  .menu-toggle-wrap {
    top: -3rem;

    .menu-toggle {
      transform: rotate(-180deg);
    }
  }
}

.button {
  background-color: #f8f8f8;
  margin-top: 1px;
  width: 20px;
  height: 50px;
}

h2 {
  font-family: "Poppins", sans-serif;
  font-size: 20px;
  margin-left: 30px;
  font-weight: 700;
  margin-top: -20px;
}

.navbar-brand {
  margin-left: 50px;
  margin-top: 15px;
  font-family: "Poppins", sans-serif;
  font-weight: 700;
  color: #000000;
}

h5 {
  font-size: 14px;
  font-family: "Poppins", sans-serif;
  font-weight: 300;
  margin-left: 20px;
  padding: 20px;
  margin-top: -30px;
  margin-bottom: 20px;
}

.submenu-item {
  text-decoration: none;
  color: #000000;
}

.menu-toggle-wrap button {
  background: #ffffff;
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

.battery-status {
  display: flex;
  margin-top: 9px;
  align-items: center;
  font-size: 18px;
  color: #333;
}

.battery-icon {
  width: 30px;
  height: 15px;
  border: 2px solid #333;
  border-radius: 3px;
  position: relative;
  background-color: #ddd;
}

.battery-icon::after {
  content: "";
  width: 3px;
  height: 7px;
  background-color: #b6b0b0;
  position: absolute;
  right: -5px;
  top: 4px;
  border-radius: 1px;
}

.battery-level {
  height: 100%;
  background-color: #20d426; /* Hijau */
  transition: width 0.3s ease-in-out;
}

.battery-level.low {
  background-color: #f44336; /* Merah */
}

.battery-status .ms-2 {
  margin-left: 5px;
}

/* Responsive CSS */
@media (max-width: 768px) {
  header {
    padding: 5px;
  }

  .navbar-nav {
    flex-direction: row;
    flex-wrap: wrap;
  }

  .navbar-brand {
    margin-left: 10px;
    font-size: 16px;
  }

  .nav-item {
    margin-right: 10px;
    margin-bottom: 10px;
  }

  .button {
    margin: 0 5px;
    width: 30px;
    height: 40px;
  }

  .fa-solid {
    font-size: 14px;
  }

  #navbarSupportedContent {
    gap: 20px;
    margin-left: 10px;
    flex-direction: column;
    align-items: flex-start;
  }

  .form-control {
    width: 150px;
    height: 30px;
  }

  .battery-status {
    font-size: 14px;
  }

  .battery-icon {
    width: 20px;
    height: 10px;
  }
}
</style>
