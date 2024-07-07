<template>
  <div class="container">
    <br />
    <div v-if="TampilTabel">
      <div class="d-flex">
        <h5 style="color: #000; font-size: 25px">
          Mission
          <span style="color: #0800ff">Management</span>
        </h5>
        <button class="btn btn-primary" @click="showTampilForm">
          Create Mission
        </button>
      </div>
      <br />
      <div
        class="card bg-light"
        style="box-shadow: 5px 10px 8px #779bff; border-radius: 10px"
      >
        <div
          class="card-header"
          style="
            margin: 10px;
            border-radius: 10px;
            font-size: 15px;
            font-weight: 700;
          "
        >
          <h2>Missions List</h2>
          <p style="margin-bottom: -8px; margin-top: -10px; font-weight: 300">
            Manage Missoins Here
          </p>
        </div>
        <div class="card-body">
          <div class="d-flex align-items-center">
            <p class="mr-2">Search</p>
            <div class="group">
              <svg class="icon" aria-hidden="true" viewBox="0 0 24 24">
                <g>
                  <path
                    d="M21.53 20.47l-3.66-3.66C19.195 15.24 20 13.214 20 11c0-4.97-4.03-9-9-9s-9 4.03-9 9 4.03 9 9 9c2.215 0 4.24-.804 5.808-2.13l3.66 3.66c.147.146.34.22.53.22s.385-.073.53-.22c.295-.293.295-.767.002-1.06zM3.5 11c0-4.135 3.365-7.5 7.5-7.5s7.5 3.365 7.5 7.5-3.365 7.5-7.5 7.5-7.5-3.365-7.5-7.5z"
                  ></path>
                </g>
              </svg>
              <input
                placeholder="Search"
                type="search"
                class="input"
                v-model="searchTerm"
                @input="fetchMissions"
              />
            </div>
          </div>
          <table class="table table-hover">
            <thead class="thead-dark">
              <tr>
                <th scope="col">ID</th>
                <th scope="col">Name</th>
                <th scope="col" class="text-end">Date Create</th>
                <th scope="col" class="text-end">Action</th>
              </tr>
            </thead>
            <tbody>
              <tr v-if="paginatedMission.length === 0">
                <td colspan="5" class="text-center">No content found</td>
              </tr>
              <tr
                v-for="(mission, index) in paginatedMission"
                :key="mission.id"
              >
                <td>{{ index + 1 + (currentPage - 1) * pageSize }}</td>
                <td>{{ mission.name }}</td>

                <td class="text-end">{{ mission.createdAt }}</td>
                <td class="text-end">
                  <button
                    @click="deleteMission(mission.id)"
                    class="btn btn-danger"
                  >
                    Delete
                  </button>
                </td>
              </tr>
            </tbody>
          </table>
          <nav aria-label="Page navigation example">
            <ul class="pagination">
              <li class="page-item" :class="{ disabled: currentPage === 1 }">
                <button class="page-link" @click="prevPage">&laquo;</button>
              </li>
              <li
                class="page-item"
                v-for="page in totalPages"
                :key="page"
                :class="{ active: currentPage === page }"
              >
                <button class="page-link" @click="changePage(page)">
                  {{ page }}
                </button>
              </li>
              <li
                class="page-item"
                :class="{ disabled: currentPage === totalPages }"
              >
                <button class="page-link" @click="nextPage">&raquo;</button>
              </li>
            </ul>
          </nav>
        </div>
      </div>
    </div>
    <div v-if="TampilForm || !TampilTabel">
      <div class="d-flex">
        <h5 style="color: #000; font-size: 25px">
          Create
          <span style="color: #0800ff">Mission</span>
        </h5>
        <button
          class="btn"
          @click="cancelForm"
          style="position: absolute; margin-left: 190px; margin-top: -6px"
        >
          <span
            class="material-symbols-outlined"
            data-bs-toggle="tooltip"
            title="Back"
          >
            undo
          </span>
        </button>
        <button type="submit" class="btn btn-success" @click="createMission">
          Create Mission
        </button>
      </div>

      <br />
      <div class="card bg-light">
        <transition name="expand">
          <div class="card-body" v-show="isFormCardExpanded">
            <form @submit.prevent="createMission">
              <div class="mb-3">
                <label for="missionName" class="form-label">Mission Name</label>
                <input
                  type="text"
                  class="form-control"
                  id="missionName"
                  v-model="mission.name"
                  required
                />
              </div>
              <!-- Form Dinamis untuk Titik -->
              <div
                v-for="(point, index) in mission.waypoints"
                :key="index"
                class="mb-3"
              >
                <label :for="'position-' + index" class="form-label"
                  >Position {{ index + 1 }}</label
                >
                <div class="input-group">
                  <input
                    type="text"
                    class="form-control"
                    :id="'position-' + index"
                    v-model="point.display"
                    readonly
                  />
                  <button
                    class="btn btn-outline-secondary"
                    type="button"
                    @click="startSelectingPosition(index)"
                  >
                    Select Position
                  </button>
                  <button
                    class="btn btn-outline-danger"
                    type="button"
                    @click="removeWaypoint(index)"
                  >
                    Remove
                  </button>
                </div>
              </div>
              <button
                class="btn btn-outline-primary"
                type="button"
                @click="addWaypoint"
              >
                + Add Waypoint
              </button>
              <div class="mb-3">
                <label for="mapSelect" class="form-label">Select Map</label>
                <select
                  class="form-select"
                  id="mapSelect"
                  v-model="selectedMap"
                >
                  <option v-for="map in maps" :key="map.id" :value="map.name">
                    {{ map.name }}
                  </option>
                </select>
                <button
                  class="btn btn-primary mt-2"
                  type="button"
                  @click="launchMap"
                >
                  Launch Map
                </button>
              </div>
            </form>
          </div>
        </transition>
        <button class="btn" @click="toggleFormCard">
          <span v-if="isFormCardExpanded" class="material-symbols-outlined">
            collapse_all
          </span>
          <span v-else class="material-symbols-outlined"> swap_vert </span>
        </button>
      </div>
    </div>
    <div class="card bg-light mt-3" v-show="TampilForm || !TampilTabel">
      <div id="nav" ref="navContainer"></div>
    </div>
  </div>
</template>

<script setup>
import { ref, watch, onMounted, computed } from "vue";
import axios from "axios";
import { useStore } from "vuex";
import Swal from "sweetalert2";
import { nextTick } from "vue";
import ROSLIB from "roslib";
const navContainer = ref(null);
const nav = ref(null);
const isFormCardExpanded = ref(true);
const toggleFormCard = () =>
  (isFormCardExpanded.value = !isFormCardExpanded.value);

const store = useStore();
const robots = computed(() => store.state.robots);
const connectedRobots = computed(() => store.state.connectedRobots);
const TampilTabel = ref(true);
const TampilForm = ref(false);
const ws = ref(null);
const viewer = ref(null);
const searchTerm = ref("");
const currentPage = ref(1);
const pageSize = ref(5);
const missions = ref([]);
const maps = ref([]);
const selectedMap = ref("");
const loading = ref(false);
const connected = ref(false);
const selectIndex = ref(null);
const cardWidth = ref(700);
const totalPages = computed(() =>
  Math.ceil(missions.value.length / pageSize.value)
);
const startIndex = computed(() => (currentPage.value - 1) * pageSize.value);
const endIndex = computed(() =>
  Math.min(startIndex.value + pageSize.value - 1, missions.value.length - 1)
);
const paginatedMission = computed(() =>
  missions.value.slice(startIndex.value, endIndex.value + 1)
);

const mission = ref({
  name: "",
  waypoints: [
    { display: "Start", x: 0, y: 0, orientation: 0 },
    { display: "Goal", x: 0, y: 0, orientation: 0 },
  ],
});

const showTampilForm = () => {
  localStorage.setItem("TampilTabel", JSON.stringify(false));
  localStorage.setItem("TampilForm", JSON.stringify(true));
  TampilTabel.value = false;
  TampilForm.value = true;
  nextTick(() => {
    initConnection();
  });
};

const deleteMission = async (id) => {
  try {
    const result = await Swal.fire({
      title: "Are you sure?",
      text: "You won't be able to revert this!",
      icon: "warning",
      showCancelButton: true,
      confirmButtonColor: "#3085d6",
      cancelButtonColor: "#d33",
      confirmButtonText: "Yes, delete it!",
    });

    if (result.isConfirmed) {
      await axios.delete(`http://localhost:5258/missions/${id}`);
      await fetchMissions(); // Perbarui data misi setelah penghapusan
      Swal.fire({
        title: "Deleted!",
        text: "Your mission has been deleted.",
        icon: "success",
        confirmButtonText: "OK",
      });
    }
  } catch (error) {
    Swal.fire("Error", "Failed to delete mission", "error");
    console.error("Failed to delete mission:", error);
  }
};

const updateCardWidth = () => {
  if (nav.value) {
    cardWidth.value = nav.value.width; // Assuming nav.value.width represents the width of the map
  } else {
    cardWidth.value = 700; // Fallback width when nav is not available or width is unknown
  }
};

const fetchMissions = async () => {
  try {
    const response = await axios.get("http://localhost:5258/missions/all");
    missions.value = response.data.filter((mission) =>
      mission.name.toLowerCase().includes(searchTerm.value.toLowerCase())
    );
  } catch (error) {
    console.error("Error fetching missions:", error);
  }
};

const fetchMaps = async () => {
  try {
    const response = await axios.get("http://localhost:5258/maps");
    maps.value = response.data;
  } catch (error) {
    console.error("Error fetching maps:", error);
  }
};

const cancelForm = () => {
  Swal.fire({
    title: "Are you sure?",
    text: "You won't be able to revert this!",
    icon: "warning",
    showCancelButton: true,
    confirmButtonColor: "#3085d6",
    cancelButtonColor: "#d33",
    confirmButtonText: "Yes, go back!",
  }).then((result) => {
    if (result.isConfirmed) {
      localStorage.setItem("TampilTabel", JSON.stringify(true));
      localStorage.setItem("TampilForm", JSON.stringify(false));
      TampilTabel.value = true;
      TampilForm.value = false;
      resetForm();
      stopLaunch();
    }
  });
};

const stopping = ref(false);
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
  } catch (error) {
    console.error("Error launching map:", error);
  } finally {
    loading.value = false;
  }
};

const startSelectingPosition = (index) => {
  selectIndex.value = index;
};

const selectPosition = (event) => {
  if (!viewer.value || !viewer.value.scene) {
    console.error("Viewer is not initialized.");
    return;
  }

  const coords = viewer.value.scene.globalToRos(event.stageX, event.stageY);
  const x = coords.x;
  const y = coords.y;
  const orientation = 0;

  mission.value.waypoints[selectIndex.value] = {
    display: `X: ${x}, Y: ${y}, Orientation: ${orientation}`,
    x,
    y,
    orientation,
  };
};

const addWaypoint = () => {
  mission.value.waypoints.push({
    display: "",
    x: 0,
    y: 0,
    orientation: 0,
  });
};

const removeWaypoint = (index) => {
  // Check if the waypoint is not the first or last (start or goal)
  if (index > 1) {
    mission.value.waypoints.splice(index, 1);
  } else {
    alert("Cannot remove start or goal waypoint.");
  }
};

const createMission = async () => {
  try {
    const missionData = {
      name: mission.value.name,
      waypoints: mission.value.waypoints.map((point) => ({
        x: point.x,
        y: point.y,
        orientation: point.orientation,
      })),
    };
    console.log("Creating mission with data:", missionData);
    const response = await axios.post(
      "http://localhost:5258/missions",
      missionData
    );
    if (response.status === 201) {
      Swal.fire({
        title: "Success",
        text: "Mission created successfully!",
        icon: "success",
        confirmButtonText: "OK",
      }).then(async (result) => {
        if (result.isConfirmed) {
          stopLaunch();
          TampilTabel.value = true;
          TampilForm.value = false;
          resetForm();
          await fetchMissions(); // Perbarui data setelah form disubmit
        }
      });
    } else {
      throw new Error("Failed to create mission");
    }
  } catch (error) {
    console.error("Error creating mission:", error);
    Swal.fire("Error", "Failed to create mission", "error");
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
    nextTick(() => {
      const navElement = navContainer.value;
      if (!navElement) {
        console.error("Element with ID 'nav' not found in DOM.");
        return;
      }

      viewer.value = new ROS2D.Viewer({
        divID: "nav",
        width: navElement.clientWidth,
        height: 550,
      });

      nav.value = new NAV2D.OccupancyGridClientNav({
        ros: new ROSLIB.Ros({
          url: `ws://${connectedRobot.ip}:${connectedRobot.port}`,
        }),
        rootObject: viewer.value.scene,
        viewer: viewer.value,
        serverName: "/navigate_to_pose",
        topic: "/map",
        withOrientation: false,
      });

      viewer.value.scene.addEventListener("click", selectPosition);

      console.log("Nav setup complete");
    });
  }
};
const resetForm = () => {
  mission.value = {
    name: "",
    waypoints: [
      { display: "Start", x: 0, y: 0, orientation: 0 },
      { display: "Goal", x: 0, y: 0, orientation: 0 },
    ],
  };
};
onMounted(() => {
  const TampilTabell = localStorage.getItem("TampilTabel");
  const TampilFormm = localStorage.getItem("TampilForm");
  TampilTabel.value = TampilTabell ? JSON.parse(TampilTabell) : true;
  TampilForm.value = TampilFormm ? JSON.parse(TampilFormm) : false;
  fetchMissions();
  fetchMaps();
  initConnection();
  // Inisialisasi tooltip Bootstrap
  const tooltipTriggerList = [].slice.call(
    document.querySelectorAll('[data-bs-toggle="tooltip"]')
  );
  tooltipTriggerList.map(function (tooltipTriggerEl) {
    return new bootstrap.Tooltip(tooltipTriggerEl);
  });
});

// Watchers to keep localStorage in sync with state
watch(TampilTabel, (newValue) => {
  localStorage.setItem("TampilTabel", JSON.stringify(newValue));
});

watch(TampilForm, (newValue) => {
  localStorage.setItem("TampilForm", JSON.stringify(newValue));
});
</script>

<style scoped>
.group {
  display: flex;
  line-height: 28px;
  align-items: center;
  position: relative;
  max-width: 190px;
}
.mr-2 {
  margin-top: 1px;
}
.group .input {
  margin-top: 1px;
  width: 100%;
  height: 30px;
  line-height: 28px;
  padding: 0 1rem;
  padding-left: 2.5rem;
  border: 2px solid transparent;
  border-radius: 8px;
  outline: none;
  background-color: #ffffff;
  color: #0d0c22;
  transition: 0.3s ease;
  box-shadow: 1px 1px 2px #000;
  margin-bottom: 10px;
}

.group .input::placeholder {
  color: #9e9ea7;
}
.input:focus,
input:hover {
  outline: none;
  border-color: rgba(0, 10, 196, 0.4);
  background-color: #fff;
  box-shadow: 0 0 0 4px rgba(49, 13, 228, 0.1);
}

.group .icon {
  position: absolute;
  top: 0.6em;
  left: 0.6rem;
  fill: #9e9ea7;
  width: 1rem;
  height: 1rem;
}

/* Transition for expanding/collapsing card */
.expand-enter-active,
.expand-leave-active {
  transition: cubic-bezier(0.075, 0.82, 0.165, 1);
}
.expand-enter, .expand-leave-to /* .expand-leave-active in <2.1.8 */ {
  max-height: 0;
  overflow: hidden;
}
</style>
