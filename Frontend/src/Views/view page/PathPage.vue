<template>
  <div class="container">
    <br />
    <div v-if="showTable">
      <div class="d-flex">
        <h5 style="color: #000; font-size: 25px">
          Path
          <span style="color: #0800ff">Management</span>
        </h5>
        <button class="btn btn-primary" @click="showFormCreate">
          Create Path
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
          <h2>Paths List</h2>
          <p style="margin-bottom: -8px; margin-top: -10px; font-weight: 300">
            Manage Paths Here
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
                @input="fetchPaths"
              />
            </div>
          </div>
          <table class="table table-hover">
            <thead class="thead-dark">
              <tr>
                <th scope="col">ID</th>
                <th scope="col">Name</th>
                <th scope="col">Site</th>
                <th scope="col" class="text-end">Action</th>
              </tr>
            </thead>
            <tbody>
              <tr v-if="paginatedMap.length === 0">
                <td colspan="5" class="text-center">No content found</td>
              </tr>
              <tr v-for="(mapPath, index) in paginatedMap" :key="mapPath.id">
                <th style="font-weight: 500">
                  {{ index + 1 + (currentPage - 1) * pageSize }}
                </th>
                <th style="font-weight: 500">{{ mapPath.name }}</th>
                <th style="font-weight: 500">{{ mapPath.map }}</th>
                <th class="d-flex justify-content-end">
                  <button @click="deleteMap(mapPath.id)" class="btn btn-danger">
                    Delete
                  </button>
                </th>
              </tr>
            </tbody>
          </table>
        </div>
      </div>
    </div>

    <div v-if="showForm || !showTable">
      <div class="d-flex justify-content-between align-items-center mb-3">
        <h5 style="color: #000; font-size: 25px">
          Create
          <span style="color: #0800ff">Path</span>
        </h5>

        <div class="d-flex align-items-center">
          <button class="btn btn-dark me-2" @click="cancelForm">Back</button>
          <button class="btn btn-success" type="submit" @click="submitForm">
            Save
          </button>
        </div>
      </div>

      <br />

      <div
        class="card bg-light"
        style="box-shadow: 5px 10px 8px #779bff; border-radius: 10px"
      >
        <transition name="expand">
          <div class="card-body" v-show="isFormCardExpanded">
            <form class="path" @submit.prevent="submitForm">
              <div
                class="d-flex justify-content-between align-items-center mb-3"
              >
                <div class="form-group flex-grow-1 mr-2">
                  <label for="pathName">Nama Path</label>
                  <input
                    type="text"
                    class="form-control"
                    id="pathName"
                    v-model="pathName"
                    placeholder="Enter Path Name"
                    required
                  />
                </div>
              </div>
              <div
                class="d-flex justify-content-between align-items-center mb-3"
              >
                <div class="form-group flex-grow-1">
                  <label for="mapSelect">Map</label>
                  <div class="input-group">
                    <select
                      class="form-control"
                      id="mapSelect"
                      v-model="selectedMap"
                    >
                      <option
                        v-for="map in maps"
                        :key="map.id"
                        :value="map.name"
                      >
                        {{ map.name }}
                      </option>
                    </select>
                    <button
                      class="btn btn-secondary"
                      type="button"
                      @click="launchMap"
                    >
                      Launch
                    </button>
                  </div>
                </div>
              </div>
              <div class="form-group">
                <label for="posX">Pos X</label>
                <input
                  type="text"
                  class="form-control"
                  id="posX"
                  v-model="posX"
                  readonly
                />
              </div>
              <div class="form-group">
                <label for="posY">Pos Y</label>
                <input
                  type="text"
                  class="form-control"
                  id="posY"
                  v-model="posY"
                  readonly
                />
              </div>
              <div class="form-group">
                <label for="orientation">Orientation</label>
                <input
                  type="text"
                  class="form-control"
                  id="orientation"
                  v-model="orientation"
                  readonly
                />
              </div>
              <div
                class="d-flex justify-content-between align-items-center mb-3"
              >
                <button
                  class="btn btn-info"
                  type="button"
                  @click="enableAutoFill"
                >
                  Enable Auto-Fill
                </button>
              </div>
            </form>
          </div>
        </transition>
        <button class="btn-expand" @click="toggleFormCard">
          <span v-if="isFormCardExpanded" class="material-symbols-outlined">
            collapse_all
          </span>
          <span v-else class="material-symbols-outlined"> swap_vert </span>
        </button>
      </div>
      <br />
    </div>
    <div class="card bg-light mt-3" v-show="showForm || !showTable">
      <div id="nav" ref="navContainer"></div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, computed, watch, nextTick } from "vue";
import axios from "axios";
import Swal from "sweetalert2";
import { useStore } from "vuex";
import ROSLIB from "roslib";

const isFormCardExpanded = ref(true);
const navContainer = ref(null);
const toggleFormCard = () => {
  isFormCardExpanded.value = !isFormCardExpanded.value;
};

// Vuex Store
const store = useStore();
const robots = computed(() => store.state.robots);
const connectedRobots = computed(() => store.state.connectedRobots);
const searchTerm = ref("");
const pathName = ref("");
const selectedMap = ref("");
const posX = ref("");
const posY = ref("");
const orientation = ref("");
const maps = ref([]);
const loading = ref(false);
const connected = ref(false);
const viewer = ref(null);
const nav = ref(null);
const ws = ref(null);
const autoFillEnabled = ref(false);
const mapPaths = ref([]);
const showTable = ref(true);
const showForm = ref(false);
const currentPage = ref(1); // Halaman saat ini
const pageSize = ref(5); // Jumlah item per halaman

// Menghitung total halaman berdasarkan jumlah item dan ukuran halaman
const totalPages = computed(() =>
  Math.ceil(mapPaths.value.length / pageSize.value)
);
const startIndex = computed(() => (currentPage.value - 1) * pageSize.value);
// Menghitung indeks akhir item pada halaman saat ini
const endIndex = computed(() =>
  Math.min(startIndex.value + pageSize.value - 1, mapPaths.value.length - 1)
);
// Memotong data misi menjadi halaman-halaman
const paginatedMap = computed(() =>
  mapPaths.value.slice(startIndex.value, endIndex.value + 1)
);

// Watchers to keep localStorage in sync with state
watch(showTable, (newValue) => {
  localStorage.setItem("showTable", JSON.stringify(newValue));
});

watch(showForm, (newValue) => {
  localStorage.setItem("showForm", JSON.stringify(newValue));
});

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
      localStorage.setItem("showTable", JSON.stringify(true));
      localStorage.setItem("showForm", JSON.stringify(false));
      showTable.value = true;
      showForm.value = false;
      resetForm();
      stopLaunch();
    }
  });
};

const resetForm = () => {
  pathName.value = "";
  posX.value = "";
  posY.value = "";
  orientation.value = "";
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

const deleteMap = async (id) => {
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
      await axios.delete(`http://localhost:5258/paths/${id}`);
      fetchPaths();
      Swal.fire("Deleted!", "Your path has been deleted.", "success");
    }
  } catch (error) {
    Swal.fire("Error", "Failed to delete map", "error");
    console.error("Failed to delete map:", error);
  }
};

const showFormCreate = () => {
  localStorage.setItem("showTable", JSON.stringify(false));
  localStorage.setItem("showForm", JSON.stringify(true));
  showTable.value = false;
  showForm.value = true;
  nextTick(() => {
    initConnection();
  });
};

const fetchPaths = async () => {
  try {
    const response = await axios.get("http://localhost:5258/paths");
    mapPaths.value = response.data.filter((mapPath) =>
      mapPath.name.toLowerCase().includes(searchTerm.value.toLowerCase())
    );
  } catch (error) {
    console.error("Failed to fetch maps:", error);
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

const submitForm = async () => {
  try {
    const formData = {
      name: pathName.value,
      map: selectedMap.value,
      posX: posX.value,
      posY: posY.value,
      orientation: orientation.value,
    };
    console.log("Submitting form data:", formData);
    const response = await axios.post("http://localhost:5258/paths", formData);
    if (response.status === 201) {
      // Periksa status kode 201 untuk Created
      Swal.fire({
        title: "Success",
        text: "Path saved successfully!",
        icon: "success",
        confirmButtonText: "OK",
      }).then(async (result) => {
        if (result.isConfirmed) {
          stopLaunch();
          showTable.value = true;
          showForm.value = false;
          resetForm();
          await fetchPaths(); // Perbarui data setelah form disubmit
        }
      });
    } else {
      throw new Error("Failed to save path");
    }
  } catch (error) {
    console.error("Error saving path:", error);
    Swal.fire("Error", "Failed to save path", "error");
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
    mapView();
  } catch (error) {
    console.error("Error launching map:", error);
  } finally {
    loading.value = false;
  }
};

const enableAutoFill = () => {
  autoFillEnabled.value = true;
  Swal.fire({
    title: "Auto Fill Enabled",
    text: "Click on the map to auto-fill the coordinates",
    icon: "info",
    confirmButtonText: "OK",
  });
};

const autoFillPath = (x, y, orientationValue) => {
  posX.value = x;
  posY.value = y;
  orientation.value = orientationValue;
  console.log(
    `Auto-filled coordinates: X=${x}, Y=${y}, Orientation=${orientationValue}`
  );
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

      viewer.value.scene.addEventListener("click", async (event) => {
        if (autoFillEnabled.value) {
          try {
            const coords = viewer.value.scene.globalToRos(
              event.stageX,
              event.stageY
            );
            const orientationValue = 0; // Gantilah dengan orientasi sesuai kebutuhan

            // Simpan koordinat ke dalam variabel yang diperlukan (posX, posY, orientation)
            autoFillPath(coords.x, coords.y, orientationValue);
            Swal.fire(
              "Coordinates Auto-filled",
              `X: ${coords.x}, Y: ${coords.y}, Orientation: ${orientationValue}`,
              "success"
            );
          } catch (error) {
            console.error("Error processing click:", error);
            Swal.fire("Error", "Failed to process click event", "error");
          }
        }
      });

      console.log("Nav setup complete");
    });
  }
};

// Fetch maps and initialize WebSocket connection on component mount
onMounted(() => {
  const showTableStorage = localStorage.getItem("showTable");
  const showFormStorage = localStorage.getItem("showForm");

  showTable.value =
    showTableStorage !== null ? JSON.parse(showTableStorage) : true;
  showForm.value =
    showFormStorage !== null ? JSON.parse(showFormStorage) : false;

  fetchMaps();
  initConnection();
  fetchPaths();

  // Ensure mapView is called if a robot is connected
  const connectedRobot = robots.value.find(
    (robot) => connectedRobots.value[robot.id]
  );
  if (connectedRobot) {
    mapView(connectedRobot);
  }
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
  background: none;
  width: 1rem;
  height: 1rem;
}

.btn-expand {
  display: block;
  width: 100%;
  background: none;
  border: none;
  text-align: center;
  margin-top: 10px; /* Adjust to your preference */
}

/* Transition for expanding/collapsing card */
.expand-enter-active,
.expand-leave-active {
  transition: ease-out 2ms;
}
.expand-enter, .expand-leave-to /* .expand-leave-active in <2.1.8 */ {
  max-height: 40px;
  overflow: hidden;
}
</style>
