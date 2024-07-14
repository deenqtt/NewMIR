<template>
  <div class="container">
    <div v-if="Tableshow">
      <br />
      <div class="d-flex">
        <h5 style="color: #000; font-size: 25px">
          Maps
          <span style="color: #0800ff">Management</span>
        </h5>

        <button @click="showCreateForm" class="btn btn-primary mb-3">
          Create Map
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
          <h2>Maps List</h2>
          <p style="margin-bottom: -8px; margin-top: -10px; font-weight: 300">
            Manage Maps Here
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
                @input="fetchMaps"
              />
            </div>
          </div>

          <table class="table table-hover">
            <thead class="thead-dark">
              <tr>
                <th scope="col">ID</th>
                <th scope="col">Name</th>
                <th scope="col">Site</th>
                <th scope="col">Timestamp</th>
                <th class="text-end">Actions</th>
              </tr>
            </thead>
            <tbody>
              <tr v-if="paginatedMap.length === 0">
                <td colspan="5" class="text-center">No content found</td>
              </tr>
              <tr v-for="(map, index) in paginatedMap" :key="map.id">
                <td>{{ index + 1 + (currentPage - 1) * pageSize }}</td>
                <td>{{ map.name }}</td>
                <td>{{ map.site }}</td>
                <td>{{ new Date(map.timestamp).toLocaleString() }}</td>
                <td class="text-end">
                  <button @click="deleteMap(map.id)" class="btn">Delete</button>
                  <button
                    @click="editMap(map.id)"
                    class="btn"
                    style="margin-left: 20px"
                  >
                    Edit
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

    <div v-if="Formshow || !Tableshow">
      <br />
      <div class="d-flex">
        <h5 style="color: #000; font-size: 25px">
          Create
          <span style="color: #0800ff">Map</span>
        </h5>
        <button type="submit" class="btn btn-success" @click="saveMap">
          Save
        </button>
      </div>
      <button
        @click="cancelCreate"
        class="btn"
        style="position: absolute; margin-left: 160px; margin-top: -45px"
      >
        <span class="material-symbols-outlined"> undo </span>
      </button>
      <br />

      <form @submit.prevent="saveMap">
        <div
          class="card bg-light"
          style="box-shadow: 5px 10px 8px #779bff; border-radius: 10px"
        >
          <transition name="expand">
            <div class="card-body" v-show="isFormCardExpanded">
              <!-- Form Inputs -->
              <div class="mb-3">
                <label for="name" class="form-label">Map Name</label>
                <input
                  type="text"
                  class="form-control"
                  id="name"
                  v-model="map.name"
                  required
                />
              </div>
              <div class="mb-3">
                <label for="site" class="form-label">Site Name</label>
                <input
                  type="text"
                  class="form-control"
                  id="site"
                  v-model="map.site"
                  required
                />
              </div>
            </div>
          </transition>
          <button class="btn-expand" @click.prevent="toggleFormCard">
            <span v-if="isFormCardExpanded" class="material-symbols-outlined">
              collapse_all
            </span>
            <span v-else class="material-symbols-outlined"> swap_vert </span>
          </button>
        </div>

        <br />
        <div class="d-flex">
          <label for="mapCanvas" class="form-label">Map</label>
          <button type="button" @click="startMapping" class="btn btn-primary">
            Start Mapping
          </button>
        </div>
        <div class="card">
          <div class="card-header"></div>
          <div id="nav" ref="navContainer"></div>
        </div>
      </form>
    </div>
  </div>
</template>

<script setup>
import {
  ref,
  reactive,
  nextTick,
  onMounted,
  onBeforeUnmount,
  computed,
} from "vue";
import axios from "axios";
import { useStore } from "vuex";
import Swal from "sweetalert2";
const navClient = ref(null);
import { useRouter } from "vue-router"; // Import useRouter
const searchTerm = ref("");
const router = useRouter(); // Initialize the router
const store = useStore();
const robots = computed(() => store.state.robots);
const connectedRobots = computed(() => store.state.connectedRobots);
const viewer = ref(null);
const currentPage = ref(1);
const pageSize = ref(5);
const isFormCardExpanded = ref(true);

const toggleFormCard = () => {
  isFormCardExpanded.value = !isFormCardExpanded.value;
};
const totalPages = computed(() =>
  Math.ceil(filteredMaps.value.length / pageSize.value)
);
const startIndex = computed(() => (currentPage.value - 1) * pageSize.value);
const endIndex = computed(() =>
  Math.min(startIndex.value + pageSize.value, filteredMaps.value.length)
);
const paginatedMap = computed(() =>
  filteredMaps.value.slice(startIndex.value, endIndex.value)
);

const maps = ref([]);
const filteredMaps = ref([]);
const map = reactive({
  name: "",
  site: "",
});
const Tableshow = ref(true);
const Formshow = ref(false);
const cardWidth = ref(700); // Initial width

const fetchMaps = async () => {
  try {
    const response = await axios.get("http://localhost:5258/maps");
    maps.value = response.data;
    applySearchFilter();
  } catch (error) {
    console.error("Failed to fetch maps:", error);
  }
};

const showCreateForm = () => {
  localStorage.setItem("Tableshow", JSON.stringify(false));
  localStorage.setItem("Formshow", JSON.stringify(true));
  Tableshow.value = false;
  Formshow.value = true;
  nextTick(() => {
    initConnection();
  });
};

const stopMapping = () => {
  fetch("http://localhost:5258/stop_mapping")
    .then((response) => {
      if (response.ok) {
        console.log("Mapping process stopped.");
      } else {
        console.log("Failed to stop mapping process.");
      }
    })
    .catch((error) => {
      console.error("Error:", error);
    });
};
const navContainer = ref(null);
const cancelCreate = () => {
  stopMapping();
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
      localStorage.setItem("Tableshow", JSON.stringify(true));
      localStorage.setItem("Formshow", JSON.stringify(false));
      Tableshow.value = true;
      Formshow.value = false;
      resetForm();
    }
  });
};

const resetForm = () => {
  map.name = "";
  map.site = "";
};

const saveMap = async () => {
  try {
    const response = await axios.post("http://localhost:5258/maps/save", {
      Name: map.name,
      Site: map.site,
      Resolution: map.resolution,
      OriginX: map.originX,
      OriginY: map.originY,
      OriginZ: map.originZ,
    });
    if (response.status === 200) {
      fetchMaps();
      resetForm();
      stopMapping();
      localStorage.setItem("Tableshow", JSON.stringify(true));
      localStorage.setItem("Formshow", JSON.stringify(false));
      Tableshow.value = true;
      Formshow.value = false;
      Swal.fire("Success", "Map saved successfully", "success");
    }
  } catch (error) {
    Swal.fire("Error", "Failed to save map", "error");
    console.error("Failed to save map:", error);
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
      await axios.delete(`http://localhost:5258/maps/${id}`);
      fetchMaps();
      Swal.fire("Deleted!", "Your map has been deleted.", "success");
    }
  } catch (error) {
    Swal.fire("Error", "Failed to delete map", "error");
    console.error("Failed to delete map:", error);
  }
};

const initConnection = () => {
  const connectedRobot = robots.value.find(
    (robot) => connectedRobots.value[robot.id]
  );
  if (connectedRobot) {
    const ros = new ROSLIB.Ros();
    ros.connect(`ws://${connectedRobot.ip}:${connectedRobot.port}`);

    const viewer = new ROS2D.Viewer({
      divID: "nav",
      width: navContainer.value.clientWidth,
      height: 400,
    });

    const gridClient = new ROS2D.OccupancyGridClient({
      ros: ros,
      rootObject: viewer.scene,
      continuous: true,
    });

    gridClient.on("change", () => {
      viewer.scaleToDimensions(
        gridClient.currentGrid.width,
        gridClient.currentGrid.height
      );
      viewer.shift(
        gridClient.currentGrid.pose.position.x,
        gridClient.currentGrid.pose.position.y
      );
    });
  }
};
const editMap = (id) => {
  // Redirect to Edit page with the map ID
  router.push({ name: "Edit", params: { id } });
};

const startMapping = () => {
  fetch("http://localhost:5258/start_mapping")
    .then((response) => {
      if (response.ok) {
        alert("Mapping process started.");
      } else {
        alert("Failed to start mapping process.");
      }
    })
    .catch((error) => {
      console.error("Error:", error);
      alert("Error starting mapping process.");
    });
};

const prevPage = () => {
  if (currentPage.value > 1) {
    currentPage.value--;
  }
};

const nextPage = () => {
  if (currentPage.value < totalPages.value) {
    currentPage.value++;
  }
};

const changePage = (page) => {
  currentPage.value = page;
};

const applySearchFilter = () => {
  filteredMaps.value = maps.value.filter((map) =>
    map.name.toLowerCase().includes(searchTerm.value.toLowerCase())
  );
};

onMounted(() => {
  const showTable = localStorage.getItem("Tableshow");
  const showForm = localStorage.getItem("Formshow");

  Tableshow.value = showTable !== null ? JSON.parse(showTable) : true;
  Formshow.value = showForm !== null ? JSON.parse(showForm) : false;

  fetchMaps();
});
</script>

<style scoped>
.nav {
  align-items: center;
  margin-left: 20px;
}
#mapCanvas {
  border: 1px solid #d3d3d3;
  width: 100%;
  height: 100%;
}
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
.expand-enter,
.expand-leave-to /* .expand-leave-active in <2.1.8 */ {
  max-height: 40px;
  overflow: hidden;
}
</style>
