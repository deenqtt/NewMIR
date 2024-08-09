<template>
  <div class="container">
    <div class="d-flex align-items-center">
      <h3>Add Dock</h3>
      <button @click="confirmBack" class="btn btn-secondary ml-3">Back</button>
      <button @click="saveDock" class="btn btn-success ml-3">Save</button>
    </div>
    <div class="card bg-light mt-3">
      <div
        class="card-header d-flex justify-content-between align-items-center"
      >
        <h4>Select Dock Location</h4>
        <button @click="enableAutoFill" class="btn btn-primary">
          Add Dock Point
        </button>
      </div>
      <div class="card-body">
        <div
          id="nav"
          ref="navContainer"
          style="width: 100%; height: 550px"
        ></div>
        <div v-if="docks.length > 0" class="mt-3">
          <h5>Docking Points</h5>
          <ul class="list-group">
            <li v-for="dock in docks" :key="dock.id" class="list-group-item">
              X: {{ dock.posX }}, Y: {{ dock.posY }}, Orientation:
              {{ dock.orientation }}
              <button
                @click="deleteDock(dock.id)"
                class="btn btn-danger btn-sm float-right"
              >
                Delete
              </button>
            </li>
          </ul>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, nextTick } from "vue";
import { useRoute, useRouter } from "vue-router";
import axios from "axios";
import Swal from "sweetalert2";

const route = useRoute();
const router = useRouter();
const navContainer = ref(null);
const viewer = ref(null);
const nav = ref(null);
const ws = ref(null);
const autoFillEnabled = ref(false);
const dockCoords = ref({ x: 0, y: 0, orientation: 0 });
const docks = ref([]);

const confirmBack = () => {
  Swal.fire({
    title: "Are you sure?",
    text: "If you go back, all your changes will be lost.",
    icon: "warning",
    showCancelButton: true,
    confirmButtonColor: "#3085d6",
    cancelButtonColor: "#d33",
    confirmButtonText: "Yes, go back!",
  }).then(async (result) => {
    if (result.isConfirmed) {
      await stopLaunch(); // Stop the map launch
      router.push("/maps"); // Ganti dengan rute yang sesuai
    }
  });
};

const saveDock = async () => {
  try {
    const response = await axios.post("http://localhost:5258/maps/save-dock", {
      mapId: route.params.id,
      posX: dockCoords.value.x,
      posY: dockCoords.value.y,
      orientation: dockCoords.value.orientation,
    });

    if (response.status === 200) {
      Swal.fire({
        title: "Success!",
        text: "Dock coordinates saved successfully.",
        icon: "success",
        confirmButtonText: "OK",
      }).then(async () => {
        await stopLaunch(); // Stop the map launch
        router.push("/maps");
      });
    }
  } catch (error) {
    console.error("Failed to save dock coordinates:", error);
    Swal.fire("Error", "Failed to save dock coordinates", "error");
  }
};

const stopLaunch = async () => {
  try {
    const response = await axios.post("http://localhost:5258/maps/stop");
    if (response.status === 200) {
      console.log("Map launch process terminated.");
    } else {
      console.warn("No map launch process is currently running.");
    }
  } catch (error) {
    console.error("Error stopping map launch:", error);
  }
};

const fetchDocks = async () => {
  try {
    const response = await axios.get(
      `http://localhost:5258/maps/get-dock/${route.params.id}`
    );
    docks.value = response.data ? [response.data] : [];
  } catch (error) {
    console.error("Failed to fetch dock coordinates:", error);
  }
};

const deleteDock = async (dockId) => {
  try {
    const response = await axios.delete(
      `http://localhost:5258/maps/delete-dock/${dockId}`
    );
    if (response.status === 200) {
      Swal.fire({
        title: "Deleted!",
        text: "Dock point has been deleted.",
        icon: "success",
        confirmButtonText: "OK",
      });
      fetchDocks(); // Refresh the list of docks
    }
  } catch (error) {
    console.error("Failed to delete dock point:", error);
    Swal.fire("Error", "Failed to delete dock point", "error");
  }
};

const initConnection = () => {
  ws.value = new WebSocket("ws://localhost:9090"); // Replace with actual websocket URL

  ws.value.onopen = () => {
    console.log("WebSocket connection established");
    mapView();
  };

  ws.value.onclose = () => {
    console.log("Connection closed");
    setTimeout(initConnection, 5000);
  };

  ws.value.onerror = (error) => {
    console.log("WebSocket error: ", error);
    setTimeout(initConnection, 5000);
  };
};

const mapView = () => {
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
          url: "ws://localhost:9090", // Replace with actual websocket URL
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
            dockCoords.value.x = coords.x;
            dockCoords.value.y = coords.y;
            dockCoords.value.orientation = 0; // Set orientation as needed

            Swal.fire(
              "Coordinates Set",
              `X: ${coords.x}, Y: ${coords.y}`,
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

const enableAutoFill = () => {
  autoFillEnabled.value = true;
  Swal.fire({
    title: "Auto Fill Enabled",
    text: "Click on the map to auto-fill the coordinates",
    icon: "info",
    confirmButtonText: "OK",
  });
};

onMounted(() => {
  initConnection();
  fetchDocks(); // Fetch the list of docks for the selected map
});
</script>
