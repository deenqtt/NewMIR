<template>
  <div class="container">
    <br />
    <br />
    <br />
    <div class="d-flex">
      <router-link to="/Maps" class="btn btn-light"> Back</router-link>
      <router-link to="/Maps" class="btn btn-success"> Save Maps</router-link>
    </div>
    <div class="card bg-light">
      <div class="card-header">
        <div class="d-flex">
          <span class="material-symbols-outlined"> open_with </span>
          <span class="material-symbols-outlined"> save </span>
          <p>Save All</p>
        </div>
        <button
          data-toggle="modal"
          data-target="#exampleModal"
          class="material-symbols-outlined"
        >
          widgets
        </button>
      </div>
      <div class="card-body">
        <div id="nav"></div>
      </div>
    </div>
    <div
      class="modal fade"
      id="exampleModal"
      tabindex="-1"
      role="dialog"
      aria-labelledby="exampleModalLabel"
      aria-hidden="true"
    >
      <div class="modal-dialog" role="document">
        <div class="modal-content">
          <div class="modal-header">
            <h5 class="modal-title" id="exampleModalLabel">Modal title</h5>
            <button
              type="button"
              class="close"
              data-dismiss="modal"
              aria-label="Close"
            >
              <span aria-hidden="true">&times;</span>
            </button>
          </div>
          <div class="modal-body">
            <div class="import">
              <button class="material-symbols-outlined">cloud_download</button>
              <p>Import</p>
            </div>
            <div class="record">
              <button class="material-symbols-outlined">
                radio_button_checked
              </button>
              <p>Record Map</p>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
  <!-- Modal -->
</template>

<script setup>
import Config from "../../data/config";
import { onMounted, ref } from "vue";
let connected = false;
let ws = null;
let viewer2 = null;
let nav = null;

const initConnection = () => {
  ws = new WebSocket(
    `ws://${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`
  );

  ws.onopen = () => {
    console.log("WebSocket connection established");
    connected = true;
    mapView();
  };

  ws.onclose = () => {
    console.log("Connection closed");
    connected = false;
    setTimeout(initConnection, 5000);
  };

  ws.onerror = (error) => {
    console.log("WebSocket error: ", error);
    setTimeout(initConnection, 5000);
  };
};

const mapView = () => {
  if (connected) {
    const navElement = navRef.value;
    if (!navElement) {
      console.error("Element with ID 'nav' not found in DOM.");
      return;
    }

    viewer2 = new ROS2D.Viewer({
      divID: "nav",
      width: 800,
      height: 550,
    });

    nav = new NAV2D.OccupancyGridClientNav({
      ros: new ROSLIB.Ros({
        url: `ws://${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`,
      }),
      rootObject: viewer2.scene,
      viewer: viewer2,
      serverName: "/navigate_to_pose",
      topic: "/map",
      withOrientation: false,
    });

    console.log("Nav setup complete");
  }
};

const handleInitialize = () => {
  if (nav) {
    nav.initializeRobotMarker();
  }
};

const navRef = ref(null);

onMounted(() => {
  // Set the reference to the 'nav' div element
  navRef.value = document.getElementById("nav");
  // Initialize the connection and map view
  initConnection();
});
</script>
<style scoped>
canvas {
  border: #000 solid 1px;
}
.modal-content {
  text-align: center;
}

.modal-body {
  display: flex;
  justify-content: space-around;
  align-items: center;
}

.import,
.record {
  text-align: center;
}

.modal .import .material-symbols-outlined {
  font-size: 36px; /* Sesuaikan ukuran ikon sesuai kebutuhan */
  color: #007bff; /* Sesuaikan warna ikon sesuai kebutuhan */
}
.modal .record .material-symbols-outlined {
  font-size: 36px; /* Sesuaikan ukuran ikon sesuai kebutuhan */
  color: #ff0000; /* Sesuaikan warna ikon sesuai kebutuhan */
}
.modal button {
  border: none;
  background: none;
}
.import p,
.record p {
  margin: 0;
  font-size: 14px; /* Sesuaikan ukuran font sesuai kebutuhan */
}
input {
  height: 20px;
  margin-top: -25px;
  font-size: 12px;
}
.container {
  font-family: "Poppins", sans-serif;
  display: flex;
  flex-direction: column; /* Mengatur tata letak elemen dalam satu kolom */
}

h5 {
  font-size: 25px;
  font-weight: 700;
  color: #0800ff;
  margin-top: 20px;
}
span {
  font-size: 25px;
  font-weight: 500;
  color: #000;
}

p {
  margin-top: -10px;
  font-size: 12px;
  font-weight: 100; /* Memberikan jarak atas antara h5 dan p */
}
.container .btn {
  text-align: center;
  width: 120px;
  color: #000;
  font-size: 12px;
  font-weight: 600;
  height: 30px;
  align-self: flex-end;
  margin-right: 40px;
  margin-top: -50px;
  margin-bottom: 20px;
}
.card {
  margin-right: 40px;
  box-shadow: 1px 2px 1px #000;
  border-radius: 10px;
}
.card-header {
  margin: 10px;
  border-radius: 10px;
  font-size: 15px;
  display: flex; /* atau display: grid; */

  justify-content: space-between;

  align-items: center;
}
th {
  font-size: 13px;
}
.card-header span {
  margin-right: 40px;
}
.card-header p {
  font-size: 12px;
  font-weight: 500;
  margin-top: 5px;
  margin-left: -30px;
}

.card-header button {
  border: none;
  margin-top: -11px;
  justify-self: end;
}
</style>
