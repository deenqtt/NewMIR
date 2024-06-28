<template>
  <div class="container">
    <br />
    <h5 style="color: #000; font-size: 25px">
      Add <span style="color: #0800ff">Robot</span>
    </h5>
    <br />
    <div
      class="card bg-light"
      style="box-shadow: 5px 10px 8px #779bff; border-radius: 10px"
    >
      <div class="card-body">
        <form @submit.prevent="submitForm">
          <div class="mb-3">
            <label for="name" class="form-label">Robot Name</label>
            <input
              type="text"
              v-model="newRobot.name"
              class="form-control"
              id="name"
              required
            />
          </div>
          <div class="mb-3">
            <label for="ip" class="form-label">IP Address</label>
            <input
              type="text"
              v-model="newRobot.ip"
              class="form-control"
              id="ip"
              required
            />
          </div>
          <div class="mb-3">
            <label for="port" class="form-label">Port</label>
            <input
              type="text"
              v-model="newRobot.port"
              class="form-control"
              id="port"
              required
            />
          </div>
          <button type="submit" class="btn btn-primary">Add Robot</button>
        </form>
      </div>
    </div>
  </div>
</template>

<script setup>
import axios from "axios";
import { onMounted, ref } from "vue";
import { useStore } from "vuex";
import Swal from "sweetalert2";
const store = useStore(); // Mengakses store Vuex di dalam setup
const apiUrl = "http://localhost:5258/robots";
const errorMessage = ref("");
const robots = ref([]);

const newRobot = ref({
  name: "",
  ip: "",
  port: "",
});
const fetchRobots = async () => {
  try {
    const response = await axios.get(apiUrl);
    robots.value = response.data;
  } catch (error) {
    errorMessage.value = "Failed to fetch robots: " + error.message;
  }
};
//submit
const submitForm = async () => {
  try {
    const response = await axios.post(apiUrl, newRobot.value);
    console.log(response.data);

    const robotsResponse = await axios.get(apiUrl);
    robots.value = robotsResponse.data;

    newRobot.value = {
      name: "",
      ip: "",
      port: "",
    };

    // Tampilkan sweet notification untuk memberi tahu pengguna bahwa robot berhasil disimpan
    await Swal.fire("Success!", "Robot successfully saved!", "success");

    // Kirim pesan notifikasi ke store Vuex
    store.commit("addNotification", "Robot successfully saved!");
  } catch (error) {
    console.error(error);
    errorMessage.value = "Failed to save robot: " + error.message;
  }
};
//clear form submit
const clearForm = () => {
  // Kosongkan nilai pada newRobot
  newRobot.value = {
    Name: "",
    Serialnumber: "",
    Ip: "",
    DomainId: "",
  };

  // Tampilkan sweet notification untuk memberi tahu pengguna bahwa form berhasil dikosongkan
  Swal.fire("Success!", "Form successfully cleared!", "success");
};

onMounted(() => {
  console.log("Component mounted");
  fetchRobots();
});
</script>

<style scoped></style>
