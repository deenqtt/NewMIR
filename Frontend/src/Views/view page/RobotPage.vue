<template>
  <div class="container">
    <h5>
      Robot
      <span>Management</span>
    </h5>
    <br />
    <!-- Card untuk menampilkan daftar robot (tanpa action) -->
    <div class="card bg-light">
      <div class="card-header">List Of Robot</div>
      <div class="card-body">
        <table class="table table-hover">
          <thead>
            <tr>
              <th>ID</th>
              <th>Name</th>
              <th>IP</th>
              <th>Port</th>
              <th>Status</th>
              <th class="text-end">Action</th>
            </tr>
          </thead>
          <tbody v-if="paginatedRobots.length > 0">
            <tr v-for="(robot, index) in paginatedRobots" :key="robot.id">
              <td>{{ index + 1 + (currentPage - 1) * pageSize }}</td>
              <td>{{ robot.name }}</td>
              <td>{{ robot.ip }}</td>
              <td>{{ robot.port }}</td>
              <td
                :class="isConnected(robot.id) ? 'text-success' : 'text-danger'"
              >
                {{ isConnected(robot.id) ? "Connected" : "Disconnected" }}
              </td>
              <td class="text-end">
                <button @click="deleteRobot(robot.id)" class="btn btn-danger">
                  Delete
                </button>
              </td>
            </tr>
          </tbody>
        </table>
        <!-- Pagination -->
        <nav aria-label="Page navigation example">
          <ul class="pagination justify-content-start">
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
</template>

<script setup>
import { onMounted, ref, computed } from "vue";
import { useStore } from "vuex";
import axios from "axios";
import Swal from "sweetalert2";

const store = useStore();
const robots = computed(() => store.state.robots);
const connectedRobots = computed(() => store.state.connectedRobots);

const apiUrl = "http://localhost:5258/robots";

const currentPage = ref(1);
const pageSize = 5; // Jumlah item per halaman

const paginatedRobots = computed(() => {
  const startIndex = (currentPage.value - 1) * pageSize;
  return robots.value.slice(startIndex, startIndex + pageSize);
});

const totalPages = computed(() => Math.ceil(robots.value.length / pageSize));

const fetchRobots = async () => {
  await store.dispatch("fetchRobots");
};

const isConnected = (robotId) => {
  return connectedRobots.value[robotId] || false;
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

const deleteRobot = async (id) => {
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
      await axios.delete(`http://localhost:5258/robots/${id}`);
      fetchRobots();
      Swal.fire("Deleted!", "Your robot has been deleted.", "success");
    }
  } catch (error) {
    Swal.fire("Error", "Failed to delete robot", "error");
    console.error("Failed to delete robot:", error);
  }
};

onMounted(() => {
  fetchRobots();
});
</script>

<style scoped>
.container {
  font-family: "Poppins", sans-serif;
  display: flex;
  flex-direction: column;
}

h5 {
  font-size: 25px;
  font-weight: 700;
  color: #000000;
  margin-top: 20px;
}

span {
  font-size: 25px;
  font-weight: 700;
  color: #0800ff;
}

.card {
  margin-right: 40px;
  box-shadow: 5px 10px 8px #779bff;
  border-radius: 10px;
}

.card-header {
  margin: 10px;
  border-radius: 10px;
  font-size: 15px;
}

th {
  font-size: 13px;
}

.dropdown-menu {
  width: 100%;
}

.dropdown-item {
  display: flex;
  justify-content: space-between;
}

.dropdown-item button {
  margin-left: 10px;
}

.pagination {
  justify-content: start; /* Mengubah posisi pagination ke kiri */
}

.modal-title {
  color: #0800ff;
}

.modal-title span {
  color: #000;
}

.modal-content {
  border-radius: 10px;
}

.modal-body input {
  margin-bottom: 10px;
}

.modal-footer {
  justify-content: flex-end;
}

.btn-primary {
  background-color: #0800ff;
  border-color: #0800ff;
}

.btn-primary:hover {
  background-color: #0600cc;
  border-color: #0600cc;
}

.btn-outline-primary {
  color: #0800ff;
  border-color: #0800ff;
}

.btn-outline-primary:hover {
  color: #ffffff;
  background-color: #0800ff;
  border-color: #0800ff;
}

.btn-outline-danger {
  color: #ff0000;
  border-color: #ff0000;
}

.btn-outline-danger:hover {
  color: #ffffff;
  background-color: #ff0000;
  border-color: #ff0000;
}

.text-success {
  color: green !important;
}

.text-danger {
  color: red !important;
}
</style>
