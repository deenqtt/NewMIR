// store.js
import Swal from "sweetalert2";
import { createStore } from "vuex";
import axios from "axios";
export default createStore({
  state: {
    robots: [],
    connectedRobots: {},
    selectedRobot: "",
    robotOptions: [],
    filteredMissions: [],
    notifications: [],
    unreadNotifications: 0,
    errorMessage: "",
  },
  mutations: {
    setRobots(state, robots) {
      state.robots = robots;
    },
    setRobotConnected(state, { robotId, connected }) {
      state.connectedRobots[robotId] = connected;
    },
    setConnectedRobots(state, connectedRobots) {
      state.connectedRobots = connectedRobots;
    },
    setSelectedRobot(state, robotName) {
      state.selectedRobot = robotName;
    },
    setRobotOptions(state, robotOptions) {
      state.robotOptions = robotOptions;
    },
    setFilteredMissions(state, missions) {
      state.filteredMissions = missions;
    },
    addNotification(state, notification) {
      state.notifications.push(notification);
      state.unreadNotifications++;
    },
    markAllNotificationsAsRead(state) {
      state.notifications.forEach((notification) => {
        notification.read = true;
      });
      state.unreadNotifications = 0;
    },
    updateUnreadNotifications(state) {
      state.unreadNotifications--;
    },
    setErrorMessage(state, errorMessage) {
      state.errorMessage = errorMessage;
    },
    removeNotification(state, notificationIndex) {
      state.notifications.splice(notificationIndex, 1);
      state.unreadNotifications--;
    },
  },
  actions: {
    async removeNotification({ commit, state }, notificationIndex) {
      try {
        commit("removeNotification", notificationIndex);
      } catch (error) {
        console.error("Failed to remove notification:", error);
      }
    },
    async fetchRobots({ commit }) {
      try {
        const response = await axios.get("http://localhost:5258/robots");
        const robots = response.data.map((robot) => ({
          ...robot,
          ros: null,
        }));
        commit("setRobots", robots);

        // Load connection statuses from localStorage
        const storedConnections =
          JSON.parse(localStorage.getItem("connectedRobots")) || {};
        commit("setConnectedRobots", storedConnections);

        // Automatically connect to robots that were previously connected
        robots.forEach((robot) => {
          if (storedConnections[robot.id]) {
            this.dispatch("connectRobot", robot);
          }
        });
      } catch (error) {
        console.error("Error fetching robots:", error);
      }
    },
    connectRobot({ commit, state }, robot) {
      const ros = new ROSLIB.Ros({
        url: `ws://${robot.ip}:${robot.port}`,
      });

      ros.on("connection", () => {
        console.log(
          `Connected to ${robot.name} at ws://${robot.ip}:${robot.port}`
        );
        commit("setRobotConnected", { robotId: robot.id, connected: true });
        robot.connected = true;
        robot.ros = ros;
        localStorage.setItem(
          "connectedRobots",
          JSON.stringify(state.connectedRobots)
        );
      });

      ros.on("error", (error) => {
        console.error(`Error connecting to ${robot.name}:`, error);
        Swal.fire({
          icon: "error",
          title: "Connection Error!",
          text: `Failed to connect to ${robot.name}: ${error.message}`,
        });
        commit("setRobotConnected", { robotId: robot.id, connected: false }); // Ubah status koneksi menjadi false jika terjadi kesalahan
        robot.connected = false;
        robot.ros = null;
        localStorage.setItem(
          "connectedRobots",
          JSON.stringify(state.connectedRobots)
        );
      });

      ros.on("close", () => {
        console.log(`Connection to ${robot.name} closed.`);
        commit("setRobotConnected", { robotId: robot.id, connected: false });
        robot.connected = false;
        robot.ros = null;
        localStorage.setItem(
          "connectedRobots",
          JSON.stringify(state.connectedRobots)
        );
      });
    },

    disconnectRobot({ commit, state }, robot) {
      if (robot.ros) {
        robot.ros.close();
        commit("setRobotConnected", { robotId: robot.id, connected: false });
        robot.connected = false;
        robot.ros = null;
        console.log(`Disconnected from ${robot.name}`);
        localStorage.setItem(
          "connectedRobots",
          JSON.stringify(state.connectedRobots)
        );
      }
    },
  },
});
