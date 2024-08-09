<template>
  <div class="sidebar">
    <button
      class="hamburger-button"
      style="color: aliceblue"
      @click="toggleSidebar"
    >
      <i class="fa-solid fa-bars"></i>
    </button>
    <aside
      :class="`${is_expanded ? 'is-expanded' : ''} ${
        is_mobile ? 'is-mobile' : ''
      }`"
    >
      <button v-if="is_expanded" class="close-button" @click="toggleSidebar">
        <i class="fa-solid fa-times"></i>
      </button>
      <br /><br />
      <div class="menu">
        <div class="menu1">
          <router-link
            to="/dashboard"
            class="button"
            :class="{ active: $route.path === '/dashboard' }"
          >
            <lord-icon
              class="fa-solid"
              src="https://cdn.lordicon.com/wmwqvixz.json"
              trigger="hover"
              colors="primary:#ffffff"
              data-toggle="tooltip"
              data-bs-placement="right"
              title="Dashboard"
            ></lord-icon>
            <span class="text" v-if="is_expanded">Dashboard</span>
          </router-link>
        </div>
        <router-link
          to="/maps"
          class="button"
          :class="{ active: $route.path === '/maps' }"
        >
          <lord-icon
            class="fa-solid"
            src="https://cdn.lordicon.com/omiqopzf.json"
            trigger="hover"
            colors="primary:#ffffff"
            data-toggle="tooltip"
            data-bs-placement="right"
            title="Setup"
          ></lord-icon>
          <span class="text" v-if="is_expanded">Setup</span>
        </router-link>
        <router-link
          to="/activity"
          class="button"
          :class="{ active: $route.path === '/activity' }"
          @click="handleNotificationClick"
        >
          <lord-icon
            class="fa-solid"
            src="https://cdn.lordicon.com/uwinmnkh.json"
            trigger="hover"
            colors="primary:#ffffff"
            data-toggle="tooltip"
            data-bs-placement="right"
            title="Monitoring"
          ></lord-icon>
          <span class="text" v-if="is_expanded">Monitoring</span>
          <span class="notification-dot" v-if="hasNewInputError"></span>
        </router-link>
        <router-link
          to="/settings"
          class="button"
          :class="{ active: $route.path === '/settings' }"
        >
          <lord-icon
            src="https://cdn.lordicon.com/ifsxxxte.json"
            trigger="hover"
            colors="primary:#ffffff"
            class="fa-solid"
            data-toggle="tooltip"
            data-bs-placement="right"
            title="System"
          ></lord-icon>
          <span class="text" v-if="is_expanded">System</span>
        </router-link>
      </div>

      <div class="flex"></div>

      <div class="menu">
        <router-link
          to="/Robot"
          class="button"
          :class="{ active: $route.path === '/Robot' }"
        >
          <lord-icon
            class="fa-solid"
            src="https://cdn.lordicon.com/ojnjgkun.json"
            trigger="hover"
            colors="primary:#ffffff"
            data-toggle="tooltip"
            data-bs-placement="right"
            title="Help"
          ></lord-icon>
          <span class="text" v-if="is_expanded">Help</span>
        </router-link>
        <button class="tombol" @click="logout">
          <span
            class="fa-solid fa-right-from-bracket custom-tooltip"
            data-toggle="tooltip"
            data-bs-placement="right"
            title="Logout"
          ></span>
          <span class="text" v-if="is_expanded">Logout</span>
        </button>
      </div>
      <div v-if="loading" class="loading-overlay">
        <div class="loading-spinner"></div>
      </div>
      <div v-if="is_mobile && is_expanded" class="background-overlay"></div>
    </aside>
  </div>
</template>

<script setup>
import { ref, onMounted, onBeforeUnmount, computed } from "vue";
import { useRouter } from "vue-router";
import { removeAuthToken } from "../router/auth";
import { useStore } from "vuex";
import store from "../store";
const loading = ref(false);
const router = useRouter();

const hasNewInputError = computed(() => {
  const unreadNotifications = store.state.unreadNotifications;
  return unreadNotifications > 0;
});
const handleNotificationClick = () => {
  store.commit("markAllNotificationsAsRead");
  hasNewInputError.value = false;
};

const logout = async () => {
  loading.value = true;
  document.body.style.overflow = "hidden";
  await new Promise((resolve) => setTimeout(resolve, 3000));
  removeAuthToken();
  router.push({ name: "Login" });
};

const is_expanded = ref(false);
const is_mobile = ref(window.innerWidth <= 660);

const toggleSidebar = () => {
  is_expanded.value = !is_expanded.value;
};

const checkWindowSize = () => {
  is_mobile.value = window.innerWidth <= 660;
};

onMounted(() => {
  checkWindowSize();
  window.addEventListener("resize", checkWindowSize);
});

onBeforeUnmount(() => {
  window.removeEventListener("resize", checkWindowSize);
});
</script>

<style lang="scss" scoped>
.is-expanded .background-overlay {
  display: block;
}

.close-button {
  position: fixed;
  top: 20px;
  left: 220px;
  z-index: 9999;
  background: none;
  border: none;
  cursor: pointer;
  padding: 0;
}

.is-expanded {
  width: 240px;
  transition: width 0.3s ease;
}

.is-expanded .close-button {
  display: block;
}

.hamburger-button {
  position: fixed;
  display: none;
  left: 10px;
  z-index: 9999;
  background: none;
  border: none;
  cursor: pointer;
  padding: 0;
}
.fa-times {
  color: #ffff;
}

.background-overlay {
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background-color: rgba(0, 0, 0, 0.5);
  z-index: 99909;
  display: none;
}
.fa-bars {
  color: #000000;
}

@media screen and (max-width: 660px) {
  .hamburger-button {
    display: block;
  }
  .close-button {
    display: none;
  }
  .is-mobile {
    display: none;
  }
  aside {
    position: absolute;
    top: 0;
    left: 0;
    bottom: 0;
    z-index: 999999;
  }
  .is-mobile.is-expanded {
    display: block;
  }
}
.loading-overlay {
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background-color: rgba(0, 0, 0, 0.5);
  display: flex;
  justify-content: center;
  align-items: center;
  z-index: 9999;
}

.loading-spinner {
  border: 8px solid #f3f3f3;
  border-top: 8px solid #3498db;
  border-radius: 50%;
  width: 50px;
  height: 50px;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  0% {
    transform: rotate(0deg);
  }
  100% {
    transform: rotate(360deg);
  }
}

.loading-overlay + .is-expanded {
  filter: blur(5px);
}
.notification-dot {
  position: absolute;
  width: 10px;
  height: 10px;
  background-color: rgb(0, 30, 255);
  border-radius: 50%;
  top: 15px;
  right: 19px;
}

.tombol {
  margin-top: 20px;
  margin-left: 2px;
  background: none;
  color: #fff;
  border: none;
  display: flex;
  align-items: center;
  padding: 1rem 1rem;
  font-size: 1.5rem;
}

aside {
  display: flex;
  flex-direction: column;
  background-color: #000000;
  width: calc(2rem + 32px);
  overflow: hidden;
  height: 100vh;
  padding: 1rem;

  .flex {
    flex: 1 1 0%;
  }

  h3,
  .button .text {
    opacity: 0;
    transition: opacity 0.3s ease-in-out;
  }

  h3 {
    color: var(--grey);
    font-size: 0.875rem;
    margin-bottom: 0.5rem;
    text-transform: uppercase;
  }

  .menu {
    margin: 0 -1rem;

    .button {
      display: flex;
      align-items: center;
      text-decoration: none;
      position: relative;
      padding: 1rem 1rem;

      .fa-solid {
        font-size: 1.5rem;
        color: var(--light);
        transition: 0.2s ease-in-out;
        margin-left: 2px;
      }

      &:hover {
        background-color: var(--dark-alt);
      }

      &::after {
        content: attr(data-tooltip);
        position: absolute;
        top: 50%;
        left: calc(100% + 10px);
        transform: translateY(-50%);
        padding: 0.5rem;
        background-color: #000;
        color: #fff;
        border-radius: 5px;
        opacity: 0;
        visibility: hidden;
        transition: opacity 0.2s ease-in-out;
      }

      &:hover::after {
        opacity: 1;
        visibility: visible;
      }
    }
  }

  h3,
  .button .text {
    opacity: 1;
  }

  .button {
    .material-symbols-outlined {
      margin-right: 1rem;
    }
  }

  .footer {
    opacity: 0;
  }
}

.menu {
  align-content: center;
  border-right: 3px solid transparent;
}

.menu .button.active {
  border-right-color: var(--primary);
}
.menu {
  align-content: center;
}

.menu .button {
  display: flex;
  align-items: center;
  text-decoration: none;
  padding: 1rem 1rem;
  position: relative;

  .fa-solid {
    font-size: 1.5rem;
    color: #000;
    transition: 0.2s ease-in-out;
    margin-left: 2px;
  }

  &:hover {
    background-color: var(--dark-alt);
  }

  &.active {
    border-right: 3px solid var(--primary);
  }
}
.menu .button::after {
  content: attr(data-tooltip);
  position: absolute;
  top: 50%;
  left: calc(100% + 10px);
  transform: translateY(-50%);
  padding: 0.5rem;
  background-color: #000;
  color: #fff;
  border-radius: 5px;
  opacity: 0;
  visibility: hidden;
  transition: opacity 0.2s ease-in-out;
}

.menu .button:hover::after {
  opacity: 1;
  visibility: visible;
}
.menu .button .fa-solid {
  font-size: 1.5rem;
  color: #000;
  transition: 0.2s ease-in-out;
  margin-left: 2px;
}
</style>
