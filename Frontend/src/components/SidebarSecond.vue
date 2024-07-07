<template>
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
</template>

<script setup>
import { ref, reactive, toRefs, computed, watch, onMounted } from "vue";
import { useRouter } from "vue-router";

const router = useRouter();

const state = reactive({
  is_expanded: localStorage.getItem("is_expanded") === "true",
  pageTitle: "",
  subMenu: [],
});

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

const isSubMenuActive = (item) => {
  const routeName = router.currentRoute.value.name;
  const isActive = routeName && routeName.toLowerCase() === item.toLowerCase();
  if (isActive) {
    activeSubMenu.value = item;
  }
  return isActive;
};

watch(
  () => router.currentRoute.value.name,
  () => {
    updatePageTitle();
  }
);

onMounted(() => {
  updatePageTitle();
});
</script>

<style scoped>
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
</style>
