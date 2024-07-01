<template>
  <div class="container mx-auto">
    <div class="mt-10">
      <div class="max-width-5xl mx-auto">
        <div class="flex flex-col">
          <div class="flex justify-end mb-5">
            <input
              v-model="searchQuery"
              type="text"
              placeholder="Search..."
              class="shadow border block border-gray-300 py-2 px-4 rounded-md text-sm font-medium"
            />
            <router-link
              :to="{ name: 'barang.new' }"
              class="shadow bg-blue-700 border block border-gray-300 text-white py-2 px-4 rounded-md text-sm font-medium ml-2"
            >
              Add Item
            </router-link>
          </div>
          <div class="-my-2 overflow-x-auto sm:-mx-6 lg:-mx-8">
            <div
              class="py-2 align-middle inline-block min-w-full sm:px-6 lg:px-8"
            >
              <div
                class="shadow overflow-hidden border-b border-gray-200 sm:rounded-lg"
              >
                <table class="min-w-full divide-y divide-gray-200">
                  <thead class="bg-gray-50">
                    <tr>
                      <th
                        scope="col"
                        class="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider"
                      >
                        Nama
                      </th>
                      <th
                        scope="col"
                        class="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider"
                      >
                        Berat
                      </th>
                      <th
                        scope="col"
                        class="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider"
                      >
                        Stock
                      </th>
                      <th scope="col" class="relative px-6 py-3">
                        <span class="sr-only">Edit</span>
                      </th>
                    </tr>
                  </thead>
                  <tbody class="bg-white divide-y divide-gray-200">
                    <tr
                      v-for="(barang, index) in filteredBarangs"
                      :key="barang.id"
                    >
                      <td class="px-6 py-4 whitespace-nowrap">
                        <div class="text-sm text-gray-900">
                          {{ barang.nama }}
                        </div>
                        <div class="text-sm text-gray-500">
                          {{ barang.category }}
                        </div>
                      </td>
                      <td class="px-6 py-4 whitespace-nowrap">
                        <div class="text-sm text-gray-900">
                          {{ barang.berat }} Kg
                        </div>
                      </td>
                      <td class="px-6 py-4 whitespace-nowrap">
                        <div class="text-sm text-gray-900">
                          {{ barang.stok }}
                        </div>
                      </td>
                      <td
                        class="px-6 py-4 whitespace-nowrap text-right text-sm font-medium"
                      >
                        <a
                          href="#"
                          @click.prevent="_toEditBarang(index)"
                          class="text-indigo-600 hover:text-indigo-900"
                          >Edit</a
                        >
                        |
                        <a
                          href="#"
                          @click.prevent="_deleteBarang(barang.id)"
                          class="text-indigo-600 hover:text-indigo-900"
                          >Hapus</a
                        >
                      </td>
                    </tr>
                  </tbody>
                </table>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted } from "vue";
import { useStore } from "vuex";
import { useRouter } from "vue-router";

const store = useStore();
const router = useRouter();
const barangs = computed(() => store.state.barang.barangs);
const searchQuery = ref("");

const filteredBarangs = computed(() => {
  if (!searchQuery.value) return barangs.value;
  return barangs.value.filter((barang) => {
    const query = searchQuery.value.toLowerCase();
    return (
      barang.nama.toLowerCase().includes(query) ||
      barang.category.toLowerCase().includes(query) ||
      barang.berat.toString().toLowerCase().includes(query) ||
      barang.stok.toString().toLowerCase().includes(query)
    );
  });
});

const _getAllBarang = async () => {
  try {
    await store.dispatch("barang/getAllBarang");
  } catch (e) {
    console.error(e);
  }
};

const _toEditBarang = (index) => {
  store.commit("barang/_assign_barang_form", barangs.value[index]);
  router.push({ name: "barang.update" });
};

const _deleteBarang = async (id) => {
  if (!confirm("Barang ini ingin dihapus?")) {
    return false;
  }
  try {
    await store.dispatch("barang/deleteBarang", id);
    _getAllBarang();
  } catch (e) {
    console.error(e);
  }
};

onMounted(() => {
  _getAllBarang();
});
</script>

<style>
.button {
  width: 90px;
}
</style>
