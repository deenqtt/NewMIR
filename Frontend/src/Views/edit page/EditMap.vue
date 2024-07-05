<template>
  <div class="container">
    <div class="d-flex align-items-center">
      <h3>Edit Map</h3>
      <button @click="confirmBack" class="btn btn-secondary ml-3">Back</button>
      <button @click="saveEdit" class="btn btn-success ml-3">Save</button>
    </div>
    <div class="card bg-light mt-3">
      <div
        class="card-header d-flex justify-content-between align-items-center"
      >
        <div>
          <button @click="setTool('draw')" class="btn btn-light">
            <i class="fa-solid fa-pencil"></i>
          </button>
          <input
            v-if="tool === 'draw'"
            type="range"
            v-model="penSize"
            min="1"
            max="10"
            step="1"
            class="ml-3"
          />
          <button @click="setTool('erase')" class="btn btn-light">
            <i class="fa-solid fa-eraser"></i>
          </button>
          <input
            v-if="tool === 'erase'"
            type="range"
            v-model="eraserSize"
            min="1"
            max="10"
            step="1"
            class="ml-3"
          />
        </div>
      </div>
      <div class="card-body">
        <canvas
          ref="canvas"
          @mousedown="startDrawing"
          @mouseup="stopDrawing"
          @mouseleave="stopDrawing"
          @mousemove="draw"
          style="border: 1px solid #ccc; width: 100%; height: auto"
        ></canvas>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted } from "vue";
import { useRoute, useRouter } from "vue-router";
import axios from "axios";
import Swal from "sweetalert2";

const route = useRoute();
const router = useRouter();
const canvas = ref(null);
const context = ref(null);
const isDrawing = ref(false);
const tool = ref("draw");
const penSize = ref(2); // Ukuran pensil
const eraserSize = ref(5); // Ukuran penghapus

const loadPGM = async () => {
  try {
    console.log(`Fetching map with ID: ${route.params.id}`);
    const response = await axios.get(
      `http://localhost:5258/maps/pgm/${route.params.id}`,
      {
        responseType: "arraybuffer",
      }
    );
    console.log("Response from server:", response);
    const pgmData = new Uint8Array(response.data);
    console.log("PGM data length:", pgmData.length);
    renderPGM(pgmData);
  } catch (error) {
    console.error("Failed to load PGM file:", error);
  }
};

const renderPGM = (pgmData) => {
  const headerEndIndex =
    pgmData.indexOf(10, pgmData.indexOf(10, pgmData.indexOf(10, 0) + 1) + 1) +
    1;
  const header = new TextDecoder().decode(pgmData.subarray(0, headerEndIndex));
  console.log("PGM header:", header);

  const [magicNumber, width, height, maxVal] = header.split(/\s+/);
  console.log("Parsed values:", { magicNumber, width, height, maxVal });

  const imageData = pgmData.subarray(headerEndIndex);
  const widthInt = parseInt(width);
  const heightInt = parseInt(height);
  const maxValInt = parseInt(maxVal);

  console.log("Parsed integers:", { widthInt, heightInt, maxValInt });

  if (!isFinite(widthInt) || !isFinite(heightInt) || !isFinite(maxValInt)) {
    console.error("Invalid PGM header values:", {
      widthInt,
      heightInt,
      maxValInt,
    });
    return;
  }

  const canvasElement = canvas.value;
  canvasElement.width = widthInt;
  canvasElement.height = heightInt;
  context.value = canvasElement.getContext("2d");
  const imageDataObject = context.value.createImageData(widthInt, heightInt);

  for (let i = 0; i < imageData.length; i++) {
    const pixelValue = (imageData[i] / maxValInt) * 255;
    imageDataObject.data[i * 4] = pixelValue; // Red
    imageDataObject.data[i * 4 + 1] = pixelValue; // Green
    imageDataObject.data[i * 4 + 2] = pixelValue; // Blue
    imageDataObject.data[i * 4 + 3] = 255; // Alpha
  }

  context.value.putImageData(imageDataObject, 0, 0);
};

const setTool = (selectedTool) => {
  tool.value = selectedTool;
};

const startDrawing = (event) => {
  isDrawing.value = true;
  draw(event); // Start drawing immediately
};

const stopDrawing = () => {
  isDrawing.value = false;
};

const draw = (event) => {
  if (!isDrawing.value) return;

  const rect = canvas.value.getBoundingClientRect();
  const scaleX = canvas.value.width / rect.width;
  const scaleY = canvas.value.height / rect.height;
  const x = (event.clientX - rect.left) * scaleX;
  const y = (event.clientY - rect.top) * scaleY;

  context.value.fillStyle = tool.value === "draw" ? "black" : "white";
  const size = tool.value === "draw" ? penSize.value : eraserSize.value;
  context.value.fillRect(x - size / 2, y - size / 2, size, size);
};

const confirmBack = () => {
  Swal.fire({
    title: "Are you sure?",
    text: "If you go back, all your changes will be lost.",
    icon: "warning",
    showCancelButton: true,
    confirmButtonColor: "#3085d6",
    cancelButtonColor: "#d33",
    confirmButtonText: "Yes, go back!",
  }).then((result) => {
    if (result.isConfirmed) {
      router.push("/"); // Ganti dengan rute yang sesuai
    }
  });
};

const saveEdit = async () => {
  const canvasElement = canvas.value;
  canvasElement.toBlob(async (blob) => {
    const formData = new FormData();
    formData.append("editedPgm", blob, "editedMap.pgm");

    try {
      const response = await axios.post(
        `http://localhost:5258/maps/update/${route.params.id}`,
        formData,
        {
          headers: {
            "Content-Type": "multipart/form-data",
          },
        }
      );
      if (response.status === 200) {
        Swal.fire("Success", "Map saved successfully", "success");
      } else {
        Swal.fire("Error", "Failed to save map", "error");
      }
    } catch (error) {
      console.error("Failed to save map:", error);
      Swal.fire("Error", "Failed to save map", "error");
    }
  }, "image/x-portable-graymap");
};

onMounted(() => {
  loadPGM();
});
</script>

<style scoped></style>
