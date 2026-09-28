<script setup>
import { ref } from 'vue'
import { $tp } from '../../../platform-i18n'
import PlatformLayout from '../../../PlatformLayout.vue'
import Checkbox from '../../../Checkbox.vue'

const props = defineProps([
  'platform',
  'config',
])
const config = ref(props.config)
</script>

<template>
  <!--max_bitrate-->
  <div class="mb-3">
    <label for="max_bitrate" class="form-label">{{ $t("config.max_bitrate") }}</label>
    <input type="number" class="form-control" id="max_bitrate" placeholder="0" v-model="config.max_bitrate" />
    <div class="form-text">{{ $t("config.max_bitrate_desc") }}</div>
  </div>

  <!--minimum_fps_target-->
  <div class="mb-3">
    <label for="minimum_fps_target" class="form-label">{{ $t("config.minimum_fps_target") }}</label>
    <input type="number" min="0" max="1000" class="form-control" id="minimum_fps_target" placeholder="0" v-model="config.minimum_fps_target" />
    <div class="form-text">{{ $t("config.minimum_fps_target_desc") }}</div>
  </div>

  <!--dynamic_framerate-->
  <div class="mb-3">
    <Checkbox id="dynamic_framerate"
      locale-prefix="config"
      v-model="config.dynamic_framerate"
      default="true"
    ></Checkbox>
  </div>

  <!--dynamic_resolution-->
  <div class="mb-3">
    <Checkbox id="dynamic_resolution"
      locale-prefix="config"
      v-model="config.dynamic_resolution"
      default="true"
    ></Checkbox>
  </div>

  <!--dynamic_bitrate-->
  <div class="mb-3">
    <Checkbox id="dynamic_bitrate"
      locale-prefix="config"
      v-model="config.dynamic_bitrate"
      default="true"
    ></Checkbox>
  </div>

  <!--dynamic_bitrate_min-->
  <div class="mb-3">
    <label for="dynamic_bitrate_min" class="form-label">{{ $t("config.dynamic_bitrate_min") }}</label>
    <input type="number" min="0" class="form-control" id="dynamic_bitrate_min" placeholder="0" v-model="config.dynamic_bitrate_min" />
    <div class="form-text">{{ $t("config.dynamic_bitrate_min_desc") }}</div>
  </div>
</template>

<style scoped>
.ms-item {
  background-color: var(--bs-dark-bg-subtle);
  font-size: 12px;
  font-weight: bold;
}
</style>
