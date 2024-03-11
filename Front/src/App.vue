<script setup>
import { ref, watch, onMounted } from 'vue'
import axios from 'axios'

// data
const ipAddress = ref("")
// LED
const leds = ref([false, false, false, false])
// Servo
const servo1 = ref(null)
const servo1data = ref(0)
const servotrim1data = ref(0)
// WebSocket
var ws = null
const isWSConnected = ref(false)

// LEDボタン変化
watch(() => [...leds.value], async (newLeds, oldLeds) => {
  console.log(newLeds)
  var json = {
    led: newLeds
  }
  axios
    .post(encodeURI("./API/set_led"), json, { headers: { 'Content-Type': 'application/json'}})
})

// Servo値変化
watch(servo1data, async(newValue, oldValue) => {
  servo((Number(newValue) + Number(servotrim1data.value)).toString())
})
watch(servotrim1data, async(newValue, oldValue) => {
  servo((Number(servo1data.value) + Number(newValue)).toString())
  if (newValue != oldValue) {
    var json = {
      servo_trim: Number(newValue)
    }
    axios
    .post(encodeURI("./API/set_data"), json, { headers: { 'Content-Type': 'application/json'}})
  }
})

function servo(value) {
  ws.send(value)
}

onMounted(() => {
  // データ取得
  axios
    .get(encodeURI("./API/get_data"))
    .then(response => {
      if (response.headers["content-type"] == "application/json") {
        ipAddress.value = response.data.ip_address
        servotrim1data.value = response.data.servo_trim
      }
    })
    .catch(error => {
      log.console("./API/get_data request error")
    })
  // LED状態取得
  axios
    .get(encodeURI("./API/get_led"))
    .then(response => {
      if (response.headers["content-type"] == "application/json") {
        leds.value = response.data.led
      }
    })
    .catch(error => {
      log.console("./API/get_led request error")
    })
  // WebSocket接続
  const url = "ws://" + window.location.host + "/ws"
  ws = new WebSocket(url)
  ws.onopen = (event) => {
    console.log("WS : サーバーConnect")
    isWSConnected.value = true
    servo((Number(servo1data.value) + Number(servotrim1data.value)).toString())
  }
  ws.onclose = (event) => {
    console.log("WS : サーバーDisconnect")
    isWSConnected.value = false
  }
  ws.onmessage = (event) => {
    console.log("WS : サーバーから受信 : " + event.data)
  }
  ws.onerror = (event) => {
    console.error("WS : エラー", event)
  }
})

function save() {
  axios
    .post(encodeURI("./API/save"), "", { headers: { 'Content-Type': 'application/json'}})
}

</script>

<template>
  <main>
    <div style="position: absolute; top: 10px; left: 10px; right: 0;">
      {{ ipAddress }}<br/>
      <div class="container text-center">
        <div class="row">
          <div class="col">
            <h1>LEDコントローラ</h1>
          </div>
        </div>
        <div class="row">
          <div class="col">
            <hr>
          </div>
        </div>
        <div class="row" style="margin-top: 5px;">
          <div class="col">
            <span v-for="n in 4" :key="n" style="margin-left: 5px;">
              <input type="checkbox" class="btn-check" :id="`led${n}`" v-model="leds[n-1]">
              <label class="btn btn-outline-primary btn-lg" :for="`led${n}`">LED{{ n }}</label>
            </span>
          </div>
        </div>
        <div class="row" style="height: 1.5em;">
        </div>
        <div class="row">
          <div class="col">
              <h1>サーボコントローラ</h1>
          </div>
        </div>        
        <div class="row">
          <div class="col">
            <hr>
          </div>
        </div>
        <div class="row">
          <div class="col">
            <label for="servo1" class="form-label zeromp">サーボ1 ({{ servo1data }})</label><button class="btn btn-danger" style="margin-left: 1em; padding: 0.7em 1em 0.8em; --bs-btn-font-size: .75rem; --bs-btn-line-height: 0;" @click="servo1data = 0">Reset</button>
            <input type="range" class="form-range" id="servo1" ref="servo1" :min="-80" :max="80" v-model="servo1data">
          </div>
        </div>
        <div class="row">
          <div class="col">
            <label for="servo-trim1" class="form-label zeromp">トリム1 ({{ servotrim1data }})</label><button class="btn btn-danger" style="margin-left: 1em; padding: 0.7em 1em 0.8em; --bs-btn-font-size: .75rem; --bs-btn-line-height: 0;" @click="servotrim1data = 0">Reset</button>
            <input type="range" class="form-range" id="servo-trim1" ref="servo-trim1" :min="-10" :max="10" v-model="servotrim1data">
          </div>
        </div>
        <dev class="row">
          <div class="col">
            <button class="btn btn-success" style="margin-top: 5em;" @click="save">Save</button>
          </div>
        </dev>
      </div>
    </div>
  </main>
</template>

<style scoped>
h1,hr {
  margin: 1px;
  padding: 1px;
}

.zeromp {
  margin: 0;
  padding: 0;
}

button {
  padding: 1em 2em 1em;
}

div {
  color: white;
}
</style>
