<script setup>
import { ref, watch, onMounted } from 'vue'
import axios from 'axios'

// data
const ipAddress = ref("")
// Motor
const motor = ref(null)
const motordata = ref(0);
// WebSocket
var ws = null
const isWSConnected = ref(false)

// Motor値変化
watch(motordata, async(newValue, oldValue) => {
  motorFunc(newValue)
})

function motorFunc(value) {
  ws.send(String(value))
}

onMounted(() => {
  // データ取得
  axios
    .get(encodeURI("./API/get_data"))
    .then(response => {
      if (response.headers["content-type"] == "application/json") {
        ipAddress.value = response.data.ip_address
      }
    })
    .catch(error => {
      log.console("./API/get_data request error")
    })
  // WebSocket接続
  const url = "ws://" + window.location.host + "/ws"
  ws = new WebSocket(url)
  ws.onopen = (event) => {
    console.log("WS : サーバーConnect")
    isWSConnected.value = true
    motor(motordata.value)
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
              <h1>モーターコントローラ</h1>
          </div>
        </div>        
        <div class="row">
          <div class="col">
            <hr>
          </div>
        </div>
        <div class="row">
          <div class="col">
            <label for="motor" class="form-label zeromp">モーター ({{ motordata }})</label>
            <button class="btn btn-danger" style="margin-left: 1em; padding: 0.7em 1em 0.8em; --bs-btn-font-size: .75rem; --bs-btn-line-height: 0;" @click="motordata = 0">Stop</button>
            <button class="btn btn-danger" style="margin-left: 1em; padding: 0.7em 1em 0.8em; --bs-btn-font-size: .75rem; --bs-btn-line-height: 0;" @click="motorFunc('brake')">Brake</button>
            <input type="range" class="form-range" id="motor" ref="motor" :min="-100" :max="100" v-model="motordata">
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
