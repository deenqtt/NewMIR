const express = require("express");
const bodyParser = require("body-parser");
const { exec } = require("child_process");

const app = express();
app.use(bodyParser.json());

let currentLinearSpeed = 0.5;
let currentAngularSpeed = 1.0;

app.post("/update_speed", (req, res) => {
  const { linear_speed, angular_speed } = req.body;
  currentLinearSpeed = linear_speed;
  currentAngularSpeed = angular_speed;
  res.sendStatus(200);
});

app.get("/cmd_vel", (req, res) => {
  const cmdVel = {
    linear: {
      x: currentLinearSpeed,
      y: 0.0,
      z: 0.0,
    },
    angular: {
      x: 0.0,
      y: 0.0,
      z: currentAngularSpeed,
    },
  };
  res.json(cmdVel);
});

const PORT = process.env.PORT || 5000;
app.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}`);
});
