const rclnodejs = require("rclnodejs");
const WebSocket = require("ws");
const { exec } = require("child_process");
const server = new WebSocket.Server({ port: 3000 });

let initializePosition = null;
let dockingPosition = null;
let isPathInProgress = false;
let isMissionInProgress = false;
let firstRun = true;
let currentMission = null;
let currentWaypointIndex = 0;
let continuePromiseResolve = null;
let maxSpeed = 0.5;
let maxTurn = 1.0;
let batteryLevel = 100.0;
let globalWs = null;
let pendingGoal = null;
let pendingMission = null;
let resumeAfterDocking = false;
let isPausedDueToLowBattery = false;
let currentPosition = { x: 0, y: 0 };

const dockingPositionTolerance = 0.1; // Tolerance in meters for docking position accuracy

function isAtDockingPosition(currentPosition, dockingPosition) {
  const distance = Math.sqrt(
    Math.pow(currentPosition.x - dockingPosition.posX, 2) +
      Math.pow(currentPosition.y - dockingPosition.posY, 2)
  );
  return distance < dockingPositionTolerance;
}

rclnodejs
  .init()
  .then(() => {
    const node = new rclnodejs.Node("my_node");
    const publisher = node.createPublisher(
      "geometry_msgs/msg/Twist",
      "cmd_vel"
    );
    const actionClient = new rclnodejs.ActionClient(
      node,
      "nav2_msgs/action/NavigateToPose",
      "navigate_to_pose"
    );

    const batterySubscriber = node.createSubscription(
      "sensor_msgs/msg/BatteryState",
      "battery_state",
      (msg) => {
        batteryLevel = msg.percentage * 100;
        console.log(`Battery level: ${batteryLevel}%`);

        if (globalWs) {
          globalWs.send(
            JSON.stringify({
              type: "battery_update",
              level: batteryLevel,
            })
          );
        }

        if (batteryLevel < 90 && !isPausedDueToLowBattery) {
          console.log(
            "Battery low, pausing current task and returning to docking station."
          );
          saveCurrentTaskStatus();
          isPausedDueToLowBattery = true;
          stopMovementAndReturnToDockingPosition(globalWs, node, actionClient);
        } else if (batteryLevel >= 95 && resumeAfterDocking) {
          if (isAtDockingPosition(currentPosition, dockingPosition)) {
            console.log("Battery recharged, resuming paused task.");
            resumeAfterDocking = false;
            isPausedDueToLowBattery = false;
            if (pendingGoal) {
              resumeGoal(globalWs, node, actionClient);
            } else if (pendingMission) {
              resumeMission(globalWs, node, actionClient);
            }
          } else {
            console.log(
              "Battery is high, but not at docking station. Waiting to dock."
            );
          }
        }
      }
    );

    const positionSubscriber = node.createSubscription(
      "geometry_msgs/msg/PoseWithCovarianceStamped",
      "/amcl_pose",
      (msg) => {
        currentPosition = {
          x: msg.pose.pose.position.x,
          y: msg.pose.pose.position.y,
        };
      }
    );

    server.on("connection", (ws) => {
      console.log("Client connected");
      globalWs = ws;

      ws.on("message", async (message) => {
        console.log("Received message from client:", message);
        const msg = JSON.parse(message);

        if (isPausedDueToLowBattery) {
          console.log("Ignoring commands during low battery state.");
          return;
        }

        if (msg.type === "set_speed") {
          maxSpeed = parseFloat(msg.maxSpeed);
          maxTurn = parseFloat(msg.maxTurn);
          console.log("Updated max speed settings:", { maxSpeed, maxTurn });

          await updateParams(maxSpeed, maxTurn, ws);
        } else if (msg.type === "send_goal" && !isMissionInProgress) {
          saveCurrentTaskStatus();
          console.log("Received goal:", msg.x, msg.y, msg.z);
          isPathInProgress = true;
          pendingGoal = { x: msg.x, y: msg.y, z: msg.z, w: msg.w };
          const sendGoal = async () => {
            const goalTimestamp = node.getClock().now();
            console.log(`Sending goal at timestamp: ${goalTimestamp}`);
            const goal = {
              pose: {
                header: {
                  frame_id: "map",
                  stamp: goalTimestamp,
                },
                pose: {
                  position: { x: msg.x, y: msg.y, z: 0.0 },
                  orientation: { x: 0.0, y: 0.0, z: msg.z, w: msg.w },
                },
              },
            };
            try {
              await actionClient.waitForServer();
              console.log("Action server is available, sending goal.");
              const goalHandle = await actionClient.sendGoal(goal);
              console.log("Goal sent, waiting for result...");
              await goalHandle.getResult();
            } catch (error) {
              console.error("Error sending goal:", error);
              ws.send(
                JSON.stringify({ type: "goal_error", error: error.message })
              );
            } finally {
              isPathInProgress = false;
              pendingGoal = null;
            }
          };
          sendGoal();
        } else if (msg.type === "initialize_robot") {
          initializePosition = { x: msg.x, y: msg.y, z: msg.z, w: msg.w };
          console.log("Robot initialized at position:", initializePosition);
        } else if (msg.type === "update_docking_position") {
          dockingPosition = msg.dockingPosition;
          console.log("Docking position updated:", dockingPosition);
        } else if (msg.type === "start_mission" && !isPathInProgress) {
          saveCurrentTaskStatus();
          console.log("Starting mission:", msg);
          isMissionInProgress = true;
          currentMission = {
            waypoints: msg.waypoints.map((wp, index) => ({
              x: wp.x,
              y: wp.y,
              z: 0.0,
              w: wp.orientation,
              label: `waypoint ${index + 1}`,
            })),
          };
          pendingMission = currentMission;
          currentWaypointIndex = 0;

          let missionActive = true;

          const moveRobot = async (position, label) => {
            if (!missionActive) return;
            if (batteryLevel < 80) {
              saveCurrentTaskStatus();
              await stopMovementAndReturnToDockingPosition(
                ws,
                node,
                actionClient
              );
              return;
            }
            const startTime = Date.now();
            console.log(`Moving to ${label}:`, position);
            const goalTimestamp = node.getClock().now();
            console.log(
              `Sending goal to ${label} at timestamp: ${goalTimestamp}`
            );
            const goal = {
              pose: {
                header: {
                  frame_id: "map",
                  stamp: goalTimestamp,
                },
                pose: {
                  position: { x: position.x, y: position.y, z: 0.0 },
                  orientation: {
                    x: 0.0,
                    y: 0.0,
                    z: position.z,
                    w: position.w,
                  },
                },
              },
            };

            try {
              await actionClient.waitForServer();
              console.log(
                `Action server is available, sending goal to ${label}.`
              );
              const goalHandle = await actionClient.sendGoal(goal);
              console.log(`Goal sent to ${label}, waiting for result...`);
              const result = await Promise.race([
                goalHandle.getResult(),
                new Promise((_, reject) =>
                  setTimeout(() => reject(new Error("Goal timeout")), 30000)
                ),
              ]);
              const endTime = Date.now();
              console.log(`Result received for ${label}:`, result);
              console.log(
                `Time taken to reach ${label}: ${endTime - startTime} ms`
              );

              if (result) {
                console.log(`Reached ${label} with result:`, result.result);
                ws.send(
                  JSON.stringify({ type: "position_reached", position: label })
                );
                currentWaypointIndex++;
              } else {
                console.error(
                  `Failed to reach ${label}`,
                  result ? result.result : "Unknown error"
                );
              }
            } catch (error) {
              console.error(`Error sending goal to ${label}:`, error);
            }
          };

          const firstRunMission = async () => {
            console.log("First run: Moving to start position");
            await moveRobot(currentMission.waypoints[0], "waypoint 1");
            await moveRobot(currentMission.waypoints[1], "waypoint 2");
            firstRun = false;
            console.log(
              'First run completed. Waiting for "continue_mission" to proceed...'
            );
            await waitForContinue();
            if (currentMission.waypoints[2]) {
              await moveRobot(currentMission.waypoints[2], "waypoint 3");
            }
            missionLoop();
          };

          const missionLoop = async () => {
            while (missionActive) {
              for (
                let i = currentWaypointIndex;
                i < currentMission.waypoints.length;
                i++
              ) {
                console.log(
                  `Waiting for "continue_mission" to proceed to waypoint ${
                    i + 1
                  }`
                );
                await waitForContinue();
                console.log(
                  `Moving to waypoint ${i + 1}:`,
                  currentMission.waypoints[i]
                );
                await moveRobot(
                  currentMission.waypoints[i],
                  `waypoint ${i + 1}`
                );
              }
              currentWaypointIndex = 0; // Loop back to the start after the last waypoint
            }
            isMissionInProgress = false;
            pendingMission = null;
          };

          if (firstRun) {
            firstRunMission();
          } else {
            missionLoop();
          }

          const handleMessage = async (message) => {
            const msg = JSON.parse(message);
            if (msg.type === "continue_mission") {
              console.log("Continuing mission");
              if (continuePromiseResolve) {
                continuePromiseResolve();
                continuePromiseResolve = null;
              }
            } else if (msg.type === "stop_mission") {
              console.log(
                "Stopping mission and returning to initialize position"
              );
              missionActive = false;
              isMissionInProgress = false;
              firstRun = true;
              ws.off("message", handleMessage);
              const returnToInitializePosition = async () => {
                const goalTimestamp = node.getClock().now();
                console.log(
                  `Sending goal to initialize position at timestamp: ${goalTimestamp}`
                );
                const goal = {
                  pose: {
                    header: {
                      frame_id: "map",
                      stamp: goalTimestamp,
                    },
                    pose: {
                      position: {
                        x: initializePosition.x,
                        y: initializePosition.y,
                        z: 0.0,
                      },
                      orientation: {
                        x: 0.0,
                        y: 0.0,
                        z: initializePosition.z,
                        w: initializePosition.w,
                      },
                    },
                  },
                };

                try {
                  await actionClient.waitForServer();
                  console.log("Action server is available, sending goal.");
                  const goalHandle = await actionClient.sendGoal(goal);
                  console.log("Goal sent, waiting for result...");
                  const result = await goalHandle.getResult();
                  console.log("Result received:", result);

                  if (result) {
                    console.log(
                      "Returned to initialize position with result:",
                      result.result
                    );
                  } else {
                    console.error(
                      "Failed to return to initialize position",
                      result ? result.result : "Unknown error"
                    );
                  }
                } catch (error) {
                  console.error(
                    "Error sending goal to initialize position:",
                    error
                  );
                }
              };

              returnToInitializePosition();
            }
          };

          ws.on("message", handleMessage);
          ws.on("close", () => {
            missionActive = false;
            ws.off("message", handleMessage);
          });
        }
      });

      ws.on("close", () => {
        console.log("Client disconnected");
      });
    });

    node.spin();
    process.on("SIGINT", async () => {
      await rclnodejs.shutdown();
      console.log("ROS2 node shut down.");
      process.exit(0);
    });
  })
  .catch((err) => {
    console.error(err);
  });

async function updateParams(maxSpeed, maxTurn, ws) {
  const commands = [
    `ros2 param set /controller_server FollowPath.max_vel_x ${maxSpeed.toFixed(
      2
    )}`,
    `ros2 param set /controller_server FollowPath.max_vel_theta ${maxTurn.toFixed(
      2
    )}`,
    `ros2 param set /velocity_smoother max_velocity "[${maxSpeed.toFixed(
      2
    )}, 0.0, ${maxTurn.toFixed(2)}]"`,
  ];

  for (const cmd of commands) {
    exec(cmd, (error, stdout, stderr) => {
      if (error) {
        console.error(`Error updating parameter: ${error.message}`);
        ws.send(JSON.stringify({ type: "update_error", error: error.message }));
        return;
      }
      if (stderr) {
        console.error(`stderr: ${stderr}`);
      }
      console.log(`stdout: ${stdout}`);
    });
  }

  ws.send(JSON.stringify({ type: "params_updated", maxSpeed, maxTurn }));
}

async function waitForContinue() {
  return new Promise((resolve) => {
    continuePromiseResolve = resolve;
  });
}

function saveCurrentTaskStatus() {
  if (isPathInProgress) {
    pendingGoal = { ...pendingGoal };
  }
  if (isMissionInProgress) {
    pendingMission = { ...currentMission };
    currentWaypointIndex = currentWaypointIndex;
  }
  resumeAfterDocking = true;
}

async function stopMovementAndReturnToDockingPosition(ws, node, actionClient) {
  console.log(
    "Battery low. Stopping movement and returning to docking position."
  );
  isMissionInProgress = false;
  isPathInProgress = false;

  if (continuePromiseResolve) {
    continuePromiseResolve();
    continuePromiseResolve = null;
  }

  if (ws) {
    ws.send(
      JSON.stringify({
        type: "low_battery",
        message: "Battery level below 90%, stopping movement.",
      })
    );
  }

  if (!dockingPosition) {
    console.error("Docking position is not set.");
    return;
  }

  const goalTimestamp = node.getClock().now();
  const goal = {
    pose: {
      header: {
        frame_id: "map",
        stamp: goalTimestamp,
      },
      pose: {
        position: {
          x: dockingPosition.posX,
          y: dockingPosition.posY,
          z: 0.0,
        },
        orientation: {
          x: 0.0,
          y: 0.0,
          z: dockingPosition.orientation,
          w: 1.0,
        },
      },
    },
  };

  try {
    await actionClient.waitForServer();
    console.log("Action server is available, sending goal.");
    const goalHandle = await actionClient.sendGoal(goal);
    console.log("Goal sent, waiting for result...");
    const result = await goalHandle.getResult();
    console.log("Result received:", result);

    if (result) {
      console.log("Returned to docking position with result:", result.result);
    } else {
      console.error(
        "Failed to return to docking position",
        result ? result.result : "Unknown error"
      );
    }
  } catch (error) {
    console.error("Error sending goal to docking position:", error);
  }
}

async function resumeGoal(ws, node, actionClient) {
  if (!pendingGoal) return;
  console.log("Resuming previous goal:", pendingGoal);
  isPathInProgress = true;
  const goalTimestamp = node.getClock().now();
  const goal = {
    pose: {
      header: {
        frame_id: "map",
        stamp: goalTimestamp,
      },
      pose: {
        position: { x: pendingGoal.x, y: pendingGoal.y, z: 0.0 },
        orientation: { x: 0.0, y: 0.0, z: pendingGoal.z, w: pendingGoal.w },
      },
    },
  };
  try {
    await actionClient.waitForServer();
    const goalHandle = await actionClient.sendGoal(goal);
    const result = await goalHandle.getResult();
    console.log("Result received:", result);
    ws.send(JSON.stringify({ type: "position_reached", position: "goal" }));
  } catch (error) {
    console.error("Error resuming goal:", error);
    ws.send(JSON.stringify({ type: "goal_error", error: error.message }));
  } finally {
    isPathInProgress = false;
    pendingGoal = null;
  }
}

async function resumeMission(ws, node, actionClient) {
  if (!pendingMission) return;
  console.log("Resuming previous mission:", pendingMission);
  currentMission = pendingMission;
  pendingMission = null;
  let missionActive = true;
  const moveRobot = async (position, label) => {
    if (!missionActive) return;
    if (batteryLevel < 80) {
      saveCurrentTaskStatus();
      await stopMovementAndReturnToDockingPosition(ws, node, actionClient);
      return;
    }
    const startTime = Date.now();
    const goalTimestamp = node.getClock().now();
    const goal = {
      pose: {
        header: {
          frame_id: "map",
          stamp: goalTimestamp,
        },
        pose: {
          position: { x: position.x, y: position.y, z: 0.0 },
          orientation: { x: 0.0, y: 0.0, z: position.z, w: position.w },
        },
      },
    };
    try {
      await actionClient.waitForServer();
      const goalHandle = await actionClient.sendGoal(goal);
      const result = await goalHandle.getResult();
      const endTime = Date.now();
      console.log(`Result received for ${label}:`, result);
      console.log(`Time taken to reach ${label}: ${endTime - startTime} ms`);
      ws.send(JSON.stringify({ type: "position_reached", position: label }));
      currentWaypointIndex++;
      if (currentWaypointIndex >= currentMission.waypoints.length) {
        currentWaypointIndex = 0; // Reset to the first waypoint after reaching the last one
      }
    } catch (error) {
      console.error(`Error sending goal to ${label}:`, error);
    }
  };

  const missionLoop = async () => {
    while (missionActive) {
      for (
        let i = currentWaypointIndex;
        i < currentMission.waypoints.length;
        i++
      ) {
        console.log(
          `Waiting for "continue_mission" to proceed to waypoint ${i + 1}`
        );
        await waitForContinue();
        console.log(
          `Moving to waypoint ${i + 1}:`,
          currentMission.waypoints[i]
        );
        await moveRobot(currentMission.waypoints[i], `waypoint ${i + 1}`);
      }
      currentWaypointIndex = 0; // Loop back to the start or next mission
    }
    isMissionInProgress = false;
  };
  missionLoop();
}

async function stopMissionAndReturnToInitializePosition(
  ws,
  node,
  actionClient
) {
  console.log("Mission stopped. Returning to initialize position.");
  isMissionInProgress = false;
  isPathInProgress = false;

  if (continuePromiseResolve) {
    continuePromiseResolve();
    continuePromiseResolve = null;
  }

  if (ws) {
    ws.send(
      JSON.stringify({
        type: "mission_stopped",
        message: "Mission has been stopped, returning to initialize position.",
      })
    );
  }

  if (!initializePosition) {
    console.error("Initialize position is not set.");
    return;
  }

  const goalTimestamp = node.getClock().now();
  const goal = {
    pose: {
      header: {
        frame_id: "map",
        stamp: goalTimestamp,
      },
      pose: {
        position: {
          x: initializePosition.x,
          y: initializePosition.y,
          z: 0.0,
        },
        orientation: {
          x: 0.0,
          y: 0.0,
          z: initializePosition.z,
          w: initializePosition.w,
        },
      },
    },
  };

  try {
    await actionClient.waitForServer();
    console.log("Action server is available, sending goal.");
    const goalHandle = await actionClient.sendGoal(goal);
    console.log("Goal sent, waiting for result...");
    const result = await goalHandle.getResult();
    console.log("Result received:", result);

    if (result) {
      console.log(
        "Returned to initialize position with result:",
        result.result
      );
    } else {
      console.error(
        "Failed to return to initialize position",
        result ? result.result : "Unknown error"
      );
    }
  } catch (error) {
    console.error("Error sending goal to initialize position:", error);
  }
}
