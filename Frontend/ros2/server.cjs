const rclnodejs = require("rclnodejs");
const WebSocket = require("ws");

const server = new WebSocket.Server({ port: 3000 });
let maxSpeed = 0.5;
let maxTurn = 1.0;
let initializePosition = null;
let isPathInProgress = false;
let isMissionInProgress = false;
let firstRun = true;
let currentMission = null;
let continuePromiseResolve = null;

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

    server.on("connection", (ws) => {
      console.log("Client connected");

      ws.on("message", async (message) => {
        console.log("Received message from client:", message);
        const msg = JSON.parse(message);

        if (msg.type === "joystick_command") {
          const twist = new rclnodejs.geometry_msgs.msg.Twist();
          twist.linear.x = msg.linear * maxSpeed;
          twist.angular.z = msg.angular * maxTurn;
          publisher.publish(twist);
        } else if (msg.type === "set_speed") {
          maxSpeed = msg.maxSpeed;
          maxTurn = msg.maxTurn;
          console.log("Updated max speed settings:", { maxSpeed, maxTurn });

          // Update parameters for navigation
          const params = [
            {
              name: "controller_server.FollowPath.vx_max",
              value: maxSpeed,
              type: rclnodejs.ParameterType.PARAMETER_DOUBLE,
            },
            {
              name: "controller_server.FollowPath.wz_max",
              value: maxTurn,
              type: rclnodejs.ParameterType.PARAMETER_DOUBLE,
            },
          ];

          try {
            await node.setParameters(params);
            console.log("Parameters set successfully");
          } catch (error) {
            console.error("Error setting parameters:", error);
          }

          // Broadcast new speed settings to all connected clients
          server.clients.forEach((client) => {
            if (client.readyState === WebSocket.OPEN) {
              client.send(
                JSON.stringify({
                  type: "set_speed",
                  maxSpeed: maxSpeed,
                  maxTurn: maxTurn,
                })
              );
            }
          });
        } else if (msg.type === "send_goal" && !isMissionInProgress) {
          console.log("Received goal:", msg.x, msg.y, msg.z);
          isPathInProgress = true;
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
              const result = await goalHandle.getResult();
              console.log("Result received:", result);

              if (result) {
                console.log(`Reached with result:`, result.result);
                ws.send(
                  JSON.stringify({ type: "position_reached", position: "goal" })
                );
              } else {
                console.error(
                  `Failed to reach`,
                  result ? result.result : "Unknown error"
                );
              }
            } catch (error) {
              console.error("Error sending goal:", error);
              ws.send(
                JSON.stringify({ type: "goal_error", error: error.message })
              );
            } finally {
              isPathInProgress = false;
            }
          };
          sendGoal();
        } else if (msg.type === "initialize_robot") {
          initializePosition = { x: msg.x, y: msg.y, z: msg.z, w: msg.w };
          console.log("Robot initialized at position:", initializePosition);
        } else if (msg.type === "start_mission" && !isPathInProgress) {
          console.log("Starting mission:", msg);
          isMissionInProgress = true;
          currentMission = {
            waypoints: msg.waypoints.map((wp, index) => ({
              x: wp.x,
              y: wp.y,
              z: 0.0,
              w: wp.orientation,
              label:
                index === 0
                  ? "waypoint 1"
                  : index === 1
                  ? "waypoint 2"
                  : `waypoint ${index + 1}`,
            })),
          };

          let missionActive = true;

          const moveRobot = async (position, label) => {
            if (!missionActive) return;
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
            await waitForContinue(); // Wait for continue to move to waypoint 3
            if (currentMission.waypoints[2]) {
              await moveRobot(currentMission.waypoints[2], "waypoint 3");
            }
            missionLoop(); // Start the main mission loop
          };

          const missionLoop = async () => {
            while (missionActive) {
              for (let i = 0; i < currentMission.waypoints.length; i++) {
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
            }
            isMissionInProgress = false;
          };

          if (firstRun) {
            firstRunMission();
          } else {
            missionLoop();
          }

          const handleMessage = async (message) => {
            const msg = JSON.parse(message);
            if (msg.type === "continue_mission" && !firstRun) {
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
              ws.off("message", handleMessage); // Remove this handler
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
            ws.off("message", handleMessage); // Ensure handler is removed when socket closes
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

async function waitForContinue() {
  return new Promise((resolve) => {
    continuePromiseResolve = resolve;
  });
}
