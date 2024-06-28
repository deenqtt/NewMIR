const rclnodejs = require("rclnodejs");
const WebSocket = require("ws");

const server = new WebSocket.Server({ port: 3000 });

let isPathInProgress = false;

rclnodejs
  .init()
  .then(() => {
    const node = new rclnodejs.Node("my_node");
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

        if (msg.type === "send_goal" && !isPathInProgress) {
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
              const result = await Promise.race([
                goalHandle.getResult(),
                new Promise(
                  (_, reject) =>
                    setTimeout(() => reject(new Error("Goal timeout")), 90000) // Timeout diperpanjang menjadi 90 detik
                ),
              ]);
              console.log("Result received:", result);

              if (result && result.result) {
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
