var NAV2 = NAV2 || {
  REVISION: "0.3.0",
};

NAV2.ImageMapClient = function (options) {
  options = options || {};
  this.ros = options.ros;
  var topic = options.topic || "/map";
  var image = options.image;
  this.rootObject = options.rootObject || new createjs.Container();
  this.viewer = options.viewer;

  var client = new ROS2D.ImageMapClient({
    ros: this.ros,
    rootObject: this.rootObject,
    topic: topic,
    image: image,
  });

  client.on("error", function (error) {
    console.error("Error in ImageMapClient: ", error);
  });
};

NAV2.OccupancyGridClient = function (options) {
  options = options || {};
  this.ros = options.ros;
  var topic = options.topic || "/map";
  this.rootObject = options.rootObject || new createjs.Container();
  this.viewer = options.viewer;

  var client = new ROS2D.OccupancyGridClient({
    ros: this.ros,
    rootObject: this.rootObject,
    topic: topic,
  });

  client.on("error", function (error) {
    console.error("Error in OccupancyGridClient: ", error);
  });
};
