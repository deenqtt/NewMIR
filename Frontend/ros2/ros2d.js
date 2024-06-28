var ROS2D = ROS2D || { REVISION: "0.10.0" };

(createjs.Stage.prototype.globalToRos = function (t, e) {
  t = (t - this.x) / this.scaleX;
  e = (this.y - e) / this.scaleY;
  return new ROSLIB.Vector3({ x: t, y: e });
}),
  (createjs.Stage.prototype.rosToGlobal = function (t) {
    return { x: t.x * this.scaleX + this.x, y: t.y * this.scaleY + this.y };
  }),
  (createjs.Stage.prototype.rosQuaternionToGlobalTheta = function (t) {
    var e = t.w,
      i = t.x,
      s = t.y,
      t = t.z;
    return (
      (180 * -Math.atan2(2 * (e * t + i * s), 1 - 2 * (s * s + t * t))) /
      Math.PI
    );
  });

ROS2D.ImageMap = function (t) {
  var e = (t = t || {}).message,
    t = t.image;
  this.pose = new ROSLIB.Pose({
    position: e.info.origin.position,
    orientation: e.info.origin.orientation,
  });
  this.width = e.info.width;
  this.height = e.info.height;
  createjs.Bitmap.call(this, t);
  this.y = -this.height * e.info.resolution;
  this.scaleX = e.info.resolution;
  this.scaleY = e.info.resolution;
  this.width *= this.scaleX;
  this.height *= this.scaleY;
  this.x += this.pose.position.x;
  this.y -= this.pose.position.y;
};
ROS2D.ImageMap.prototype.__proto__ = createjs.Bitmap.prototype;
ROS2D.ImageMapClient = function (t) {
  var e = this,
    i = (t = t || {}).ros,
    s = t.topic || "/map",
    o =
      ((this.image = t.image),
      (this.rootObject = t.rootObject || new createjs.Container()),
      (this.currentImage = new createjs.Shape()),
      new ROSLIB.Topic({
        ros: i,
        name: s,
        messageType: "nav_msgs/OccupancyGrid",
      }));
  o.subscribe(function (t) {
    o.unsubscribe(),
      (e.currentImage = new ROS2D.ImageMap({ message: t, image: e.image })),
      e.rootObject.addChild(e.currentImage),
      e.rootObject.addChild(new ROS2D.Grid({ size: 1 })),
      e.emit("change");
  });
};
ROS2D.ImageMapClient.prototype.__proto__ = EventEmitter2.prototype;

// Define NavigationArrow
ROS2D.NavigationArrow = function (options) {
  options = options || {};
  var size = options.size || 1;
  var strokeSize = options.strokeSize || 2;
  var strokeColor = options.strokeColor || createjs.Graphics.getRGB(0, 0, 0);
  var fillColor = options.fillColor || createjs.Graphics.getRGB(255, 0, 0);
  var pulse = options.pulse || false;

  // draw the arrow
  var graphics = new createjs.Graphics();
  // line width
  graphics.setStrokeStyle(strokeSize);
  graphics.moveTo(-size / 2.0, -size / 2.0);
  graphics.beginStroke(strokeColor);
  graphics.beginFill(fillColor);
  graphics.lineTo(size, 0);
  graphics.lineTo(-size / 2.0, size / 2.0);
  graphics.closePath();
  graphics.endFill();
  graphics.endStroke();

  createjs.Shape.call(this, graphics);

  if (pulse) {
    var expanding = true;
    var scale = 1;
    createjs.Ticker.addEventListener(
      "tick",
      function () {
        if (expanding) {
          scale *= 1.02;
          if (scale >= 1.05) {
            expanding = false;
          }
        } else {
          scale /= 1.02;
          if (scale <= 0.95) {
            expanding = true;
          }
        }
        this.scaleX = scale;
        this.scaleY = scale;
      }.bind(this)
    );
  }
};

ROS2D.NavigationArrow.prototype = Object.create(createjs.Shape.prototype);
ROS2D.NavigationArrow.prototype.constructor = ROS2D.NavigationArrow;

ROS2D.PathShape = function (options) {
  options = options || {};
  var path = options.path;
  this.strokeSize = options.strokeSize || 3;
  this.strokeColor = options.strokeColor || createjs.Graphics.getRGB(0, 0, 0);

  // draw the line
  this.graphics = new createjs.Graphics();

  if (path !== null && typeof path !== "undefined") {
    this.graphics.setStrokeStyle(this.strokeSize);
    this.graphics.beginStroke(this.strokeColor);
    this.graphics.moveTo(
      path.poses[0].pose.position.x / this.scaleX,
      path.poses[0].pose.position.y / -this.scaleY
    );
    for (var i = 1; i < path.poses.length; ++i) {
      this.graphics.lineTo(
        path.poses[i].pose.position.x / this.scaleX,
        path.poses[i].pose.position.y / -this.scaleY
      );
    }
    this.graphics.endStroke();
  }

  // create the shape
  createjs.Shape.call(this, this.graphics);
};

/**
 * Set the path to draw
 *
 * @param path of type nav_msgs/Path
 */
ROS2D.PathShape.prototype.setPath = function (path) {
  this.graphics.clear();
  if (path !== null && typeof path !== "undefined") {
    this.graphics.setStrokeStyle(this.strokeSize);
    this.graphics.beginStroke(this.strokeColor);
    this.graphics.moveTo(
      path.poses[0].pose.position.x / this.scaleX,
      path.poses[0].pose.position.y / -this.scaleY
    );
    for (var i = 1; i < path.poses.length; ++i) {
      this.graphics.lineTo(
        path.poses[i].pose.position.x / this.scaleX,
        path.poses[i].pose.position.y / -this.scaleY
      );
    }
    this.graphics.endStroke();
  }
};

ROS2D.PathShape.prototype.__proto__ = createjs.Shape.prototype;

ROS2D.OccupancyGrid = function (options) {
  options = options || {};
  var message = options.message;

  // internal drawing canvas
  var canvas = document.createElement("canvas");
  var context = canvas.getContext("2d");

  // save the metadata we need
  this.pose = new ROSLIB.Pose({
    position: message.info.origin.position,
    orientation: message.info.origin.orientation,
  });

  // set the size
  this.width = message.info.width;
  this.height = message.info.height;
  canvas.width = this.width;
  canvas.height = this.height;

  // Check if width and height are valid numbers
  if (!(this.width > 0) || !(this.height > 0)) {
    console.error("Invalid width or height:", this.width, this.height);
    return;
  }

  var imageData = context.createImageData(this.width, this.height);
  var data = message.data;
  var imageDataArray = imageData.data;
  var width = this.width;
  var height = this.height;

  for (var row = 0; row < height; row++) {
    for (var col = 0; col < width; col++) {
      var mapI = col + (height - row - 1) * width;
      var val;
      switch (data[mapI]) {
        case 100:
          val = 0;
          break;
        case 0:
          val = 255;
          break;
        default:
          val = 127;
      }

      var i = (col + row * width) * 4;
      imageDataArray[i] = val; // r
      imageDataArray[i + 1] = val; // g
      imageDataArray[i + 2] = val; // b
      imageDataArray[i + 3] = 255; // a
    }
  }
  context.putImageData(imageData, 0, 0);

  // create the bitmap
  createjs.Bitmap.call(this, canvas);
  // change Y direction
  this.y = -this.height * message.info.resolution;

  // scale the image
  this.scaleX = message.info.resolution;
  this.scaleY = message.info.resolution;
  this.width *= this.scaleX;
  this.height *= this.scaleY;

  // set the pose
  this.x += this.pose.position.x;
  this.y -= this.pose.position.y;
};

ROS2D.OccupancyGrid.prototype.__proto__ = createjs.Bitmap.prototype;

ROS2D.OccupancyGridClient = function (options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var topic = options.topic || "/map";
  this.continuous = options.continuous;
  this.rootObject = options.rootObject || new createjs.Container();

  // current grid that is displayed
  this.currentGrid = new createjs.Shape();
  this.rootObject.addChild(this.currentGrid);
  this.rootObject.addChild(new ROS2D.Grid({ size: 1 }));

  var rosTopic = new ROSLIB.Topic({
    ros: ros,
    name: topic,
    messageType: "nav_msgs/OccupancyGrid",
  });

  let isUpdating = false;
  const updateMap = (message) => {
    if (isUpdating) return;
    isUpdating = true;
    requestAnimationFrame(() => {
      if (that.currentGrid) {
        that.rootObject.removeChild(that.currentGrid);
      }
      that.currentGrid = new ROS2D.OccupancyGrid({ message: message });
      that.rootObject.addChildAt(that.currentGrid, 0);
      that.emit("change");
      isUpdating = false;
    });
  };

  rosTopic.subscribe(updateMap);
};

ROS2D.OccupancyGridClient.prototype.__proto__ = EventEmitter2.prototype;

ROS2D.OccupancyGridSrvClient = function (t) {
  var e = this,
    i = (t = t || {}).ros,
    s = t.service || "/static_map";
  this.rootObject = t.rootObject || new createjs.Container();
  this.currentGrid = null;
  new ROSLIB.Service({
    ros: i,
    name: s,
    serviceType: "nav_msgs/GetMap",
  }).callService(new ROSLIB.ServiceRequest(), function (t) {
    e.currentGrid && e.rootObject.removeChild(e.currentGrid);
    e.currentGrid = new ROS2D.OccupancyGrid({ message: t.map });
    e.rootObject.addChild(e.currentGrid);
    e.emit("change", e.currentGrid);
  });
};
ROS2D.OccupancyGridSrvClient.prototype.__proto__ = EventEmitter2.prototype;

ROS2D.ArrowShape = function (options) {
  var that = this;
  options = options || {};
  var size = options.size || 10;
  var strokeSize = options.strokeSize || 3;
  var strokeColor = options.strokeColor || createjs.Graphics.getRGB(0, 0, 0);
  var fillColor = options.fillColor || createjs.Graphics.getRGB(255, 0, 0);
  var pulse = options.pulse;

  // draw the arrow
  var graphics = new createjs.Graphics();

  var headLen = size / 3.0;
  var headWidth = (headLen * 2.0) / 3.0;

  graphics.setStrokeStyle(strokeSize);
  graphics.beginStroke(strokeColor);
  graphics.moveTo(0, 0);
  graphics.lineTo(size - headLen, 0);

  graphics.beginFill(fillColor);
  graphics.moveTo(size, 0);
  graphics.lineTo(size - headLen, headWidth / 2.0);
  graphics.lineTo(size - headLen, -headWidth / 2.0);
  graphics.closePath();
  graphics.endFill();
  graphics.endStroke();

  // create the shape
  createjs.Shape.call(this, graphics);

  // check if we are pulsing
  if (pulse) {
    // have the model "pulse"
    var growCount = 0;
    var growing = true;
    createjs.Ticker.addEventListener("tick", function () {
      if (growing) {
        that.scaleX *= 1.035;
        that.scaleY *= 1.035;
        growing = ++growCount < 10;
      } else {
        that.scaleX /= 1.035;
        that.scaleY /= 1.035;
        growing = --growCount < 0;
      }
    });
  }
};
ROS2D.ArrowShape.prototype.__proto__ = createjs.Shape.prototype;

ROS2D.Grid = function (t) {
  var e = (t = t || {}).size || 10,
    i = t.cellSize || 0.1,
    t = t.lineWidth || 0.001,
    s = new createjs.Graphics();
  s.setStrokeStyle(5 * t);
  s.beginStroke(createjs.Graphics.getRGB(0, 0, 0));
  s.beginFill(createjs.Graphics.getRGB(255, 0, 0));
  s.moveTo(-e * i, 0);
  s.lineTo(e * i, 0);
  s.moveTo(0, -e * i);
  s.lineTo(0, e * i);
  s.endFill();
  s.endStroke();
  s.setStrokeStyle(t);
  s.beginStroke(createjs.Graphics.getRGB(0, 0, 0));
  s.beginFill(createjs.Graphics.getRGB(255, 0, 0));
  for (var o = -e; o <= e; o++)
    s.moveTo(-e * i, o * i),
      s.lineTo(e * i, o * i),
      s.moveTo(o * i, -e * i),
      s.lineTo(o * i, e * i);
  s.endFill();
  s.endStroke();
  createjs.Shape.call(this, s);
};
ROS2D.Grid.prototype.__proto__ = createjs.Shape.prototype;

ROS2D.NavPath = function (t) {
  var e = (t = t || {}).size || 10,
    t = t.strokeSize || 3,
    i = new createjs.Graphics();
  i.setStrokeStyle(t);
  i.beginStroke(createjs.Graphics.getRGB(255, 64, 128));
  for (var s = 0; s < e.length - 1; s++) {
    var o = e[s],
      n = e[s + 1];
    i.moveTo(o.pose.position.x, -o.pose.position.y);
    i.lineTo(n.pose.position.x, -n.pose.position.y);
  }
  i.endStroke();
  createjs.Shape.call(this, i);
};
ROS2D.NavPath.prototype.__proto__ = createjs.Shape.prototype;

ROS2D.Viewer = function (t) {
  t = t || {};
  var e = t.width || 400,
    i = t.height || 300,
    s = t.background || "#111111",
    t =
      ((this.scene = new createjs.Stage(document.createElement("canvas"))),
      (this.scene.canvas.width = e),
      (this.scene.canvas.height = i),
      (this.scene.canvas.style.backgroundColor = s),
      this.scene);
  var element = document.getElementById(t);
  if (element) {
    element.appendChild(this.scene.canvas);
  } else {
    console.error("Element dengan ID '" + t + "' tidak ditemukan dalam DOM.");
  }
  document.getElementById(t).appendChild(this.scene.canvas);
  var o = new createjs.Container();
  this.scene.addChild(o),
    (this.scene.scaleX = 1),
    (this.scene.scaleY = 1),
    (this.scene.x = 0),
    (this.scene.y = 0),
    createjs.Ticker.setFPS(30),
    createjs.Ticker.addEventListener("tick", this.scene),
    (this.scaleToDimensions = function (t, e) {
      this.scene.scaleX = e / this.scene.canvas.width;
      this.scene.scaleY = t / this.scene.canvas.height;
      this.scene.x = this.scene.canvas.width / 2 - e / 2;
      this.scene.y = this.scene.canvas.height / 2 - t / 2;
    });
};
(ROS2D.Viewer = function (t) {
  var e = (t = t || {}).divID;
  (this.width = t.width), (this.height = t.height);
  var i = t.background || "#111111",
    t = document.createElement("canvas");
  (t.width = this.width),
    (t.height = this.height),
    (t.style.background = i),
    document.getElementById(e).appendChild(t),
    (this.scene = new createjs.Stage(t)),
    (this.scene.y = this.height),
    document.getElementById(e).appendChild(t),
    createjs.Ticker.setFPS(30),
    createjs.Ticker.addEventListener("tick", this.scene);
}),
  (ROS2D.Viewer.prototype.addObject = function (t) {
    this.scene.addChild(t);
  }),
  (ROS2D.Viewer.prototype.scaleToDimensions = function (t, e) {
    (this.scene.x =
      void 0 !== this.scene.x_prev_shift
        ? this.scene.x_prev_shift
        : this.scene.x),
      (this.scene.y =
        void 0 !== this.scene.y_prev_shift
          ? this.scene.y_prev_shift
          : this.scene.y),
      (this.scene.scaleX = this.width / t),
      (this.scene.scaleY = this.height / e);
  }),
  (ROS2D.Viewer.prototype.shift = function (t, e) {
    (this.scene.x_prev_shift = this.scene.x),
      (this.scene.y_prev_shift = this.scene.y),
      (this.scene.x -= t * this.scene.scaleX),
      (this.scene.y += e * this.scene.scaleY);
  });

ROS2D.LaserScan = function (options) {
  options = options || {};
  var ros = options.ros;
  var topic = options.topic || "/scan";
  var map = options.map; // Asumsi Anda telah memiliki akses ke objek peta

  this.rootObject = options.rootObject || new createjs.Container();
  this.points = [];

  var that = this;

  // Create a shape to draw the points
  this.scanShape = new createjs.Shape();
  this.rootObject.addChild(this.scanShape);

  // Set up the topic subscriber
  var scanListener = new ROSLIB.Topic({
    ros: ros,
    name: topic,
    messageType: "sensor_msgs/LaserScan",
    throttle_rate: options.throttle_rate || 66,
  });

  scanListener.subscribe(function (message) {
    console.log("Received LaserScan message");
    that.points = [];
    var angle = message.angle_min;

    for (var i = 0; i < message.ranges.length; i++) {
      var range = message.ranges[i];

      // Check for NaN or infinite values
      if (isNaN(range) || !isFinite(range)) {
        angle += message.angle_increment;
        continue;
      }

      // Calculate the coordinates of the point in laser frame
      var x = range * Math.cos(angle);
      var y = range * Math.sin(angle);

      // Check if the point is within the map boundaries
      if (isPointWithinMapBounds(x, y, map)) {
        that.points.push({ x: x, y: y });
      }

      angle += message.angle_increment;
    }

    that.drawScan();
  });

  // Function to check if a point is within map boundaries
  function isPointWithinMapBounds(x, y, map) {
    // Implement your logic here to check if the point is within the map boundaries
    // You may need to use information from the map object
    // For example, if you have access to the map dimensions, you can check if the point falls within those dimensions
    // You can also use other map information such as occupancy grid data to further refine this check
    return true; // Placeholder return value, replace with your actual implementation
  }

  this.drawScan = function () {
    var g = that.scanShape.graphics;
    g.clear();

    // Set the fill color to red
    g.beginFill(createjs.Graphics.getRGB(255, 0, 0));

    // Draw a small rectangle at each point
    for (var i = 0; i < that.points.length; i++) {
      var point = that.points[i];
      g.drawRect(point.x - 0.01, -point.y - 0.01, 0.02, 0.02);
    }

    g.endFill();
  };
};
