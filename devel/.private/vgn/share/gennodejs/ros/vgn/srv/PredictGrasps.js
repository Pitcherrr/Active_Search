// Auto-generated. Do not edit!

// (in-package vgn.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

let GraspConfig = require('../msg/GraspConfig.js');

//-----------------------------------------------------------

class PredictGraspsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.voxel_size = null;
      this.map_cloud = null;
    }
    else {
      if (initObj.hasOwnProperty('voxel_size')) {
        this.voxel_size = initObj.voxel_size
      }
      else {
        this.voxel_size = 0.0;
      }
      if (initObj.hasOwnProperty('map_cloud')) {
        this.map_cloud = initObj.map_cloud
      }
      else {
        this.map_cloud = new sensor_msgs.msg.PointCloud2();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PredictGraspsRequest
    // Serialize message field [voxel_size]
    bufferOffset = _serializer.float64(obj.voxel_size, buffer, bufferOffset);
    // Serialize message field [map_cloud]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.map_cloud, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PredictGraspsRequest
    let len;
    let data = new PredictGraspsRequest(null);
    // Deserialize message field [voxel_size]
    data.voxel_size = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [map_cloud]
    data.map_cloud = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.map_cloud);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'vgn/PredictGraspsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '40b2d5c19b1ba0f3eae539bbfbf252ac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 voxel_size
    sensor_msgs/PointCloud2 map_cloud
    
    ================================================================================
    MSG: sensor_msgs/PointCloud2
    # This message holds a collection of N-dimensional points, which may
    # contain additional information such as normals, intensity, etc. The
    # point data is stored as a binary blob, its layout described by the
    # contents of the "fields" array.
    
    # The point cloud data may be organized 2d (image-like) or 1d
    # (unordered). Point clouds organized as 2d images may be produced by
    # camera depth sensors such as stereo or time-of-flight.
    
    # Time of sensor data acquisition, and the coordinate frame ID (for 3d
    # points).
    Header header
    
    # 2D structure of the point cloud. If the cloud is unordered, height is
    # 1 and width is the length of the point cloud.
    uint32 height
    uint32 width
    
    # Describes the channels and their layout in the binary data blob.
    PointField[] fields
    
    bool    is_bigendian # Is this data bigendian?
    uint32  point_step   # Length of a point in bytes
    uint32  row_step     # Length of a row in bytes
    uint8[] data         # Actual point data, size is (row_step*height)
    
    bool is_dense        # True if there are no invalid points
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: sensor_msgs/PointField
    # This message holds the description of one point entry in the
    # PointCloud2 message format.
    uint8 INT8    = 1
    uint8 UINT8   = 2
    uint8 INT16   = 3
    uint8 UINT16  = 4
    uint8 INT32   = 5
    uint8 UINT32  = 6
    uint8 FLOAT32 = 7
    uint8 FLOAT64 = 8
    
    string name      # Name of field
    uint32 offset    # Offset from start of point struct
    uint8  datatype  # Datatype enumeration, see above
    uint32 count     # How many elements in the field
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PredictGraspsRequest(null);
    if (msg.voxel_size !== undefined) {
      resolved.voxel_size = msg.voxel_size;
    }
    else {
      resolved.voxel_size = 0.0
    }

    if (msg.map_cloud !== undefined) {
      resolved.map_cloud = sensor_msgs.msg.PointCloud2.Resolve(msg.map_cloud)
    }
    else {
      resolved.map_cloud = new sensor_msgs.msg.PointCloud2()
    }

    return resolved;
    }
};

class PredictGraspsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.grasps = null;
    }
    else {
      if (initObj.hasOwnProperty('grasps')) {
        this.grasps = initObj.grasps
      }
      else {
        this.grasps = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PredictGraspsResponse
    // Serialize message field [grasps]
    // Serialize the length for message field [grasps]
    bufferOffset = _serializer.uint32(obj.grasps.length, buffer, bufferOffset);
    obj.grasps.forEach((val) => {
      bufferOffset = GraspConfig.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PredictGraspsResponse
    let len;
    let data = new PredictGraspsResponse(null);
    // Deserialize message field [grasps]
    // Deserialize array length for message field [grasps]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.grasps = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.grasps[i] = GraspConfig.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 64 * object.grasps.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'vgn/PredictGraspsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b55c9ab96e86d642dca84f4b80ed95b7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    vgn/GraspConfig[] grasps
    
    
    ================================================================================
    MSG: vgn/GraspConfig
    geometry_msgs/Pose pose
    float32 width
    float32 quality
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PredictGraspsResponse(null);
    if (msg.grasps !== undefined) {
      resolved.grasps = new Array(msg.grasps.length);
      for (let i = 0; i < resolved.grasps.length; ++i) {
        resolved.grasps[i] = GraspConfig.Resolve(msg.grasps[i]);
      }
    }
    else {
      resolved.grasps = []
    }

    return resolved;
    }
};

module.exports = {
  Request: PredictGraspsRequest,
  Response: PredictGraspsResponse,
  md5sum() { return 'c81c71905509ce157d8c512e48cd31ad'; },
  datatype() { return 'vgn/PredictGrasps'; }
};
