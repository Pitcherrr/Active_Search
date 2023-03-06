// Auto-generated. Do not edit!

// (in-package active_grasp.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SeedRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.seed = null;
    }
    else {
      if (initObj.hasOwnProperty('seed')) {
        this.seed = initObj.seed
      }
      else {
        this.seed = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SeedRequest
    // Serialize message field [seed]
    bufferOffset = _serializer.uint32(obj.seed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SeedRequest
    let len;
    let data = new SeedRequest(null);
    // Deserialize message field [seed]
    data.seed = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'active_grasp/SeedRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '56fd131f75eb969fd4909892624a2fdc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 seed
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SeedRequest(null);
    if (msg.seed !== undefined) {
      resolved.seed = msg.seed;
    }
    else {
      resolved.seed = 0
    }

    return resolved;
    }
};

class SeedResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SeedResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SeedResponse
    let len;
    let data = new SeedResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'active_grasp/SeedResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SeedResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SeedRequest,
  Response: SeedResponse,
  md5sum() { return '56fd131f75eb969fd4909892624a2fdc'; },
  datatype() { return 'active_grasp/Seed'; }
};
