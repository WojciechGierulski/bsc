// Auto-generated. Do not edit!

// (in-package ray_trace.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class ray_traceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vertices = null;
      this.faces = null;
      this.rays = null;
      this.origins = null;
    }
    else {
      if (initObj.hasOwnProperty('vertices')) {
        this.vertices = initObj.vertices
      }
      else {
        this.vertices = [];
      }
      if (initObj.hasOwnProperty('faces')) {
        this.faces = initObj.faces
      }
      else {
        this.faces = [];
      }
      if (initObj.hasOwnProperty('rays')) {
        this.rays = initObj.rays
      }
      else {
        this.rays = [];
      }
      if (initObj.hasOwnProperty('origins')) {
        this.origins = initObj.origins
      }
      else {
        this.origins = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ray_traceRequest
    // Serialize message field [vertices]
    // Serialize the length for message field [vertices]
    bufferOffset = _serializer.uint32(obj.vertices.length, buffer, bufferOffset);
    obj.vertices.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [faces]
    // Serialize the length for message field [faces]
    bufferOffset = _serializer.uint32(obj.faces.length, buffer, bufferOffset);
    obj.faces.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [rays]
    // Serialize the length for message field [rays]
    bufferOffset = _serializer.uint32(obj.rays.length, buffer, bufferOffset);
    obj.rays.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [origins]
    // Serialize the length for message field [origins]
    bufferOffset = _serializer.uint32(obj.origins.length, buffer, bufferOffset);
    obj.origins.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ray_traceRequest
    let len;
    let data = new ray_traceRequest(null);
    // Deserialize message field [vertices]
    // Deserialize array length for message field [vertices]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.vertices = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.vertices[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [faces]
    // Deserialize array length for message field [faces]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.faces = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.faces[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [rays]
    // Deserialize array length for message field [rays]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.rays = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.rays[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [origins]
    // Deserialize array length for message field [origins]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.origins = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.origins[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.vertices.length;
    length += 24 * object.faces.length;
    length += 24 * object.rays.length;
    length += 24 * object.origins.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ray_trace/ray_traceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6f38a86bd0beb8c650e2c1d9ff788e3c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point[] vertices
    geometry_msgs/Point[] faces
    geometry_msgs/Point[] rays
    geometry_msgs/Point[] origins
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ray_traceRequest(null);
    if (msg.vertices !== undefined) {
      resolved.vertices = new Array(msg.vertices.length);
      for (let i = 0; i < resolved.vertices.length; ++i) {
        resolved.vertices[i] = geometry_msgs.msg.Point.Resolve(msg.vertices[i]);
      }
    }
    else {
      resolved.vertices = []
    }

    if (msg.faces !== undefined) {
      resolved.faces = new Array(msg.faces.length);
      for (let i = 0; i < resolved.faces.length; ++i) {
        resolved.faces[i] = geometry_msgs.msg.Point.Resolve(msg.faces[i]);
      }
    }
    else {
      resolved.faces = []
    }

    if (msg.rays !== undefined) {
      resolved.rays = new Array(msg.rays.length);
      for (let i = 0; i < resolved.rays.length; ++i) {
        resolved.rays[i] = geometry_msgs.msg.Point.Resolve(msg.rays[i]);
      }
    }
    else {
      resolved.rays = []
    }

    if (msg.origins !== undefined) {
      resolved.origins = new Array(msg.origins.length);
      for (let i = 0; i < resolved.origins.length; ++i) {
        resolved.origins[i] = geometry_msgs.msg.Point.Resolve(msg.origins[i]);
      }
    }
    else {
      resolved.origins = []
    }

    return resolved;
    }
};

class ray_traceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.intersections = null;
    }
    else {
      if (initObj.hasOwnProperty('intersections')) {
        this.intersections = initObj.intersections
      }
      else {
        this.intersections = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ray_traceResponse
    // Serialize message field [intersections]
    // Serialize the length for message field [intersections]
    bufferOffset = _serializer.uint32(obj.intersections.length, buffer, bufferOffset);
    obj.intersections.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ray_traceResponse
    let len;
    let data = new ray_traceResponse(null);
    // Deserialize message field [intersections]
    // Deserialize array length for message field [intersections]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.intersections = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.intersections[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.intersections.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ray_trace/ray_traceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'df28a08516f5405586d6e6e2f427f310';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point[] intersections
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ray_traceResponse(null);
    if (msg.intersections !== undefined) {
      resolved.intersections = new Array(msg.intersections.length);
      for (let i = 0; i < resolved.intersections.length; ++i) {
        resolved.intersections[i] = geometry_msgs.msg.Point.Resolve(msg.intersections[i]);
      }
    }
    else {
      resolved.intersections = []
    }

    return resolved;
    }
};

module.exports = {
  Request: ray_traceRequest,
  Response: ray_traceResponse,
  md5sum() { return '81c2826f4b1acbbe34aaa616c1b2d0b8'; },
  datatype() { return 'ray_trace/ray_trace'; }
};
