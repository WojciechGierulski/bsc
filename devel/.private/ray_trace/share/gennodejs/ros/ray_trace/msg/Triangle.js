// Auto-generated. Do not edit!

// (in-package ray_trace.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Triangle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.p1 = null;
      this.p2 = null;
      this.p3 = null;
    }
    else {
      if (initObj.hasOwnProperty('p1')) {
        this.p1 = initObj.p1
      }
      else {
        this.p1 = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('p2')) {
        this.p2 = initObj.p2
      }
      else {
        this.p2 = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('p3')) {
        this.p3 = initObj.p3
      }
      else {
        this.p3 = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Triangle
    // Serialize message field [p1]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.p1, buffer, bufferOffset);
    // Serialize message field [p2]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.p2, buffer, bufferOffset);
    // Serialize message field [p3]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.p3, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Triangle
    let len;
    let data = new Triangle(null);
    // Deserialize message field [p1]
    data.p1 = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [p2]
    data.p2 = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [p3]
    data.p3 = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 72;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ray_trace/Triangle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '480a47a04a0e0681cafbb5fbe734f2d4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point p1
    geometry_msgs/Point p2
    geometry_msgs/Point p3
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
    const resolved = new Triangle(null);
    if (msg.p1 !== undefined) {
      resolved.p1 = geometry_msgs.msg.Point.Resolve(msg.p1)
    }
    else {
      resolved.p1 = new geometry_msgs.msg.Point()
    }

    if (msg.p2 !== undefined) {
      resolved.p2 = geometry_msgs.msg.Point.Resolve(msg.p2)
    }
    else {
      resolved.p2 = new geometry_msgs.msg.Point()
    }

    if (msg.p3 !== undefined) {
      resolved.p3 = geometry_msgs.msg.Point.Resolve(msg.p3)
    }
    else {
      resolved.p3 = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = Triangle;
