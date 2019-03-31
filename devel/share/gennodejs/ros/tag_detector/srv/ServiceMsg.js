// Auto-generated. Do not edit!

// (in-package tag_detector.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ServiceMsgRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.size1 = null;
      this.size2 = null;
    }
    else {
      if (initObj.hasOwnProperty('size1')) {
        this.size1 = initObj.size1
      }
      else {
        this.size1 = 0;
      }
      if (initObj.hasOwnProperty('size2')) {
        this.size2 = initObj.size2
      }
      else {
        this.size2 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ServiceMsgRequest
    // Serialize message field [size1]
    bufferOffset = _serializer.int32(obj.size1, buffer, bufferOffset);
    // Serialize message field [size2]
    bufferOffset = _serializer.int32(obj.size2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ServiceMsgRequest
    let len;
    let data = new ServiceMsgRequest(null);
    // Deserialize message field [size1]
    data.size1 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [size2]
    data.size2 = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tag_detector/ServiceMsgRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c63cae4a9bc1989d6d22ddaf34a9e93b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 size1
    int32 size2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ServiceMsgRequest(null);
    if (msg.size1 !== undefined) {
      resolved.size1 = msg.size1;
    }
    else {
      resolved.size1 = 0
    }

    if (msg.size2 !== undefined) {
      resolved.size2 = msg.size2;
    }
    else {
      resolved.size2 = 0
    }

    return resolved;
    }
};

class ServiceMsgResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.isSetOK = null;
    }
    else {
      if (initObj.hasOwnProperty('isSetOK')) {
        this.isSetOK = initObj.isSetOK
      }
      else {
        this.isSetOK = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ServiceMsgResponse
    // Serialize message field [isSetOK]
    bufferOffset = _serializer.bool(obj.isSetOK, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ServiceMsgResponse
    let len;
    let data = new ServiceMsgResponse(null);
    // Deserialize message field [isSetOK]
    data.isSetOK = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tag_detector/ServiceMsgResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c864c7a1d4f7d9d28b33390ae7e7d71d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool isSetOK
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ServiceMsgResponse(null);
    if (msg.isSetOK !== undefined) {
      resolved.isSetOK = msg.isSetOK;
    }
    else {
      resolved.isSetOK = false
    }

    return resolved;
    }
};

module.exports = {
  Request: ServiceMsgRequest,
  Response: ServiceMsgResponse,
  md5sum() { return 'b8e47ef395f4ff61960de951a76c82dd'; },
  datatype() { return 'tag_detector/ServiceMsg'; }
};
